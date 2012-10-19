#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <tgmath.h>
#include "ext.h"

// double num[] = {0.0, 1.0};
// double den[] = {1, -0.8};
// double mnum[] = {0.0, 0.5};
// double mden[] = {1.0, -0.5};

double durand[] = {0,0,1,0,0,0,0,-1,0,0};

int t;
int sim_t;

void freq (void) {
    rk(1,3);
}

void timeUpdate (pr pp, est pe, ss pss, fb pc) {
    // Time update signals
    pp->y[t-VECLEN] = pp->y[t];
    pp->u[t-VECLEN] = pp->u[t];
    pe->du[t-VECLEN] = pe->du[t];
    pe->dy[t-VECLEN] = pe->dy[t];
    pe->yest[t-VECLEN] = pe->yest[t];
    pc->s[t-VECLEN] = pc->s[t];
    pc->sf[t-VECLEN] = pc->sf[t];
    pc->ds[t-VECLEN] = pc->ds[t];
    pc->e[t-VECLEN] = pc->e[t];
    pc->du[t-VECLEN] = pc->du[t];
    t++;
    if (t >= CIRC) t = VECLEN;
}

void timeUpdate2() {
    u[sim_t-VECLEN] = u[sim_t];
    y[sim_t-VECLEN] = y[sim_t];
    sim_t++;
    if (sim_t >= CIRC) sim_t = VECLEN;
}

void display (int i, pr pp, est pe, fb pc) {
    // Display
    int j;
    FILE *fp;
    printf("%d   %5.3f   %5.3f",i,u[sim_t],y[sim_t]);
    printf("   %5.3f   %5.3f",pc->s[t],pc->sf[t]);
    for (j = 0; j < pe->na; j++) printf("   %5.3f",pe->a[j]);
    for (j = 0; j < pe->nb; j++) printf("   %5.3f",pe->b[j]);
    printf("  %5.3f",y[t]);
    printf("\n");
    fp = fopen("out.dat","a");
    fprintf(fp,"%d   %5.3f   %5.3f",i,u[sim_t],y[sim_t]);
    fprintf(fp,"   %5.3f   %5.3f",pc->s[t],pc->sf[t]);
    for (j = 0; j < pe->na; j++) fprintf(fp,"   %5.3f",pe->a[j]);
    for (j = 0; j < pe->nb; j++) fprintf(fp,"   %5.3f",pe->b[j]);
    
    fprintf(fp,"  %5.3f",y[t]);
    fprintf(fp,"\n");
    fclose(fp);
}

int main()
{
  pr pp = malloc(sizeof(process));
  est pe = malloc(sizeof(estimates));
  ss pss = malloc(sizeof(statespace));
  fb pc = malloc(sizeof(control));
  int i, j, ii, bump;
  FILE *fp;
  //double dummy[] = {1.0, 0};
  double du;
  double max = RAND_MAX;
  double noise;

  // Initialise things
  fp = fopen("out.dat","w");
  fprintf(fp,"0 0 0 0 0 0 0 0 0 0 0 0\n");
  fclose(fp);
  t = VECLEN;
  sim_t = VECLEN;
  init_pr(pp);
  init_est(pe);
  init_ss(pss);
  init_pc(pc);
  bump = 0;
  srand(123456);
  // Main loop
  for (i = 0; i < 1000; i++) {
    sleep(1);

    
    // Simulate process - y(t)
    //filter(pp->y, pp->num, pp->den, pp->u, t, pp->nn, pp->nd);
    u[sim_t] = u[sim_t-1];
    rk(1,3);
    if ((i % 2) == 1) { 
        timeUpdate(pp,pe,pss,pc);      
        //pp->y[t] = y[sim_t] - y[sim_t-1]; 
        pp->y[t] = y[sim_t];
        noise = rand();
        pp->y[t] += 0.05 * noise / max;
        // Measurement vector for parest
        pe->dy[t] = pp->y[t] - pp->y[t-1];
        // Parest
        recls(pe, bump);
        bump = 0;
        // NMSS
        nmss(pss, pe->b, pe->nb, pe->a, pe->na, 
                pc->mnum, pc->nn, pc->mden, pc->nd);


        /*
           if (i == 90) {
           matprint(pss->A,pss->n,pss->n);
           vecprint(pss->B,pss->n);
           vecprint(pss->C,pss->n);
           printf("\n");
           }
           */

        // FB Gains
        lqg(pc->k,pss->A,pss->B,pss->C,pss->n,5.0,20);

        /*
           if (i == 90) {
           vecprint(pc->k,pss->n);
           printf("\n");
           ii = getchar();
           }
           */

        // Setpoint
        if (rand() < RAND_MAX/10) {
            pc->s[t] = 1.0 - pc->s[t];
            bump = 1;
        }
        else pc->s[t] = pc->s[t-1];
        // Controller
        if (i < 20) du = durand[(int) (i/2)];
        else du = cont(pc, pp->y[t], t, pss->na, pss->nb, pss->nc);
        // Integral action
        pp->u[t] = pp->u[t-1] + du;
        u[sim_t] = pp->u[t];
        // Actuation vector for parest
        pe->du[t] = du;
    } 
    display(i,pp, pe, pc);
    timeUpdate2();
  }
  // End loop

  /* MPC test
  i = nmss(a,b,c,num,2,den,2,mnum,2,mden,2);
  lqg(k,a,b,c,i,1.0,20);
  matprint(a,i,i);
  vecprint(b,i);
  vecprint(c,i);
  printf("\n");
  vecprint(k,i);
  */

  return 0;
}
