#include <linux/module.h>
#include <linux/init.h>
#include <rtai.h>
#include <rtai_sched.h>
#include <rtai_fifos.h>
#include <rtai_shm.h>
#include <rtai_math.h>
#include <rtai_nam2num.h>
#include <rtai_rwl.h>
#include "/home/lswith/Documents/thesisrepo/ext2.h"

#define ARG 0
#define STACK_SIZE 1024
#define SIM_PRIORITY 0
#define SAMP_PRIORITY 1
#define ACT_PRIORITY 1
#define EST_PRIORITY 2
#define NOW rt_get_time()
#define USE_FPU 1
#define SIM_PERIOD nano2count(1e8)
#define SAMP_PERIOD nano2count(3e8)
#define RTF_SIZE 60


RT_TASK thread_data;
RT_TASK thread_data2;
RT_TASK thread_data3;
RT_TASK thread_data4;

pr pp;
est pe;
ss pss;
fb pc;
int i, t;
double du;
int estimation_flag;
double durand[] = {0,0,1,0,0,0,0,-1,0,0};
double current_y;
double current_u;
double old_u;
double setpoint;
double fsetpoint;
int *mode;
int *setpointmode;
double *sp;
double *samplerate;
RTIME sampledTime;

void display (void) {
    static struct {
        int time;
        double u,y,s,sf;
        double alength;
        double blength;
        double a[VECLEN];
        double b[VECLEN];
        int mode;
    } disp;
    
    int j;
    disp.time = i;
    disp.u = current_u;
    disp.y = current_y;
    disp.s = setpoint;
    disp.sf = fsetpoint;
    disp.alength = pe->na;
    disp.blength = pe->nb;
    disp.mode = *mode;
    for (j = 0; j < pe->na; j++) {
        disp.a[j] = pe->a[j];
    }
    for (j = 0; j < pe->nb; j++) {
        disp.b[j] = pe->b[j];
    }
    rtf_put(DISPLAYFIFO,&disp,sizeof(disp));
}

void time_step(void)
{
    u[sim_t-30] = u[sim_t];
    y[sim_t-30] = y[sim_t];
    sim_t++;
    i++;
    if (sim_t >= 60) sim_t = 30;
}

void time_step2(void) {
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


void simulator (long arg) {

    double noise;
    int j;
    for (j = 0; j < 60; j++) {
        u[j] = 0.0; y[j] = 0.0; 
    }
    sim_t = VECLEN;
    while (1) {
        current_u = u[sim_t]; 
        display();
        time_step();
        u[sim_t] = u[sim_t-1];
        rk(1, 3);
        noise = (rand2() % 10000); 
        noise = noise / 20000;
        y[sim_t] += noise;
        current_y = y[sim_t]/10;
        rt_task_wait_period();
    }
}

int k = 0;
//void mpc (long arg) {
void mpc (void) {
    int bump = 0;
    int randomNum;
    //while (1) {
        pe->dy[t] = pp->y[t] - pp->y[t-1];

        if (*mode != CONTROLONLY) {
            // Parest
            recls(pe, bump);
            bump = 0;
            // NMSS
            nmss(pss, pe->b, pe->nb, pe->a, pe->na, 
                    pc->mnum, pc->nn, pc->mden, pc->nd);
            // FB Gains
            lqg(pc->k,pss->A,pss->B,pss->C,pss->n,5.0,20);
        }

        if (*setpointmode == USERAND) {
            // Random Setpoint
            randomNum = rand2() % 10;
            if (randomNum == 1) {
                pc->s[t] = 1.0 - pc->s[t];
                bump = 1;
            } else {
                pc->s[t] = pc->s[t-1];
            }
        } else {
            pc->s[t] = *sp;
        }

        // Controller
        if (k < 10) du = durand[k];
        else du = cont(pc, pp->y[t], t, pss->na, pss->nb, pss->nc);

        switch (*mode) {
            case MPCCONTROL:
                pp->u[t] = pp->u[t-1] + du;
                pe->du[t] = du;
                k++;
                break;

            case CONTROLONLY:
                pp->u[t] = pp->u[t-1] + du;
                pe->du[t] = du;
                k++;
                break;

            case MANUAL:
                pp->u[t] = pc->sf[t];
                pe->du[t] = pp->u[t] - pp->u[t-1];
                break;

            case GAINFEEDBACK:
                //TODO get basic gain
                pp->u[t] = -2*pc->e[t];
                printk("pc->e[t] = %d\n",(int) (pc->e[t]*1000));
                pe->du[t] = pp->u[t] - pp->u[t-1];
                break;

            default:
                pp->u[t] = pp->u[t-1];
                pe->du[t] = 0;
                break;
        } 


        setpoint = pc->s[t];
        fsetpoint = pc->sf[t];
        u[sim_t] = pp->u[t];

    //    rt_task_wait_period();
    //}        
}

void sampler (long arg) {
    while (1) {
        time_step2();
        pp->y[t] = current_y;
        mpc();
        rt_task_wait_period();
    }
}

static int sampleHandler(unsigned int fifo)
{
    double num;
    double temp;
    do {
        num = rtf_get(SAMPLERATEFIFO,&temp,sizeof(temp));
    } while (num != 0);

    printk("temp = %d\n",(int) (temp));
    *samplerate = temp;
    rt_set_period(&thread_data2,nano2count((1/temp)*1e9));
    return 0;
}

int init_module(void) {
    int i;
    sim_t = 30;

    pp = rt_shm_alloc(nam2num("pp"), sizeof(process),USE_VMALLOC);
    pe = rt_shm_alloc(nam2num("pe"), sizeof(estimates),USE_VMALLOC);
    pss = rt_shm_alloc(nam2num("pss"), sizeof(statespace),USE_VMALLOC);
    pc = rt_shm_alloc(nam2num("pc"), sizeof(control),USE_VMALLOC);
    sp = rt_shm_alloc(nam2num("sp"),sizeof(double),USE_VMALLOC);
    mode = rt_shm_alloc(nam2num("mode"),sizeof(int),USE_VMALLOC);
    setpointmode = rt_shm_alloc(nam2num("spmode"),sizeof(int),USE_VMALLOC);
    samplerate = rt_shm_alloc(nam2num("samprt"),sizeof(double),USE_VMALLOC);
    
    init_pr(pp);
    init_est(pe);
    init_ss(pss);
    init_pc(pc);
    t = VECLEN;
    i = 0;
    estimation_flag = 0;
    *mode = ESTIMATEONLY;
    *setpointmode = USERAND;
    *sp = 0; 
    setpoint = 0;
    fsetpoint = 0;
    *samplerate = 1/(count2nano(SAMP_PERIOD)/1e9); 
    rtf_create(DISPLAYFIFO, RTF_SIZE);
    rtf_reset(DISPLAYFIFO);

    rtf_create(SAMPLERATEFIFO, RTF_SIZE);
    rtf_reset(SAMPLERATEFIFO);
    rtf_create_handler(SAMPLERATEFIFO, sampleHandler);

    rt_set_oneshot_mode();
    start_rt_timer(SIM_PERIOD);

    rt_task_init(&thread_data, simulator,
            ARG, STACK_SIZE, SIM_PRIORITY, USE_FPU, NULL);
    rt_task_make_periodic(&thread_data, NOW, SIM_PERIOD);

    rt_task_init(&thread_data2, sampler,
            ARG, STACK_SIZE, SAMP_PRIORITY, USE_FPU, NULL);
    rt_task_make_periodic(&thread_data2, NOW, SAMP_PERIOD);

    //rt_task_init(&thread_data3, mpc,
    //        ARG, STACK_SIZE, SAMP_PRIORITY, USE_FPU, NULL);
    //rt_task_make_periodic(&thread_data3, NOW, SAMP_PERIOD);

    return 0;
}

void cleanup_module(void) {

    rt_shm_free(nam2num("pp"));
    rt_shm_free(nam2num("pe"));
    rt_shm_free(nam2num("pss"));
    rt_shm_free(nam2num("pc"));
    rt_shm_free(nam2num("sp"));
    rt_shm_free(nam2num("mode"));
    rt_shm_free(nam2num("spmode"));

    i = -1;
    display();
    stop_rt_timer();
    rt_task_delete(&thread_data);
    rt_task_delete(&thread_data2);
    rt_task_delete(&thread_data3);
    rt_task_delete(&thread_data4);

}
