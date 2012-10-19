#include <linux/module.h>
#include <linux/init.h>
#include <rtai.h>
#include <rtai_sched.h>
#include <rtai_fifos.h>
#include "/home/lswith/Documents/thesisrepo/ext2.h"
#include <rtai_shm.h>
#include <rtai_math.h>


float u[60], y[60];
static float x1[10];
int sim_t = 30;
double A[] = {1.0, -0.8, 0.4};
double B[] = {0.0, 0.0, 0.2, 0.3};
double simA[] = {1.0, -0.5, 1.0, 0.00002983};
double simB[] = {0.0, 0.0, -1.0, 1.5};
int pdim[] = {4, 3};
int simpdim[] = {4,4};
int edim[] = {6, 6};


void ab(float xdot[], float x[], float u)
{
  xdot[0] = -0.4615*x[0] + 0.8871*x[1];
  xdot[1] = -0.5870*x[0] + 0.0856*x[1] - 1.3013*x[2] + 0.9887*u;
  xdot[2] = -0.2405*x[0] + 0.3628*x[1] - 0.1241*x[2] + 1.2339*u;
}

void cd(float x[], float u)
{
  y[sim_t] = -1.1402*x[0];
}

void rk(float dt, int n)
{
  float s1[10], s2[10], s3[10];
  float x2[10], x3[10];
  int j;

  // NB 1 second time3delay, use u[t-2]
  ab(s1, x1, u[sim_t-1]);
  for (j = 0; j < n; j++) x2[j] = x1[j] + s1[j]*dt;
  ab(s2, x2, u[sim_t-1]);
  for (j = 0; j < n; j++) x3[j] = x1[j] + 0.25*(s1[j]+s2[j])*dt;
  ab(s3, x3, u[sim_t-1]);
  for (j = 0; j < n; j++) x1[j] += dt*(s1[j]+4*s3[j]+s2[j])/6.0;
  cd(x1, u[sim_t-1]);
}

void init_pr(pr p)
{
  int i;

  for (i = 0; i < VECLEN; i++) {
    p->num[i] = 0.0; p->den[i] = 0.0;
  }
  for (i = 0; i < 2*VECLEN; i++) {
    p->y[i] = 0.0; p->u[i] = 0.0;
  }
  p->nn = simpdim[0]; p->nd = simpdim[1]; 
  for (i = 0; i < p->nd; i++) p->den[i] = simA[i];
  for (i = 0; i < p->nn; i++) p->num[i] = simB[i];

}
  
void init_est(est p)
{
  int i, ii, n;

  for (i = 0; i < VECLEN; i++) {
    p->theta[i] = 0.0; 
    p->a[i] = 0.0; p->b[i] = 0.0;
  }
  p->a[0] = 1.0;
  for (i = 0; i < CIRC; i++) {
    p->du[i] = 0.0; p->dy[i] = 0.0; p->yest[i] = 0.0;
  }
  for (i = 0; i < VECLEN*VECLEN; i++) p->P[i] = 0.0;
  p->nb = edim[0]; p->na = edim[1];
  n = p->na + p->nb - 2;
  for (i = 0, ii = 0; i < n; i++, ii += n) p->P[ii + i] = 10000.0; 
}

void init_ss(ss p)
{
  int i;

  for (i = 0; i < 500; i++) p->A[i] = 0.0;
  for (i = 0; i < VECLEN; i++) {
    p->B[i] = 0.0; p->C[i] = 0.0;
  }
}

double mtop[] = {0.0, 0.0, 0.075, 0.15, 0.075};
double mbot[] = {1.0, -0.7};
int mdim[] = {5, 2};

void init_pc(fb p)
{
  int i;

  for (i = 0; i < VECLEN; i++) {
    p->x[i] = 0.0; p->k[i] = 0.0;
    p->mnum[i] = 0.0; p->mden[i] = 0.0;
  }
  for (i = 0; i < CIRC; i++) {
    p->s[i] = 0.0; p->sf[i] = 0.0; p->e[i] = 0.0; p->du[i] = 0.0;
  }
  p->nn = mdim[0]; p->nd = mdim[1];
  for (i = 0; i < p->nn; i++) p->mnum[i] = mtop[i];
  for (i = 0; i < p->nd; i++) p->mden[i] = mbot[i];
}

int m_w = 123456;    /* must not be zero */
int m_z = 9876;    /* must not be zero */
 
unsigned int rand2()
{
    m_z = 36969 * (m_z & 65535) + (m_z >> 16);
    m_w = 18000 * (m_w & 65535) + (m_w >> 16);
    return (m_z << 16) + m_w;  /* 32-bit result */
}

/* Vector, matrix, polynomial operations */

int conv(double a[], double b[], double c[], int n, int nn)
{
  int m, i, j;

  m = n + nn -1;
  for (i = 0; i < m; i++) a[i] = 0.0;
  for (i = 0; i < n; i++)
    for (j = 0; j < nn; j++) a[i+j] += b[i] * c[j];
  return(m);
}

void filter(double y[], double num[], double den[], double u[], 
	    int t, int inum, int iden)
{
  int i;
  double yy = 0.0;

  for (i = 0; i < inum; i++) yy += num[i] * u[t-i];
  for (i = 1; i < iden; i++) yy -= den[i] * y[t-i];
  y[t] = yy;
}

/* Forms non-minimal state-space matrices */

double delta[] = {1, -1};

void nmss(ss pss,
	 double num[], int n1, double den[], int n2,
	 double mnum[], int n3, double mden[], int n4)
{
  int i,ii,j,k,na,nb,nc,enda,endb;
  double x[VECLEN], a[VECLEN], b[VECLEN], c[VECLEN];

  // convolution of vectors
  j = conv(x, delta, den, 2, n2);
  na = conv(a,x,mden,j,n4);
  nb = conv(b,num,mden,n1,n4);
  nc = conv(c,mnum,den,n3,n2);

  // first row A matrix
  k = 0;
  for (i = 1; i < na; i++,k++) pss->A[k] = -a[i];
  enda = k;
  for (i = 2; i < nb; i++,k++) pss->A[k] = b[i];
  endb = k;
  for (i = 1; i < nc; i++,k++) pss->A[k] = -c[i];

  // A, B, C matrices
  for (j = 0; j < k; j++) {
    for (i = 1, ii = k; i < k; i++, ii += k) {
      pss->A[ii + j] = 0.0;
      pss->B[i] = 0.0;
    }
    pss->C[j] = 0.0;
  }

  for (i = 1, ii = k; i < k; i++, ii += k) pss->A[ii + i-1] = 1.0;
  pss->B[0] = b[1];
  if (nb > 2) {
    pss->A[enda*k + enda-1] = 0.0;
    pss->B[enda] = 1.0;
  }
  pss->A[endb*k + endb-1] = 0.0;
  pss->C[0] = 1.0;
  pss->na = na; pss->nb = nb; pss->nc = nc; pss->n = k;
}

/* Householder transformation for 2 x n matrix */

void house(double A[], int n)
{
  int i, ir, ih, j, k, ka;
  double a[2], H[4], r[2*VECLEN], sigma;

  a[0] = A[0]; a[1] = A[n];
  sigma = sqrt(a[0]*a[0] + a[1]*a[1]);
  if (a[0] < 0.0) sigma = -sigma;
  a[0] += sigma;
  a[1] /= a[0]; 
  a[0] = 1.0;
  sigma = 2.0 / (a[0] * a[0] + a[1] * a[1]);
  H[0] = 1.0 - sigma * a[0] * a[0];
  H[1] = -sigma * a[0] * a[1];
  H[2] = -sigma * a[1] * a[0];
  H[3] = 1.0 - sigma * a[1] * a[1];
  for (i = 0, ir = 0, ih = 0; i < 2; i++, ir += n, ih += 2) {
    for (j = 0; j < n; j++) {
      r[ir + j] = 0.0;
      for (k = 0, ka = 0; k < 2; k++, ka += n)
	r[ir + j] += H[ih + k] * A[ka + j];
    }
  }
  for (i = 0; i < n+n; i++) A[i] = r[i];
}

/* Optimal controller feedback gains */

void lqg(double g[], double a[], double b[], double c[], 
	 int k, double qy, int reps)
{
  double sigma, p[VECLEN], d[VECLEN+VECLEN];
  int i, ii, j, jj, cols, n;

  cols = k+1;
  for (j = 0; j < cols+cols; j++) d[j] = 0.0;

  for (j = 0; j < k; j++) p[j] = c[j] * qy;
  d[0] = 1.0;
  for (n = 0; n < reps; n++) {
    sigma = 0.0;
    for (j = 0; j < k; j++) sigma += p[j] * b[j];
    d[cols] = sigma;
    for (j = 0, jj = cols+1; j < k; j++, jj++) {
      sigma = 0.0;
      for (i = 0, ii = i; i < k; i++, ii += k) sigma += p[i] * a[ii + j];
      d[jj] = sigma;
    }
    house(d, cols);
    for (j = 0, jj = cols+1; j < k; j++, jj++) p[j] = d[jj];
  }
  sigma = d[0];
  for (j = 0; j < k; j++) g[j] = d[j+1] / sigma;
}

/* Recursive Least Squares */

void recls(est p, int bump)
{
  int i, ii, k, j, jj;
  double phi[VECLEN], pphi[VECLEN], den, e, yhat, bb[VECLEN];
  double alpha, beta, gamma, lambda;

  // phi = [yest(t-1) yest(t-2) ... du(t-1) du(t-2) ...]
  for (i = 1, k = 0; i < p->na; i++) phi[k++] = p->yest[t-i];
  for (i = 1; i < p->nb; i++) phi[k++] = p->du[t-i];
  //  for (i = 0; i < k; i++) printf("  %5.3f",phi[i]);
  //  ii = getchar(); printf("\n");
  // Bump covariance matrix
  if (bump) for (i = 0, ii = 0; i < k; i++, ii += k) p->P[ii + i] += 2.0;

  // yhat(t) = phi'*theta;
  yhat = 0.0;
  for (i = 0; i < k; i++) {
    yhat += phi[i] * p->theta[i];
    pphi[i] = phi[i];
  }
  e = p->dy[t] - yhat;

  for (j = k-1, jj = j*k; j > 0; j--, jj -= k) {
    for (i = 0, ii = 0; i < j; i++, ii += k) {
      pphi[j] += p->P[ii + j] * pphi[i];
    }
    bb[j] = p->P[jj + j] * pphi[j];
  }
  bb[0] = p->P[0] * pphi[0];

  alpha = 1.0 + bb[0] * pphi[0];
  gamma = 1.0 / alpha;
  p->P[0] *= gamma;

  for (j = 1, jj = k; j < k; j++, jj += k) {
    beta = alpha;
    alpha += bb[j] * pphi[j];
    lambda = -pphi[j] * gamma;
    gamma = 1.0 / alpha;
    p->P[jj + j] *= beta * gamma;
    for (i = 0, ii = 0; i < j; i++, ii += k) {
      beta = p->P[ii + j];
      p->P[ii + j] = beta + bb[i] * lambda;
      bb[i] += bb[j] * beta;
    }
  }

  e *= gamma;
  for (j = 0; j < k; j++)
    p->theta[j] += bb[j] * e;

  // yest(t) = phi'*theta;
  yhat = 0.0;
  for (i = 0; i < k; i++) yhat += phi[i] * p->theta[i];
  p->yest[t] = yhat;
  // Model polynomials
  p->a[0] = 1.0; p->b[0] = 0.0;
  for (i = 1, ii = 0; i < p->na; i++) p->a[i] = -p->theta[ii++];
  for (i = 1; i < p->nb; i++) p->b[i] = p->theta[ii++];
}

double cont(fb p, double y, int t, int na, int nb, int nc)
{
  int i, j;

  // Filter setpoint
  filter(p->sf, p->mnum, p->mden, p->s, t, p->nn, p->nd);
  // Error
  p->e[t] = y - p->sf[t];
  // Differencing
  p->ds[t] = p->s[t] - p->s[t-1];
  // State vector
  j = 0;
  for (i = 0; i < na-1; i++) p->x[j++] = p->e[t - i];
  for (i = 1; i < nb-1; i++) p->x[j++] = p->du[t - i];
  for (i = 0; i < nc-1; i++) p->x[j++] = p->ds[t - i];
  // Actuation
  p->du[t] = 0.0;
  for (i = 0; i < j; i++) p->du[t] -= p->k[i] * p->x[i];
  return p->du[t];
}
