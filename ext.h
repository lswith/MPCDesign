#define VECLEN 30
#define CIRC 60
//#define RAND_MAX 4294967295 
#define SETPOINT_DEV "/dev/rtf0"
// Process
typedef struct {
  double num[VECLEN], den[VECLEN]; // numerator, denominator
  double y[CIRC], u[CIRC]; // signal circular buffers
  int nn, nd; // no. terms in numerator, denominator
} process, *pr;

// Least Squares
typedef struct {
  double P[VECLEN*VECLEN]; // covariance
  double theta[VECLEN]; // estimated params
  double a[VECLEN], b[VECLEN]; // model polynomials
  double du[CIRC], dy[CIRC], yest[CIRC]; // differenced data
  int na, nb; // number of model terms
} estimates, *est;

// NMSS
typedef struct {
  double A[500], B[VECLEN], C[VECLEN]; // nmss model
  int na, nb, nc, n; // # model terms
} statespace, *ss;

// Control
typedef struct {
  double x[VECLEN]; // state vector
  double k[VECLEN];  // feedback gains
  double mnum[VECLEN], mden[VECLEN]; // setpoint filter
  int nn, nd; // setpoint filter dimensions
  double s[CIRC], sf[CIRC], ds[CIRC]; // setpoint, filtered, differenced
  double e[CIRC], du[CIRC]; // error, actuation (differenced)
} control, *fb;

extern int t;
extern int sim_t;
extern float u[CIRC], y[CIRC];

int rand(void);
void ab(float xdot[], float x[], float u);
void cd(float x[], float u);
void rk(float dt, int n);
void rotate(double S[], int nu, int n);
int conv(double *a, double *b, double *c, int n, int nn);
void vecprint(double *a, int n);
void matprint(double a[], int n, int m);
void nmss(ss pss,
	 double num[], int n1, double den[], int n2,
	 double mnum[], int n3, double mden[], int n4);
void lqg(double g[], double a[], double b[], double c[], 
	 int k, double qy, int reps);
void filter(double y[], double num[], double den[], double u[], 
	    int t, int inum, int iden);
void init_pr(pr p);
void init_est(est p);
void recls(est p, int bump);
void init_ss(ss p);
void init_pc(fb p);
double cont(fb p, double y, int t, int na, int nb, int nc);
