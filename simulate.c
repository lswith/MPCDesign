#include <linux/kernel.h>
#include <linux/module.h>
#include <math.h>		//please ensure rtai_math.ko is insmoded
#include <asm/io.h>
#include <rtai.h>
#include <rtai_sched.h>
#include <rtai_fifos.h>

#define TIMERTICKS 1000000
#define CMDF0 0
#define CMDF1 1

static RT_TASK Task;

static float u[60], y[60], s[60], e[60], yf[60], du[60];
static float x1[10];
static int t = 30;

static struct {
  float sp;
  int kn, kd;
  float num[10], den[10];
  int n1, n2, n3, n;
  float K[40];
} par;

void spgains(int fifo)
{
  rtf_get(fifo, &par, sizeof(par));
}

void ab(float xdot[], float x[], float u)
{
  xdot[0] = -0.4615*x[0] + 0.8871*x[1];
  xdot[1] = -0.5870*x[0] + 0.0856*x[1] - 1.3013*x[2] + 0.9887*u;
  xdot[2] = -0.2405*x[0] + 0.3628*x[1] - 0.1241*x[2] + 1.2339*u;
}

void cd(float x[], float u)
{
  y[t] = -1.1402*x[0];
}

void rk(float dt, int n)
{
  float s1[10], s2[10], s3[10];
  float x2[10], x3[10];
  int j;

  // NB 1 second time delay, use u[t-2]
  ab(s1, x1, u[t-2]);
  for (j = 0; j < n; j++) x2[j] = x1[j] + s1[j]*dt;
  ab(s2, x2, u[t-2]);
  for (j = 0; j < n; j++) x3[j] = x1[j] + 0.25*(s1[j]+s2[j])*dt;
  ab(s3, x3, u[t-2]);
  for (j = 0; j < n; j++) x1[j] += dt*(s1[j]+4*s3[j]+s2[j])/6.0;
  cd(x1, u[t-2]);
}

void process(void)
{
  rk(1.0, 3);
}

void spfilt(float sp, int kn, float num[], int kd, float den[])
{
  int j;

  s[t] = sp;
  yf[t] = 0.0;
  for (j = 0; j < kn; j++) yf[t] += num[j]*s[t-j];
  for (j = 1; j < kd; j++) yf[t] -= den[j]*yf[t-j];
}

void control(float K[], int n1, int n2, int n3, int n)
{
  float x[40];
  int j, k;

  e[t] = y[t] - yf[t];
  k = 0;
  for (j = 0; j < n1; j++) x[k++] = e[t-j];
  for (j = 0; j < n2; j++) x[k++] = du[t-j-1];
  for (j = 0; j < n3; j++) x[k++] = s[t-j] - s[t-j-1];
  du[t] = 0.0;
  for (j = 0; j < n; j++) du[t] -= (K[j] * x[j]);
  u[t] = u[t-1] + du[t];
  // Testing
  // u[t] = s[t] - s[t-1];
}

void time_step(void)
{
  u[t-30] = u[t];
  y[t-30] = y[t];
  s[t-30] = s[t];
  e[t-30] = e[t];
  yf[t-30] = yf[t];
  du[t-30] = du[t];
  t++;
  if (t >= 60) t = 30;
}

static void Thread(int tt)
{
  static struct {
    RTIME time;
    float ms, my, myf, mu; 
  } msg;

  while (1) {
    msg.time = rt_get_cpu_time_ns();
    spfilt(par.sp, par.kn, par.num, par.kd, par.den);
    process();
    control(par.K, par.n1, par.n2, par.n3, par.n);
    msg.ms = s[t]; msg.my = y[t]; msg.myf = yf[t]; msg.mu = u[t];
    rtf_put(CMDF0, &msg, sizeof(msg));
    time_step();
    rt_task_wait_period();
  }
}

int init_module(void)
{
  int i;

  RTIME tick_period;
  RTIME now;

  t = 30;
  for (i = 0; i < 60; i++) {
    u[i] = 0.0; y[i] = 0.0; s[i] = 0.0;
    e[i] = 0.0; yf[i] = 0.0; du[i] = 0.0;
  }

  par.sp = 0.0;
  par.kn = 0; par.kd = 0;
  for (i = 0; i < 10; i++) {
    par.num[i] = 0.0; par.den[i] = 0.0;
  }
  par.n = 0;
  for (i = 0; i < 40; i++) par.K[i] = 0.0;

  rtf_create_using_bh(CMDF0, 20000, 0);
  rtf_create_using_bh(CMDF1, 20000, 0);
  rtf_create_handler(CMDF1, (void *)&spgains);
  rt_task_init(&Task, (void *)&Thread, 0, 2000, 0, 1, 0);
  tick_period = 1000*start_rt_timer(nano2count(TIMERTICKS));
  now = rt_get_time();
  rt_task_make_periodic(&Task, now + tick_period, tick_period);

  return 0;
}

void cleanup_module(void)
{
  stop_rt_timer();
  rt_busy_sleep(10000000);
  rtf_destroy(CMDF0);
  rtf_destroy(CMDF1);
  rt_task_delete(&Task);
}
