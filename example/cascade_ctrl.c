/**
 * @file main.c
 * @author york (york-fu@outlook.com)
 * @brief Example of elmo's ecm
 * @version 0.1
 * @date 2021-09-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <signal.h>
#include <unistd.h>
#include "elmo_motor.h"
#include "util.h"

#define NUM_MOTOR_MAX 6
#define BIT_17 (1 << 17)
#define MAX_CURRENT 10

static uint8_t th_run = 0;
static double dt = 1e-3;

MotorConfig_t motor_cfg;

uint8_t ids[NUM_MOTOR_MAX];
MotorData_t m_data[NUM_MOTOR_MAX];
MotorData_t m_data2[NUM_MOTOR_MAX];
MotorData_t m_data_d[NUM_MOTOR_MAX];
MotorData_t m_data_d2[NUM_MOTOR_MAX];

double initial_pos[NUM_MOTOR_MAX] = {0};

PIDParam_t v_ctl[NUM_MOTOR_MAX];
PIDParam_t p_ctl[NUM_MOTOR_MAX];

void set_pos(double pos, MotorData_t *data, MotorData_t *data2, double dt)
{
  data2->pos = data->pos;
  data2->vel = data->vel;
  data->pos = pos;
  data->vel = (data->pos - data2->pos) / dt;
  data->acc = (data->vel - data2->vel) / dt;
}

void set_vel(double vel, MotorData_t *data, MotorData_t *data2, double dt)
{
  data2->vel = data->vel;
  data->vel = vel;
  data->acc = (data->vel - data2->vel) / dt;
}

void load_controller_parameters()
{
  for (uint16_t i = 0; i < NUM_MOTOR_MAX; i++)
  {
    v_ctl[i].intergral_lim[0] = -1e3;
    v_ctl[i].intergral_lim[1] = 1e3;
    v_ctl[i].kp = 0.05;
    v_ctl[i].ki = 0;

    p_ctl[i].intergral_lim[0] = -1e3;
    p_ctl[i].intergral_lim[1] = 1e3;
    p_ctl[i].kp = 1;
  }
}

double v_control(uint16_t i, MotorData_t *desire, const MotorData_t *state)
{
  return PI_controller(&v_ctl[i], desire->vel - state->vel);
}

double p_control(uint16_t i, MotorData_t *desire, const MotorData_t *state)
{
  double v = P_controller(&p_ctl[i], desire->pos - state->pos);
  desire->vel += v;
  return v_control(i, desire, state);
}

static double cos_traj(double start, double stop, double T, double t)
{
  double A = (stop - start) / 2.0;
  return A * -cos(M_PI / T * t) + start + A;
}

void go_pos(uint8_t *ids, uint8_t num, double *pos_d, double speed, double dt)
{
  em_get_data(ids, num, m_data);
  double pos[num];
  double T[num];
  for (uint32 i = 0; i < num; i++)
  {
    pos[i] = m_data[i].pos;
    T[i] = fabs(pos_d[i] - pos[i]) / speed;
    printf("Motor %d from %f to %f\n", i + 1, pos[i], pos_d[i]);
  }
  double max_T = T[0];
  for (uint32 i = 1; i < num; i++)
  {
    if (max_T < T[i])
    {
      max_T = T[i];
    }
  }
  if (max_T < 0.5)
  {
    max_T = 0.5;
  }
  printf("Duration %f\n", max_T);
  double t = 0;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (1)
  {
    em_get_data(ids, num, m_data);
    for (uint32 i = 0; i < num; i++)
    {
      set_pos(cos_traj(pos[i], pos_d[i], max_T, t), &m_data_d[i], &m_data_d2[i], dt);
      m_data_d[i].tau = p_control(i, &m_data_d[i], &m_data[i]);
      m_data_d[i].tau_ff = 0;
      m_data_d[i].max_tau = MAX_CURRENT;
    }
    em_set_torques(ids, num, m_data_d);

    t += dt;
    if (t > max_T)
    {
      for (uint32 i = 0; i < num; i++)
      {
        m_data_d[i].tau = 0;
        m_data_d[i].tau_ff = 0;
        m_data_d[i].max_tau = MAX_CURRENT;
      }
      em_set_torques(ids, num, m_data_d);
      osal_usleep(2000);
      em_get_data(ids, num, m_data);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void go_vel(uint8_t *ids, uint8_t num, double *vel_d, double acc, double dt)
{
  em_get_data(ids, num, m_data);
  double vel[num];
  double T[num];
  for (uint32 i = 0; i < num; i++)
  {
    vel[i] = m_data[i].vel;
    T[i] = fabs(vel_d[i] - vel[i]) / acc;
    printf("Motor %d from %f to %f\n", i + 1, vel[i], vel_d[i]);
  }
  double max_T = T[0];
  for (uint32 i = 1; i < num; i++)
  {
    if (max_T < T[i])
    {
      max_T = T[i];
    }
  }
  if (max_T < 0.5)
  {
    max_T = 0.5;
  }
  printf("Duration %f\n", max_T);
  double t = 0;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (1)
  {
    em_get_data(ids, num, m_data);
    for (uint32 i = 0; i < num; i++)
    {
      set_vel(cos_traj(vel[i], vel_d[i], max_T, t), &m_data_d[i], &m_data_d2[i], dt);
      m_data_d[i].tau = v_control(i, &m_data_d[i], &m_data[i]);
      m_data_d[i].tau_ff = 0;
      m_data_d[i].max_tau = MAX_CURRENT;
    }
    em_set_torques(ids, num, m_data_d);

    t += dt;
    if (t > max_T)
    {
      for (uint32 i = 0; i < num; i++)
      {
        m_data_d[i].tau = 0;
        m_data_d[i].tau_ff = 0;
        m_data_d[i].max_tau = MAX_CURRENT;
      }
      em_set_torques(ids, num, m_data_d);
      osal_usleep(2000);
      em_get_data(ids, num, m_data);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void update_data(MotorData_t *m_data, MotorData_t *m_data2, double dt)
{
  for (uint32_t i = 0; i < motor_cfg.num; i++)
  {
    m_data2[i] = m_data[i];
  }
  em_get_data(ids, motor_cfg.num, m_data);
  for (uint32_t i = 0; i < motor_cfg.num; i++)
  {
    m_data[i].acc = (m_data[i].vel - m_data2[i].vel) / dt;
  }
}

void pd_test(const MotorData_t *m_data, const MotorData_t *m_data2,
             MotorData_t *m_data_d, MotorData_t *m_data_d2, double dt)
{
  static double time = 0;
  WaveParam_t wp;
  wp.A = 90;
  wp.T = 4;

  double kp = 2, kd = 0.01;
  for (uint16_t i = 0; i < motor_cfg.num; i++)
  {
    wp.b = initial_pos[i];
    double pos = cos_wave(&wp, time);
    set_pos(pos, &m_data_d[i], &m_data_d2[i], dt);
    m_data_d[i].tau = kp * (m_data_d[i].pos - m_data[i].pos) + kd * (m_data_d[i].vel - m_data[i].vel);
    m_data_d[i].tau_ff = 0;
    m_data_d[i].max_tau = MAX_CURRENT;
  }
  em_set_torques(ids, motor_cfg.num, m_data_d);

  time += dt;
  if (time >= wp.T)
  {
    time = 0;
  }
}

double admittance_control(MotorData_t *desire, MotorData_t *state, double_t m, double_t b, double_t k, double_t dt)
{
  double spring = (desire->pos - state->pos) * k;
  double damping = (desire->vel - state->vel) * b;
  double inertia = (desire->acc - state->acc) * m;
  return (inertia + damping + spring);
}

void csv_test(const MotorData_t *m_data, const MotorData_t *m_data2,
              MotorData_t *m_data_d, MotorData_t *m_data_d2, double dt)
{
  static double time = 0;
  WaveParam_t wp;
  wp.A = 180;
  wp.T = 4;
  wp.b = 0;

  for (uint16_t i = 0; i < motor_cfg.num; i++)
  {
    double vel = sin_wave(&wp, time);
    set_vel(vel, &m_data_d[i], &m_data_d2[i], dt);
    m_data_d[i].tau = v_control(i, &m_data_d[i], &m_data[i]);
    m_data_d[i].tau_ff = 0;
    m_data_d[i].max_tau = MAX_CURRENT;
  }
  em_set_torques(ids, motor_cfg.num, m_data_d);

  time += dt;
  if (time >= wp.T)
  {
    time = 0;
  }
}

void csp_test(const MotorData_t *m_data, const MotorData_t *m_data2,
              MotorData_t *m_data_d, MotorData_t *m_data_d2, double dt)
{
  static double time = 0;
  WaveParam_t wp;
  wp.A = 90;
  wp.T = 4;

  for (uint16_t i = 0; i < motor_cfg.num; i++)
  {
    wp.b = initial_pos[i];
    double pos = cos_wave(&wp, time);
    set_pos(pos, &m_data_d[i], &m_data_d2[i], dt);
    m_data_d[i].tau = p_control(i, &m_data_d[i], &m_data[i]);
    m_data_d[i].tau_ff = 0;
    m_data_d[i].max_tau = MAX_CURRENT;
  }
  em_set_torques(ids, motor_cfg.num, m_data_d);

  time += dt;
  if (time >= wp.T)
  {
    time = 0;
  }
}

OSAL_THREAD_FUNC_RT control_thread(void *ptr)
{
  double dt = *(double *)ptr;

  double run_time;
  struct timespec real_time, last_time;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (th_run)
  {
    update_data(m_data, m_data2, dt);
    csp_test(m_data, m_data2, m_data_d, m_data_d2, dt);

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

    clock_gettime(CLOCK_MONOTONIC, &real_time);
    run_time = (real_time.tv_sec + real_time.tv_nsec * 1e-9) - (last_time.tv_sec + last_time.tv_nsec * 1e-9);
    last_time = real_time;
    if (run_time >= (dt * 1.4))
    {
      printf("timeout %f, run time: %f\n", dt, run_time);
    }
  }
}

int8_t load_config(ECMConfig_t *ec_cfg, MotorConfig_t *m_cfg)
{
  strcpy(ec_cfg->ifname, "enp8s0");
  ec_cfg->dt = 5e-4;

  m_cfg->num = 0;
  m_cfg->enable = 0;
  m_cfg->circle_unit = 360;
  for (uint16_t i = 0; i < NUM_MOTOR_MAX; i++)
  {
    m_cfg->gear[i] = 15.39;
    m_cfg->range[i] = BIT_17;
    m_cfg->offset[i] = 0;
  }

  for (uint16_t i = 0; i < NUM_MOTOR_MAX; i++)
  {
    ids[i] = i + 1;
  }
  printf("range: [");
  for (uint16_t i = 0; i < NUM_MOTOR_MAX - 1; i++)
  {
    printf("%d, ", motor_cfg.range[i]);
  }
  printf("%d]\n", motor_cfg.range[NUM_MOTOR_MAX - 1]);

  printf("gear: [");
  for (uint16_t i = 0; i < NUM_MOTOR_MAX - 1; i++)
  {
    printf("%f, ", motor_cfg.gear[i]);
  }
  printf("%f]\n", motor_cfg.gear[NUM_MOTOR_MAX - 1]);

  printf("offset: [");
  for (uint16_t i = 0; i < NUM_MOTOR_MAX - 1; i++)
  {
    printf("%f, ", motor_cfg.offset[i]);
  }
  printf("%f]\n", motor_cfg.offset[NUM_MOTOR_MAX - 1]);
}

void sigint_handler(int sig)
{
  th_run = 0;
  osal_usleep(5000);
  go_pos(ids, motor_cfg.num, initial_pos, 100, dt);
  em_deinit();
  printf("\nsignal exit.\n");
  exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
  signal(SIGINT, sigint_handler);

  int ret = 0;
  ECMConfig_t ecm_cfg;
  load_config(&ecm_cfg, &motor_cfg);
  ret = em_init(&ecm_cfg, &motor_cfg);
  if (ret != 0)
  {
    return -1;
  }

  ret = em_enable_all(2000, 0);
  if (ret != 0)
  {
    return -2;
  }

  load_controller_parameters();
  go_pos(ids, motor_cfg.num, initial_pos, 100, dt);

  th_run = 1;
  OSAL_THREAD_HANDLE thread;
  ret = osal_thread_create_rt(&thread, 204800, &control_thread, (void *)(&dt));
  if (ret != 1)
  {
    return -3;
  }

  while (th_run)
  {
    osal_usleep(5000);
  }

  return 0;
}
