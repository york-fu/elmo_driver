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
#define BIT_14 (1 << 14)
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
    for (uint32 i = 0; i < num; i++)
    {
      m_data_d[i].pos = cos_traj(pos[i], pos_d[i], max_T, t);
      m_data_d[i].max_tau = MAX_CURRENT;
    }
    em_set_positions(ids, num, m_data_d);

    t += dt;
    if (t > max_T)
    {
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
    for (uint32 i = 0; i < num; i++)
    {
      m_data_d[i].vel = cos_traj(vel[i], vel_d[i], max_T, t);
      m_data_d[i].max_tau = MAX_CURRENT;
    }
    em_set_velocities(ids, num, m_data_d);

    t += dt;
    if (t > max_T)
    {
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

OSAL_THREAD_FUNC_RT control_thread(void *ptr)
{
  double dt = *(double *)ptr;

  char *filename = "./data/p_cos_360_4.csv";
  FILE *fp = NULL;
  fp = fopen(filename, "w+");
  if (fp == NULL)
  {
    printf("Failed to open %s\n", filename);
    th_run = 0;
    return;
  }
  fprintf(fp, "p, v, t, p_d, v_d, t_d, p_ff, v_ff, t_ff\n");

  double time = 0;
  WaveParam_t wp;
  wp.A = 360;
  wp.T = 4;
  wp.b = initial_pos[0];

  uint32_t count = 0;
  uint32_t sample_size = ceil((wp.T * 3) / dt);
  MotorData_t sample_state[sample_size], sample_cmd[sample_size];

  em_get_data(ids, 1, m_data);
  m_data2[0] = m_data[0];
  m_data_d[0].pos = initial_pos[0];
  m_data_d[0].vel = 0;
  m_data_d[0].tau = 0;
  m_data_d[0].max_tau = MAX_CURRENT;
  m_data_d[0].pos_ff = 0;
  m_data_d[0].vel_ff = 0;
  m_data_d[0].tau_ff = 0;
  m_data_d2[0] = m_data_d[0];

  printf("Start sampling.\n");

  double run_time;
  struct timespec real_time, last_time;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (th_run)
  {
    m_data2[0] = m_data[0];
    em_get_data(ids, 1, m_data);

    set_pos(cos_wave(&wp, time), &m_data_d[0], &m_data_d2[0], dt);
    em_set_positions(ids, 1, m_data_d);

    time += dt;
    if (time >= wp.T)
    {
      time = 0;
    }

    sample_state[count].pos = m_data[0].pos;
    sample_state[count].vel = m_data[0].vel;
    sample_state[count].tau = m_data[0].tau;
    sample_cmd[count].pos = m_data_d[0].pos;
    sample_cmd[count].vel = m_data_d[0].vel;
    sample_cmd[count].tau = m_data_d[0].tau;
    sample_cmd[count].pos_ff = m_data_d[0].pos_ff;
    sample_cmd[count].vel_ff = m_data_d[0].vel_ff;
    sample_cmd[count].tau_ff = m_data_d[0].tau_ff;

    count++;
    if (count >= sample_size)
    {
      printf("Stop sampling, write file...\n");
      for (uint32_t i = 0; i < sample_size; i++)
      {
        fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                sample_state[i].pos, sample_state[i].vel, sample_state[i].tau,
                sample_cmd[i].pos, sample_cmd[i].vel, sample_cmd[i].tau,
                sample_cmd[i].pos_ff, sample_cmd[i].vel_ff, sample_cmd[i].tau_ff);
      }
      fclose(fp);
      printf("write file complete.\n");
      th_run = 0;
      break;
    }

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
  go_pos(ids, motor_cfg.num, initial_pos, 100, dt);
}

int8_t load_config(ECMConfig_t *ec_cfg, MotorConfig_t *m_cfg)
{
  strcpy(ec_cfg->ifname, "enp8s0");
  ec_cfg->dt = 5e-4;

  m_cfg->num = 0;
  for (uint16_t i = 0; i < NUM_MOTOR_MAX; i++)
  {
    m_cfg->enable[i] = 0;
    m_cfg->pos_offset[i] = 0;
    m_cfg->pos_factor[i] = BIT_17 / 360.0;
    m_cfg->vel_factor[i] = BIT_14 * 35.05 / 360.0;
  }

  for (uint16_t i = 0; i < NUM_MOTOR_MAX; i++)
  {
    ids[i] = i + 1;
  }
  motor_cfg_print(m_cfg, NUM_MOTOR_MAX);
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

  ret = em_enable_all(2000, 2);
  if (ret != 0)
  {
    return -2;
  }

  go_pos(ids, motor_cfg.num, initial_pos, 100, dt);

  th_run = 1;
  OSAL_THREAD_HANDLE thread;
  ret = osal_thread_create_rt(&thread, 8192000, &control_thread, (void *)(&dt));
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
