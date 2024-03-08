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

void pd_contorl(const MotorData_t *m_data, const MotorData_t *m_data2,
                MotorData_t *m_data_d, MotorData_t *m_data_d2, double dt)
{
  double kp = 2, kd = 0.01;
  for (uint16_t i = 0; i < motor_cfg.num; i++)
  {
    set_pos(initial_pos[i], &m_data_d[i], &m_data_d2[i], dt);
    m_data_d[i].tau = kp * (m_data_d[i].pos - m_data[i].pos) + kd * (m_data_d[i].vel - m_data[i].vel);
    m_data_d[i].tau_ff = 0;
    m_data_d[i].max_tau = MAX_CURRENT;
  }
  em_set_torques(ids, motor_cfg.num, m_data_d);
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
    pd_contorl(m_data, m_data2, m_data_d, m_data_d2, dt);

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

  em_get_data(ids, motor_cfg.num, m_data);
  for (uint16_t i = 0; i < motor_cfg.num; i++)
  {
    initial_pos[i] = m_data[i].pos;
  }

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
