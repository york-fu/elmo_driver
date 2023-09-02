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

#define NUM_MOTOR_MAX 12
#define BIT_17 (1 << 17)
#define BIT_19 (1 << 19)
#define BIT_17_9 (BIT_17 * 9)
#define MAX_CURRENT (44.12)

const char *ifname = "enp8s0";
double motor_range[NUM_MOTOR_MAX] = {
    BIT_17, BIT_17, BIT_17, BIT_17, BIT_17, BIT_17,
    BIT_17, BIT_17, BIT_17, BIT_17, BIT_17, BIT_17};
double motor_offset[NUM_MOTOR_MAX] = {0};

double dt = 1e-3;
uint16_t num_motor = 1;
int8_t running = 0;
ECMConfig_t ec_cfg;
uint8_t motor_ids[12] = {
    1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12};
MotorParam_t motor_data[NUM_MOTOR_MAX];
MotorParam_t motor_cmd[NUM_MOTOR_MAX];
MotorParam_t motor_data2[NUM_MOTOR_MAX];
MotorParam_t motor_cmd2[NUM_MOTOR_MAX];

MotorParam_t mdata_filter[NUM_MOTOR_MAX];
MotorParam_t mdata2_flter[NUM_MOTOR_MAX];

double initial_position[NUM_MOTOR_MAX] = {0};

PIDParam_t v_ctl[NUM_MOTOR_MAX];
PIDParam_t p_ctl[NUM_MOTOR_MAX];

int8_t load_em_cfg()
{
  strcpy(ec_cfg.ifname, ifname);
  ec_cfg.dt = dt;

  MotorConfig_t cfg;
  cfg.circle_unit = 360;
  cfg.sw_check = 0;
  cfg.gear = 9;
  for (uint16_t i = 0; i < num_motor; i++)
  {
    cfg.range[i] = motor_range[i];
    cfg.offset[i] = motor_offset[i];
  }

  em_set_motor_cfg(cfg, num_motor);
}

static double clac_cos(double start, double stop, double T, double t)
{
  double A = (stop - start) / 2.0;
  return A * -cos(M_PI / T * t) + start + A;
}

void go_pos(uint8_t *ids, uint8_t num_id, double *end_pos, double speed, double dt)
{
  em_get_data(ids, num_id, motor_data);
  double start_pos[num_id];
  double T[num_id];
  for (uint32 i = 0; i < num_id; i++)
  {
    start_pos[i] = motor_data[i].position;
    T[i] = fabs(end_pos[i] - start_pos[i]) / speed;
    printf("Motor %d from %f to %f\n", i + 1, start_pos[i], end_pos[i]);
  }
  double max_T = T[0];
  for (uint32 i = 1; i < num_id; i++)
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
    for (uint32 i = 0; i < num_id; i++)
    {
      motor_cmd[i].position = clac_cos(start_pos[i], end_pos[i], max_T, t);
      motor_cmd[i].max_torque = MAX_CURRENT;
    }
    em_set_positions(ids, num_id, motor_cmd);

    t += dt;
    if (t > max_T)
    {
      osal_usleep(2000);
      em_get_data(ids, num_id, motor_data);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void go_vel(uint8_t *ids, uint8_t num_id, double *end_vel, double acc, double dt)
{
  em_get_data(ids, num_id, motor_data);
  double start_vel[num_id];
  double T[num_id];
  for (uint32 i = 0; i < num_id; i++)
  {
    start_vel[i] = motor_data[i].velocity;
    T[i] = fabs(end_vel[i] - start_vel[i]) / acc;
    printf("Motor %d from %f to %f\n", i + 1, start_vel[i], end_vel[i]);
  }
  double max_T = T[0];
  for (uint32 i = 1; i < num_id; i++)
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
    for (uint32 i = 0; i < num_id; i++)
    {
      motor_cmd[i].velocity = clac_cos(start_vel[i], end_vel[i], max_T, t);
      motor_cmd[i].max_torque = MAX_CURRENT;
    }
    em_set_velocities(ids, num_id, motor_cmd);

    t += dt;
    if (t > max_T)
    {
      osal_usleep(2000);
      em_get_data(ids, num_id, motor_data);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void set_pos_cmd(double pos, MotorParam_t *cmd, MotorParam_t *cmd_old, double dt)
{
  cmd->position = pos;
  cmd->velocity = (cmd->position - cmd_old->position) / dt;
  cmd->acceleration = (cmd->velocity - cmd_old->velocity) / dt;
}

void set_vel_cmd(double vel, MotorParam_t *cmd, MotorParam_t *cmd_old, double dt)
{
  cmd->velocity = vel;
  cmd->acceleration = (cmd->velocity - cmd_old->velocity) / dt;
}

void load_control_param()
{
  for (uint16_t i = 0; i < num_motor; i++)
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

double v_control(uint16_t i, MotorParam_t *desire, MotorParam_t *state)
{
  return PI_controller(&v_ctl[i], desire->velocity - state->velocity);
}

double p_control(uint16_t i, MotorParam_t *desire, MotorParam_t *state)
{
  double v = P_controller(&p_ctl[i], desire->position - state->position);
  desire->velocity += v;
  return v_control(i, desire, state);
}

double admittance_control(MotorParam_t *desire, MotorParam_t *state, double_t m, double_t b, double_t k, double_t dt)
{
  double spring = (desire->position - state->position) * k;
  double damping = (desire->velocity - state->velocity) * b;
  double inertia = (desire->acceleration - state->acceleration) * m;
  return (inertia + damping + spring);
}

void csv2_test()
{
  static double time = 0;
  WaveParam_t wp;
  wp.A = 180;
  wp.T = 4;
  wp.b = 0;

  for (uint16_t i = 0; i < num_motor; i++)
  {
    motor_cmd2[i] = motor_cmd[i];
    double vel = sin_wave(&wp, time);
    set_vel_cmd(vel, &motor_cmd[i], &motor_cmd2[i], dt);
    motor_cmd[i].torque = v_control(i, &motor_cmd[i], &motor_data[i]);
    motor_cmd[i].torque_offset = 0;
    motor_cmd[i].max_torque = MAX_CURRENT;
  }
  em_set_torques(motor_ids, num_motor, motor_cmd);

  time += dt;
  if (time >= wp.T)
  {
    time = 0;
  }
}

void csp2_test()
{
  static double time = 0;
  WaveParam_t wp;
  wp.A = 90;
  wp.T = 4;

  for (uint16_t i = 0; i < num_motor; i++)
  {
    motor_cmd2[i] = motor_cmd[i];
    wp.b = initial_position[i];
    double pos = cos_wave(&wp, time);
    set_pos_cmd(pos, &motor_cmd[i], &motor_cmd2[i], dt);
    motor_cmd[i].torque = p_control(i, &motor_cmd[i], &motor_data[i]);
    motor_cmd[i].torque_offset = 0;
    motor_cmd[i].max_torque = MAX_CURRENT;
  }
  em_set_torques(motor_ids, num_motor, motor_cmd);

  time += dt;
  if (time >= wp.T)
  {
    time = 0;
  }
}

void pd_ctl_test()
{
  static double time = 0;
  WaveParam_t wp;
  wp.A = 30;
  wp.T = 4;

  double kp = 20, kd = 0.01;
  for (uint16_t i = 0; i < num_motor; i++)
  {
    motor_cmd2[i] = motor_cmd[i];
    wp.b = initial_position[i];
    double pos = cos_wave(&wp, time);
    set_pos_cmd(pos, &motor_cmd[i], &motor_cmd2[i], dt);
    motor_cmd[i].torque = kp * (motor_cmd[i].position - motor_cmd2[i].position) + kd * (motor_cmd[i].velocity - motor_cmd2[i].velocity);
    motor_cmd[i].torque_offset = 0;
    motor_cmd[i].max_torque = MAX_CURRENT;
  }
  em_set_torques(motor_ids, num_motor, motor_cmd);

  time += dt;
  if (time >= wp.T)
  {
    time = 0;
  }
}

void cst_test()
{
  static double time = 0;
  WaveParam_t wp;
  wp.A = 2;
  wp.T = 0.01;
  wp.b = 0;

  for (uint16_t i = 0; i < num_motor; i++)
  {
    motor_cmd2[i] = motor_cmd[i];
    motor_cmd[i].torque = square_wave(&wp, time);
    motor_cmd[i].torque_offset = 0;
    motor_cmd[i].max_torque = MAX_CURRENT;
  }
  em_set_torques(motor_ids, num_motor, motor_cmd);

  time += dt;
  if (time >= wp.T)
  {
    time = 0;
  }
}

void csv_test()
{
  static double time = 0;
  WaveParam_t wp;
  wp.A = 180;
  wp.T = 1;
  wp.b = 0;

  for (uint16_t i = 0; i < num_motor; i++)
  {
    motor_cmd2[i] = motor_cmd[i];
    double vel = sin_wave(&wp, time);
    set_vel_cmd(vel, &motor_cmd[i], &motor_cmd2[i], dt);
    motor_cmd[i].torque_offset = 0;
    motor_cmd[i].velocity_offset = 0;
    motor_cmd[i].max_torque = MAX_CURRENT;
  }
  em_set_velocities(motor_ids, num_motor, motor_cmd);

  time += dt;
  if (time >= wp.T)
  {
    time = 0;
  }
}

void csp_test()
{
  static double time = 0;
  WaveParam_t wp;
  wp.A = 90;
  wp.T = 4;

  for (uint16_t i = 0; i < num_motor; i++)
  {
    motor_cmd2[i] = motor_cmd[i];
    wp.b = initial_position[i];
    double pos = cos_wave(&wp, time);
    set_pos_cmd(pos, &motor_cmd[i], &motor_cmd2[i], dt);
    motor_cmd[i].torque_offset = 0;
    motor_cmd[i].velocity_offset = 0;
    motor_cmd[i].position_offset = 0;
    motor_cmd[0].max_torque = MAX_CURRENT;
  }
  em_set_positions(motor_ids, num_motor, motor_cmd);

  time += dt;
  if (time >= wp.T)
  {
    time = 0;
  }
}

OSAL_THREAD_FUNC_RT control_thread(void *ptr)
{
  (void)ptr;
  double alpha = 0.7;

  go_pos(motor_ids, num_motor, initial_position, 100, dt);
  load_control_param();

  struct timespec real_time, last_time;
  double run_time;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (running)
  {
    for (uint32_t i = 0; i < num_motor; i++)
    {
      motor_data2[i] = motor_data[i];
    }
    em_get_data(motor_ids, num_motor, motor_data);
    for (uint32_t i = 0; i < num_motor; i++)
    {
      motor_data[i].acceleration = (motor_data[i].velocity - motor_data2[i].velocity) / dt;
    }

    for (uint32_t i = 0; i < num_motor; i++)
    {
      mdata2_flter[i] = mdata_filter[i];
      mdata_filter[i] = motor_data[i];
      mdata_filter[i].velocity = alpha * mdata_filter[i].velocity + (1 - alpha) * mdata2_flter[i].velocity;
      mdata_filter[i].acceleration = (mdata_filter[i].velocity - mdata2_flter[i].velocity) / dt;
    }

    csp_test();

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

    clock_gettime(CLOCK_MONOTONIC, &real_time);
    run_time = (real_time.tv_sec + real_time.tv_nsec * 1e-9) - (last_time.tv_sec + last_time.tv_nsec * 1e-9);
    last_time = real_time;
    // printf("read_thread run time:%f\n", run_time);
  }
}

OSAL_THREAD_FUNC_RT sampling_thread(void *ptr)
{
  (void)ptr;
  double_t run_time;
  uint32_t count = 0;
  uint32_t sample_size = ceil(12.0 / dt);
  MotorParam_t sample_state[sample_size], sample_cmd[sample_size];
  struct timespec next_time;
  struct timespec real_time, last_time;

  go_pos(motor_ids, num_motor, initial_position, 100, dt);

  FILE *fp = NULL;
  fp = fopen("./data/p_cos_360_4.csv", "w+");
  if (fp == NULL)
  {
    printf("failed to fopen\n");
    running = 0;
    return;
  }
  fprintf(fp, "p, v, t, p_cmd, v_cmd, t_cmd, p_ff, v_ff, t_ff\n");

  em_get_data(motor_ids, 1, motor_data);
  motor_data2[0] = motor_data[0];
  motor_cmd[0].position = initial_position[0];
  motor_cmd[0].velocity = 0;
  motor_cmd[0].torque = 0;
  motor_cmd[0].max_torque = MAX_CURRENT;
  motor_cmd[0].position_offset = 0;
  motor_cmd[0].velocity_offset = 0;
  motor_cmd[0].torque_offset = 0;
  motor_cmd2[0] = motor_cmd[0];

  double time = 0;
  WaveParam_t wp;
  wp.A = 360;
  wp.T = 4;
  wp.b = initial_position[0];

  printf("sampling start.\n");
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (running)
  {
    motor_data2[0] = motor_data[0];
    em_get_data(motor_ids, 1, motor_data);

    motor_cmd2[0] = motor_cmd[0];
    double pos = cos_wave(&wp, time);
    set_pos_cmd(pos, &motor_cmd[0], &motor_cmd2[0], dt);
    em_set_positions(motor_ids, 1, motor_cmd);

    time += dt;
    if (time >= wp.T)
    {
      wp.T = 0;
    }

    sample_state[count].position = motor_data[0].position;
    sample_state[count].velocity = motor_data[0].velocity;
    sample_state[count].torque = motor_data[0].torque;
    sample_cmd[count].position = motor_cmd[0].position;
    sample_cmd[count].velocity = motor_cmd[0].velocity;
    sample_cmd[count].torque = motor_cmd[0].torque;
    sample_cmd[count].position_offset = motor_cmd[0].position_offset;
    sample_cmd[count].velocity_offset = motor_cmd[0].velocity_offset;
    sample_cmd[count].torque_offset = motor_cmd[0].torque_offset;

    count++;
    if (count >= sample_size)
    {
      printf("sampling stop, write file...\n");
      for (uint32_t i = 0; i < sample_size; i++)
      {
        fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                sample_state[i].position, sample_state[i].velocity, sample_state[i].torque,
                sample_cmd[i].position, sample_cmd[i].velocity, sample_cmd[i].torque,
                sample_cmd[i].position_offset, sample_cmd[i].velocity_offset, sample_cmd[i].torque_offset);
      }
      fclose(fp);
      printf("write file complete.\n");
      running = 0;
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

    clock_gettime(CLOCK_MONOTONIC, &real_time);
    run_time = (real_time.tv_sec + real_time.tv_nsec * 1e-9) - (last_time.tv_sec + last_time.tv_nsec * 1e-9);
    last_time = real_time;
    if (run_time > (dt + 0.0002))
      printf("sampling_thread timeout, run_time:%f\n", run_time);
  }
}

void sigint_handler(int sig)
{
  running = 0;
  osal_usleep(4000);
  printf("\nMotor resetting...\n");
  go_pos(motor_ids, num_motor, initial_position, 100, dt);
  printf("Motor reset.\n");
  em_deinit();
  printf("signal exit.\n");
  exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
  signal(SIGINT, sigint_handler);

  int ret = 0;
  load_em_cfg();
  ret = em_init(ec_cfg);
  if (ret != 0)
  {
    return -1;
  }
  ret = em_enable_all(2000);
  if (ret != 0)
  {
    return -2;
  }

  running = 1;
  OSAL_THREAD_HANDLE thread_control;
  ret = osal_thread_create_rt(&thread_control, 204800, &control_thread, NULL);
  if (ret != 1)
  {
    return -2;
  }

  while (running)
  {
    osal_usleep(1000);
  }

  return 0;
}