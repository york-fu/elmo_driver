#include "elmo_motor.h"
#include <signal.h>
#include <unistd.h>
#include "util.h"

#define NUM_MOTOR_MAX 12
#define BIT_17 (1 << 17)
#define BIT_17_9 (BIT_17 * 9)
#define MAX_CURRENT (44.12)
#define MAX_TORQUE (MAX_CURRENT * 1.0)

uint32_t encoder_range[] = {BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9,
                            BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9};

const char *ifname = "enp2s0";
uint32_t num_motor = 1;
double dt = 1e-3; // s
double_t motor_offset[] = {0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0};
uint8_t motor_ids[12] = {1, 2, 3, 4, 5, 6,
                         7, 8, 9, 10, 11, 12};
MotorParam_t raw_motor_data[NUM_MOTOR_MAX];
MotorParam_t raw_motor_data_old[NUM_MOTOR_MAX];
MotorParam_t motor_data[NUM_MOTOR_MAX];
MotorParam_t motor_data_old[NUM_MOTOR_MAX];
MotorParam_t motor_cmd[NUM_MOTOR_MAX];
MotorParam_t motor_cmd_old[NUM_MOTOR_MAX];
double initial_position[NUM_MOTOR_MAX] = {0};

OSAL_THREAD_HANDLE thread_control;
OSAL_THREAD_HANDLE thread_sampling;
static uint8_t _running = 0;

static double calcCos(double start, double stop, double T, double t)
{
  double A = (stop - start) / 2.0;
  return A * -cos(M_PI / T * t) + start + A;
}

void motorMoveTo(uint8_t *ids, uint8_t num_id, double *end_pos, double speed, double dt)
{
  EM_getData(ids, num_id, motor_data);
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
      motor_cmd[i].position = calcCos(start_pos[i], end_pos[i], max_T, t);
      motor_cmd[i].maxTorque = MAX_TORQUE;
    }
    EM_setPositions(ids, num_id, motor_cmd);

    t += dt;
    if (t > max_T)
    {
      osal_usleep(2000);
      EM_getData(ids, num_id, motor_data);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void motorVelTo(uint8_t *ids, uint8_t num_id, double *end_vel, double acc, double dt)
{
  EM_getData(ids, num_id, motor_data);
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
      motor_cmd[i].velocity = calcCos(start_vel[i], end_vel[i], max_T, t);
      motor_cmd[i].maxTorque = MAX_TORQUE;
    }
    EM_setVelocities(ids, num_id, motor_cmd);

    t += dt;
    if (t > max_T)
    {
      osal_usleep(2000);
      EM_getData(ids, num_id, motor_data);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

double velControl(MotorParam_t *desire, MotorParam_t *state)
{
  static double intergral = 0;
  double err = desire->velocity - state->velocity;
  intergral += err;
  intergral = LIMITING(intergral, -5e3, 5e3);
  return 0.03 * err + 1e-3 * intergral + desire->torqueOffset;
}

double posControl(MotorParam_t *desire, MotorParam_t *state)
{
  double err = desire->position - state->position;
  desire->velocity = 40 * err + desire->velocityOffset;
  return velControl(desire, state);
}

void admittanceController(MotorParam_t *state, MotorParam_t *ref, double_t m, double_t b, double_t k, double_t dt)
{
  double_t spring = 0, damping = 0, inertia = 0;
  spring = (ref->position - state->position) * k;
  damping = (ref->velocity - state->velocity) * b;
  inertia = (ref->acceleration - state->acceleration) * m;
  ref->torque = inertia + damping + spring;
}

void setPosCmd(double pos, MotorParam_t *data, MotorParam_t *data_old, double dt)
{
  data->position = pos;
  data->velocity = (data->position - data_old->position) / dt;
  data->acceleration = (data->velocity - data_old->velocity) / dt;
}

void setVelCmd(double vel, MotorParam_t *data, MotorParam_t *data_old, double dt)
{
  data->velocity = vel;
  data->acceleration = (data->velocity - data_old->velocity) / dt;
}

void cst_test()
{
  motor_cmd_old[0] = motor_cmd[0];
  motor_cmd[0].torque = get_square_wave(2, 0.01, 0, dt);
  motor_cmd[0].maxTorque = MAX_TORQUE;
  EM_setTorques(motor_ids, 1, motor_cmd);
}

void csv_test()
{
  motor_cmd_old[0] = motor_cmd[0];
  double vel = get_sin_wave(180, 1, 0, dt);
  setVelCmd(vel, &motor_cmd[0], &motor_cmd_old[0], dt);
  motor_cmd[0].maxTorque = MAX_TORQUE;
  EM_setVelocities(motor_ids, 1, motor_cmd);
}

void csp_test()
{
  motor_cmd_old[0] = motor_cmd[0];
  double pos = get_cos_wave(30, 4, initial_position[0], dt);
  setPosCmd(pos, &motor_cmd[0], &motor_cmd_old[0], dt);
  motor_cmd[0].maxTorque = MAX_TORQUE;
  EM_setPositions(motor_ids, 1, motor_cmd);
}

OSAL_THREAD_FUNC_RT control_thread(void *ptr)
{
  (void)ptr;
  uint32_t vel_filter_size = 10;
  double vel_filter[num_motor][vel_filter_size];
  uint32_t accel_filter_size = 10;
  double accel_filter[num_motor][accel_filter_size];

  motorMoveTo(motor_ids, num_motor, initial_position, 100, dt);

  struct timespec real_time, last_time;
  double run_time;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (_running)
  {
    for (uint32_t i = 0; i < num_motor; i++)
    {
      raw_motor_data_old[i] = raw_motor_data[i];
      motor_data_old[i] = motor_data[i];
    }
    EM_getData(motor_ids, num_motor, raw_motor_data);
    for (uint32_t i = 0; i < num_motor; i++)
    {
      raw_motor_data[i].acceleration = (raw_motor_data[i].velocity - raw_motor_data_old[i].velocity) / dt;
      motor_data[i] = raw_motor_data[i];
    }

    for (uint32_t i = 0; i < num_motor; i++)
    {
      average_filter(vel_filter[i], vel_filter_size, &motor_data[i].velocity);
      motor_data[i].acceleration = (motor_data[i].velocity - motor_data_old[i].velocity) / dt;
      average_filter(accel_filter[i], accel_filter_size, &motor_data[i].acceleration);
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

  motorMoveTo(motor_ids, num_motor, initial_position, 100, dt);

  FILE *fp = NULL;
  fp = fopen("./data/p_cos_360_4.csv", "w+");
  if (fp == NULL)
  {
    printf("fopen failed!\n");
    _running = 0;
    return;
  }
  fprintf(fp, "p, v, t, p_cmd, v_cmd, t_cmd, p_ff, v_ff, t_ff\n");

  EM_getData(motor_ids, 1, motor_data);
  motor_data_old[0] = motor_data[0];
  motor_cmd[0].position = initial_position[0];
  motor_cmd[0].velocity = 0;
  motor_cmd[0].torque = 0;
  motor_cmd[0].maxTorque = MAX_TORQUE;
  motor_cmd[0].positionOffset = 0;
  motor_cmd[0].velocityOffset = 0;
  motor_cmd[0].torqueOffset = 0;
  motor_cmd_old[0] = motor_cmd[0];

  printf("sampling start.\n");
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (_running)
  {
    motor_data_old[0] = motor_data[0];
    EM_getData(motor_ids, 1, motor_data);

    motor_cmd_old[0] = motor_cmd[0];
    double pos = get_cos_wave(360, 4, initial_position[0], dt);
    setPosCmd(pos, &motor_cmd[0], &motor_cmd_old[0], dt);
    EM_setPositions(motor_ids, 1, motor_cmd);

    sample_state[count].position = motor_data[0].position;
    sample_state[count].velocity = motor_data[0].velocity;
    sample_state[count].torque = motor_data[0].torque;
    sample_cmd[count].position = motor_cmd[0].position;
    sample_cmd[count].velocity = motor_cmd[0].velocity;
    sample_cmd[count].torque = motor_cmd[0].torque;
    sample_cmd[count].positionOffset = motor_cmd[0].positionOffset;
    sample_cmd[count].velocityOffset = motor_cmd[0].velocityOffset;
    sample_cmd[count].torqueOffset = motor_cmd[0].torqueOffset;

    count++;
    if (count >= sample_size)
    {
      printf("sampling stop, write file...\n");
      for (uint32_t i = 0; i < sample_size; i++)
      {
        fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                sample_state[i].position, sample_state[i].velocity, sample_state[i].torque,
                sample_cmd[i].position, sample_cmd[i].velocity, sample_cmd[i].torque,
                sample_cmd[i].positionOffset, sample_cmd[i].velocityOffset, sample_cmd[i].torqueOffset);
      }
      fclose(fp);
      printf("write file complete.\n");
      _running = 0;
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

int32_t process_rt()
{
  int32_t ret;
  pid_t pid = getpid();
  struct sched_param param;
  param.sched_priority = sched_get_priority_max(SCHED_FIFO); // SCHED_RR
  ret = sched_setscheduler(pid, SCHED_FIFO, &param);
  if (ret != 0)
  {
    printf("Failed to set rt of process %d. %s\n", pid, strerror(ret));
  }
  return ret;
}

void sigintHandler(int sig)
{
  _running = 0;
  osal_usleep(2000);
  printf("\nMotor resetting...\n");
  motorMoveTo(motor_ids, num_motor, initial_position, 100, dt);
  printf("Motor reset.\n");
  EM_deInit();
  printf("signal exit.\n");
  exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
  process_rt();
  _running = 1;
  signal(SIGINT, sigintHandler);

  MotorOptions_t motor_opt;
  motor_opt.size = NUM_MOTOR_MAX;
  motor_opt.encoder_range = encoder_range;
  motor_opt.position_limit = 0;
  if (EM_init(ifname, 1e-3, motor_opt) != 0)
  {
    return 1;
  }
  EM_setPositionsOffset(motor_offset, num_motor);

  osal_thread_create_rt(&thread_control, 204800, &control_thread, NULL);
  // osal_thread_create_rt(&thread_sampling, 2048000, &sampling_thread, NULL);

  while (_running)
  {
    osal_usleep(1000);
  }

  return 0;
}
