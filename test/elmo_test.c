#include "elmo_motor.h"
#include <signal.h>

OSAL_THREAD_HANDLE thread_sensor;
OSAL_THREAD_HANDLE thread_control;
OSAL_THREAD_HANDLE thread_sampling;
uint8_t test_running = 0;

uint8_t joint_ids[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
JointParam_t joint_data[12];
JointParam_t joint_data_old[12];
JointParam_t joint_cmd[12];
JointParam_t joint_cmd_old[12];
double_t jointOffset[] = {38, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double_t initial_position = 0;
double_t dt = 0.001; // s

double_t get_sin_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  out = A * -cos(2 * M_PI / T * time) + b + A;
  time += dt;
  if (time >= T) // prevention overflow
  {
    time = 0;
  }
  return out;
}

double_t get_square_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  if (time < (T / 2.0))
  {
    out = A + b;
  }
  else
  {
    out = -A + b;
  }
  time += dt;
  if (time >= T)
  {
    time = 0;
  }
  return out;
}

double_t get_triangular_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  if (time < (T / 2.0))
  {
    out = (A / (T / 2.0)) * time + b;
  }
  else
  {
    out = (A / (T / 2.0)) * (T - time) + b;
  }
  time += dt;
  if (time >= T)
  {
    time = 0;
  }
  return out;
}

double_t average_filter(double_t v, double_t *filter_array, uint16_t size)
{
  uint16_t i;
  double_t sum = 0;
  for (i = 0; i < size - 1; i++)
  {
    filter_array[i] = filter_array[i + 1];
  }
  filter_array[size - 1] = v;
  for (i = 0; i < size; i++)
  {
    sum += filter_array[i];
  }
  return (sum / size);
}

double_t velocityController(double_t targetVelocity, double_t torqueOffset)
{
  joint_cmd[0].torque = (targetVelocity - joint_data[0].velocity) * 0.04;
  joint_cmd[0].torque += torqueOffset;
  setJointTorque(joint_ids, 1, joint_cmd);
}

void positionController(double_t targetPosition, double_t velocityOffset, double_t torqueOffset)
{
  double_t goal_vel = (targetPosition - joint_data[0].position) * 40;
  velocityController(goal_vel + velocityOffset, torqueOffset);
}

void admittenceController(JointParam_t *state, JointParam_t *ref, double_t m, double_t b, double_t k, double_t dt)
{
  double_t spring = 0, damping = 0, inertia = 0;
  spring = (ref->position - state->position) * k;
  damping = (ref->velocity - state->velocity) * b;
  inertia = (ref->acceleration - state->acceleration) * m;
  ref->torque = inertia + damping + spring;
}

void joint_to_pos(uint8_t id, double_t goal_pos, double_t vel)
{
  double_t A = 0, T = 0, b = 0, w = 0;
  uint32_t count = 0, total_cnt = 0;
  struct timespec next_time;

  getJointData(&joint_ids[id - 1], 1, &joint_data[id - 1]);
  A = goal_pos - joint_data[id - 1].position;
  T = fabs(A) / vel;
  b = joint_data[id - 1].position;
  w = 2.0 * M_PI / T;
  total_cnt = (uint32_t)(T / dt);
  printf("start:%f stop:%f T:%f \n", b, goal_pos, T);
  if (fabs(A) < 1e-4)
    return;
  joint_cmd_old[id - 1].position = joint_data[id - 1].position;

  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    count++;
    joint_cmd[id - 1].position = A / 2.0 * -cos(w / 2.0 * (count * dt)) + A / 2.0 + b;
    joint_cmd[id - 1].velocity = (joint_cmd[id - 1].position - joint_cmd_old[0].position) / dt;
    setJointPosition(&joint_ids[id - 1], 1, &joint_cmd[id - 1]);
    // positionController(joint_cmd[0].position, joint_cmd[0].velocityOffset, joint_cmd[0].torqueOffset);
    joint_cmd_old[id - 1] = joint_cmd[id - 1];
    if (count >= total_cnt)
    {
      osal_usleep(2000);
      getJointData(&joint_ids[id - 1], 1, &joint_data[id - 1]);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void joint_to_vel(uint8_t id, double_t goal_vel, double_t acc)
{
  double_t A = 0, T = 0, b = 0, w = 0;
  uint32_t count = 0, total_cnt = 0;
  struct timespec next_time;

  getJointData(&joint_ids[id - 1], 1, &joint_data[id - 1]);
  A = goal_vel - joint_data[id - 1].velocity;
  T = fabs(A) / acc;
  b = joint_data[id - 1].velocity;
  w = 2.0 * M_PI / T;
  total_cnt = (uint32_t)(T / dt);
  printf("start:%f stop:%f T:%f \n", b, goal_vel, T);
  if (fabs(A) < 1e-4)
    return;
  joint_cmd_old[id - 1].velocity = joint_data[id - 1].velocity;

  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    count++;
    joint_cmd[id - 1].velocity = A / 2.0 * -cos(w / 2.0 * (count * dt)) + A / 2.0 + b;
    joint_cmd[id - 1].acceleration = (joint_cmd[id - 1].velocity - joint_cmd_old[0].velocity) / dt;
    setJointVelocity(&joint_ids[id - 1], 1, &joint_cmd[id - 1]);
    // printf("%f, %f\n", joint_cmd[id - 1].velocity, joint_cmd[id - 1].acceleration);
    joint_cmd_old[id - 1] = joint_cmd[id - 1];
    if (count >= total_cnt)
    {
      osal_usleep(2000);
      getJointData(&joint_ids[id - 1], 1, &joint_data[id - 1]);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

OSAL_THREAD_FUNC_RT sensor_update_thread(void *ptr)
{
  (void)ptr;
  struct timespec next_time;
  struct timespec real_time, last_time;
  double_t run_time;
  double_t velocity_filter[20] = {0};
  double_t acceleration_filter[50] = {0};

  getJointData(joint_ids, 1, joint_data_old);

  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  for (uint16_t i = 0; i < 20; i++)
  {
    getJointData(joint_ids, 1, joint_data);
    average_filter(joint_data[0].velocity, velocity_filter, 20);
    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  for (uint16_t i = 0; i < 50; i++)
  {
    getJointData(joint_ids, 1, joint_data);
    average_filter(joint_data[0].acceleration, acceleration_filter, 50);
    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }

  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    getJointData(joint_ids, 1, joint_data);
    // joint_data[0].velocity = average_filter(joint_data[0].velocity, velocity_filter, 20);
    joint_data[0].acceleration = (joint_data[0].velocity - joint_data_old[0].velocity) / dt;
    joint_data[0].acceleration = average_filter(joint_data[0].acceleration, acceleration_filter, 50);
    joint_data_old[0] = joint_data[0];

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

    clock_gettime(CLOCK_MONOTONIC, &real_time);
    run_time = (real_time.tv_sec + real_time.tv_nsec * 1e-9) - (last_time.tv_sec + last_time.tv_nsec * 1e-9);
    last_time = real_time;
    // printf("sensor_update_thread run time:%f\n", run_time);
  }
}

OSAL_THREAD_FUNC_RT control_thread(void *ptr)
{
  (void)ptr;
  uint32_t count = 0;
  struct timespec next_time;
  struct timespec real_time, last_time;
  double_t run_time;

  // joint_cmd[0].torque = 0.5;
  // setJointTorque(joint_ids, 1, joint_cmd);
  joint_to_pos(1, initial_position, 60);

  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    joint_cmd[0].position = get_sin_wave(30, 2, initial_position, dt);
    setJointPosition(joint_ids, 1, joint_cmd);
    // positionController(get_sin_wave(180, 2, initial_position, dt));

    count++;

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

    clock_gettime(CLOCK_MONOTONIC, &real_time);
    run_time = (real_time.tv_sec + real_time.tv_nsec * 1e-9) - (last_time.tv_sec + last_time.tv_nsec * 1e-9);
    last_time = real_time;
    // printf("control_thread run time:%f\n", run_time);
  }
}

OSAL_THREAD_FUNC_RT sampling_thread(void *ptr)
{
  (void)ptr;
  double_t run_time;
  uint32_t count = 0;
  uint32_t sampleSize = ceil(10.0 / dt);
  JointParam_t sampleState[sampleSize], sampleCmd[sampleSize];
  struct timespec next_time;
  struct timespec real_time, last_time;
  FILE *fp = NULL;
  fp = fopen("/data/p_cos_180_2.csv", "w");
  if (fp == NULL)
  {
    printf("fopen failed!\n");
    return;
  }
  fprintf(fp, "p, v, t, p_cmd, v_cmd, t_cmd, p_ff, v_ff, t_ff\n");
  getJointData(joint_ids, 1, joint_data);
  joint_data_old[0] = joint_data[0];
  joint_cmd_old[0].position = initial_position;
  joint_cmd_old[0].velocity = 0;
  joint_cmd_old[0].acceleration = 0;

  // joint_cmd[0].velocity = 720;
  // joint_to_vel(1, joint_cmd[0].velocity, 180);

  printf("sampling start.\n");
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    getJointData(joint_ids, 1, joint_data);
    joint_cmd[0].position = get_sin_wave(180, 2, initial_position, dt);
    joint_cmd[0].velocity = (joint_cmd[0].position - joint_cmd_old[0].position) / dt;
    joint_cmd[0].acceleration = (joint_cmd[0].velocity - joint_cmd_old[0].velocity) / dt;
    setJointPosition(joint_ids, 1, joint_cmd);

    sampleState[count].position = joint_data[0].position;
    sampleState[count].velocity = joint_data[0].velocity;
    sampleState[count].torque = joint_data[0].torque;
    sampleCmd[count].position = joint_cmd[0].position;
    sampleCmd[count].velocity = joint_cmd[0].velocity;
    sampleCmd[count].torque = joint_cmd[0].torque;
    sampleCmd[count].positionOffset = joint_cmd[0].positionOffset;
    sampleCmd[count].velocityOffset = joint_cmd[0].velocityOffset;
    sampleCmd[count].torqueOffset = joint_cmd[0].torqueOffset;
    sampleCmd[count].acceleration = joint_cmd[0].acceleration;

    joint_data_old[0] = joint_data[0];
    joint_cmd_old[0] = joint_cmd[0];

    count++;
    if (count >= sampleSize)
    {
      // joint_to_vel(1, 0, 180);
      printf("sampling stop, write file...\n");
      for (uint32_t i = 0; i < sampleSize; i++)
      {
        fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                sampleState[i].position, sampleState[i].velocity, sampleState[i].torque,
                sampleCmd[i].position, sampleCmd[i].velocity, sampleCmd[i].torque,
                sampleCmd[i].positionOffset, sampleCmd[i].velocityOffset, sampleCmd[i].torqueOffset);
      }
      fclose(fp);
      printf("write file complete.\n");

      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

    clock_gettime(CLOCK_MONOTONIC, &real_time);
    run_time = (real_time.tv_sec + real_time.tv_nsec * 1e-9) - (last_time.tv_sec + last_time.tv_nsec * 1e-9);
    last_time = real_time;
    if (run_time > (dt + 0.0001))
      printf("sampling_thread timeout, run_time:%f\n", run_time);
  }
}

void sigintHandler(int sig)
{
  joint_to_pos(1, initial_position, 60);
  test_running = 0;
  elmoDeInit();
  printf("signal exit.\n");
  exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
  test_running = 1;
  signal(SIGINT, sigintHandler);
  if (elmoInit() != 0)
  {
    return 1;
  }
  setJointOffset(jointOffset, 12);

  osal_thread_create_rt(&thread_sensor, 204800, (void *)&sensor_update_thread, NULL);
  osal_thread_create_rt(&thread_control, 204800, &control_thread, NULL);
  // osal_thread_create_rt(&thread_sampling, 8192000, &sampling_thread, NULL);

  while (1)
  {
    osal_usleep(1000);
  }

  return 0;
}
