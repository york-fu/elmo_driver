#include "elmo_interface.h"
#include <signal.h>

typedef struct
{
  double_t position;
  double_t velocity;
  double_t current;
  double_t acceleration;
} MotorState_t;

OSAL_THREAD_HANDLE thread_control;
OSAL_THREAD_HANDLE thread_sampling;
uint8_t test_running = 0;

uint8_t joint_ids[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
JointParam_t joint_data[12];
JointParam_t joint_data_old[12];
JointParam_t joint_param[12];
JointParam_t joint_param_old[12];
double_t initial_position = 0;

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

void velocity_controller(double_t vel)
{
  joint_param[0].torque = (vel - joint_data[0].velocity) * 0.03;
  set_joint_torque(joint_ids, 1, joint_param);
}

void position_controller(double_t pos)
{
  double_t goal_vel = (pos - joint_data[0].position) * 50;
  velocity_controller(goal_vel);
}

void admittenceController(MotorState_t *state, MotorState_t *ref, double_t m, double_t b, double_t k, double_t dt)
{
  static MotorState_t stateLast;
  double_t spring = 0, damping = 0, inertia = 0;
  state->acceleration = (state->velocity - stateLast.velocity) / dt;
  stateLast = *state;
  spring = (ref->position - state->position) * k;
  damping = (ref->velocity - state->velocity) * b;
  inertia = (ref->acceleration - state->acceleration) * m;
  ref->current = inertia + damping + spring;
}

void joint_to_pos(uint8_t id, double_t goal_pos, double_t vel)
{
  double_t dt = 0.001; // s
  double_t A = 0, T = 0, w = 0;
  uint32_t count = 0, total_cnt = 0;
  struct timespec next_time;

  get_joint_data(&joint_ids[id - 1], 1, &joint_data[id - 1]);
  A = goal_pos - joint_data[id - 1].position;
  T = fabs(A) / vel;
  w = 2.0 * M_PI / T;
  total_cnt = (uint32_t)(T / dt);
  printf("start:%f stop:%f T:%f \n", joint_data[id - 1].position, goal_pos, T);
  if (fabs(A) < 1e-4)
    return;
  joint_param_old[id - 1].position = joint_data[id - 1].position;

  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    count++;
    joint_param[id - 1].position = A / 2.0 * -cos(w / 2.0 * (count * dt)) + A / 2.0 + joint_data[id - 1].position;
    joint_param[id - 1].velocity = (joint_param[id - 1].position - joint_param_old[0].position) / dt;
    set_joint_position(&joint_ids[id - 1], 1, &joint_param[id - 1]);
    joint_param_old[id - 1] = joint_param[id - 1];
    if (count >= total_cnt)
    {
      osal_usleep(2000);
      get_joint_data(&joint_ids[id - 1], 1, &joint_data[id - 1]);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void joint_to_vel(uint8_t id, double_t goal_vel, double_t acc)
{
  double_t dt = 0.001; // s
  double_t A = 0, T = 0, w = 0;
  uint32_t count = 0, total_cnt = 0;
  struct timespec next_time;

  get_joint_data(&joint_ids[id - 1], 1, &joint_data[id - 1]);
  A = goal_vel - joint_data[id - 1].velocity;
  T = fabs(A) / acc;
  w = 2.0 * M_PI / T;
  total_cnt = (uint32_t)(T / dt);
  printf("start:%f stop:%f T:%f \n", joint_data[id - 1].velocity, goal_vel, T);
  if (fabs(A) < 1e-4)
    return;
  joint_param_old[id - 1].velocity = joint_data[id - 1].velocity;

  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    count++;
    joint_param[id - 1].velocity = A / 2.0 * -cos(w / 2.0 * (count * dt)) + A / 2.0 + joint_data[id - 1].velocity;
    joint_param[id - 1].acceleration = (joint_param[id - 1].velocity - joint_param_old[0].velocity) / dt;
    set_joint_velocity(&joint_ids[id - 1], 1, &joint_param[id - 1]);
    // printf("%f, %f\n", joint_param[id - 1].velocity, joint_param[id - 1].acceleration);
    joint_param_old[id - 1] = joint_param[id - 1];
    if (count >= total_cnt)
    {
      osal_usleep(2000);
      get_joint_data(&joint_ids[id - 1], 1, &joint_data[id - 1]);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

OSAL_THREAD_FUNC_RT control_thread(void *ptr)
{
  (void)ptr;
  double dt = 0.001; // s
  double_t run_time;
  uint32_t count = 0;
  struct timespec next_time;
  struct timespec real_time, last_time;

  // joint_param[0].torque = 0.5;
  // set_joint_torque(joint_ids, 1, joint_param);
  
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    get_joint_data(joint_ids, 1, joint_data);

    joint_param[0].position = get_sin_wave(30, 1, initial_position, dt);
    set_joint_position(joint_ids, 1, joint_param);
    // position_controller(get_sin_wave(180, 2, initial_position, dt));
    // printf("%f, %f, %f\n", joint_param[0].torque, joint_data[0].velocity, joint_data[0].torque);

    count++;

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

    // clock_gettime(CLOCK_MONOTONIC, &real_time);
    // run_time = (real_time.tv_sec + real_time.tv_nsec * 1e-9) - (last_time.tv_sec + last_time.tv_nsec * 1e-9);
    // last_time = real_time;
    // printf("run_time:%f  curr_pos:%f  cmd_pos:%f\n", run_time, joint_data[0].position, joint_param[0].position);
  }
}

OSAL_THREAD_FUNC_RT sampling_thread(void *ptr)
{
  (void)ptr;
  double dt = 0.001; // s
  double_t run_time;
  uint32_t count = 0;
  uint32_t size = ceil(10.0 / dt);
  MotorState_t motorState[size];
  MotorState_t motorCmd[size];
  struct timespec next_time;
  struct timespec real_time, last_time;
  FILE *fp = NULL;
  fp = fopen("/data/motor_cos_180_2.csv", "w");
  get_joint_data(joint_ids, 1, joint_data);
  joint_data_old[0] = joint_data[0];
  joint_param_old[0].position = initial_position;
  joint_param_old[0].velocity = 0;
  joint_param_old[0].acceleration = 0;

  // joint_param[0].velocity = 720;
  // joint_to_vel(1, joint_param[0].velocity, 180);

  printf("sampling start.\n");
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    get_joint_data(joint_ids, 1, joint_data);
    joint_param[0].position = get_sin_wave(180, 2, initial_position, dt);
    joint_param[0].velocity = (joint_param[0].position - joint_param_old[0].position) / dt;
    joint_param[0].acceleration = (joint_param[0].velocity - joint_param_old[0].velocity) / dt;
    set_joint_position(joint_ids, 1, joint_param);

    motorState[count].position = joint_data[0].position;
    motorState[count].velocity = joint_data[0].velocity;
    motorState[count].acceleration = (joint_data[0].velocity - joint_data_old[0].velocity) / dt;
    motorState[count].current = joint_data[0].torque;
    motorCmd[count].position = joint_param[0].position;
    motorCmd[count].velocity = joint_param[0].velocity;
    motorCmd[count].acceleration = joint_param[0].acceleration;

    joint_data_old[0] = joint_data[0];
    joint_param_old[0] = joint_param[0];

    count++;
    if (count >= size)
    {
      // joint_to_vel(1, 0, 180);
      printf("sampling stop, write file...\n");
      for (uint32_t i = 0; i < size; i++)
      {
        fprintf(fp, "%f, %f, %f, %f, %f, %f, %f\n",
                motorState[i].position, motorState[i].velocity, motorState[i].acceleration, motorState[i].current,
                motorCmd[i].position, motorCmd[i].velocity, motorCmd[i].acceleration);
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
  test_running = 0;
  device_close();
  printf("signal exit.\n");
  exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
  test_running = 1;
  signal(SIGINT, sigintHandler);
  if (device_init() != 0)
  {
    return 1;
  }
  joint_to_pos(1, initial_position, 180);

  osal_thread_create_rt(&thread_control, 204800, &control_thread, NULL);
  // osal_thread_create_rt(&thread_sampling, 8192000, &sampling_thread, NULL);

  while (1)
  {
    osal_usleep(1000);
  }

  return 0;
}
