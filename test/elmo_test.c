#include "elmo_interface.h"
#include <signal.h>

uint8_t joint_ids[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
JointParam_t joint_data[12] = {0};
JointParam_t joint_param[12] = {0};
double_t initial_position = 90;

OSAL_THREAD_HANDLE thread_control;
uint8_t test_running = 0;

void joint_move(uint8_t id, double_t goal_pos, double_t vel)
{
  double_t dt = 0.001; // s
  double_t A = 0, T = 0;
  uint32_t count = 0, total_cnt = 0;
  struct timespec next_time;

  get_joint_data(&joint_ids[id - 1], 1, &joint_data[id - 1]);
  A = goal_pos - joint_data[id - 1].position;
  T = fabs(A) / vel;
  total_cnt = (uint32_t)(T / dt);
  printf("start:%f stop:%f T:%f \n", joint_data[id - 1].position, goal_pos, T);
  if (fabs(A) < 1e-4)
    return;

  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    count++;
    joint_param[id - 1].position = A * sin((2 * M_PI / (4 * T)) * (count * dt)) + joint_data[id - 1].position;
    set_joint_position(&joint_ids[id - 1], 1, &joint_param[id - 1]);
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

double_t get_sin_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  out = (A * sin((2 * M_PI / T) * time) + b);
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

void set_joint_vel(double_t vel)
{
  joint_param[0].torque = (vel - joint_data[0].velocity) * 0.07;
  set_joint_torque(joint_ids, 1, joint_param);
}

void set_joint_pos(double_t pos)
{
  double_t goal_vel = (pos - joint_data[0].position) * 70;
  set_joint_vel(goal_vel);
}

OSAL_THREAD_FUNC_RT control_thread(void *ptr)
{
  (void)ptr;
  double dt = 0.001; // s
  double_t run_time;
  uint32_t count = 0;
  struct timespec next_time;
  struct timespec real_time, last_time;
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (test_running)
  {
    get_joint_data(joint_ids, 1, joint_data);

    // joint_param[0].position = get_square_wave(0.5, 1, initial_position, dt);
    // joint_param[0].position = get_triangular_wave(10, 1, initial_position, dt);
    joint_param[0].position = get_sin_wave(5, 1, initial_position, dt);
    set_joint_position(joint_ids, 1, joint_param);

    // joint_param[0].velocity = get_square_wave(10, 1, 0, dt);
    // set_joint_velocity(joint_ids, 1, joint_param);

    // joint_param[0].torque = get_square_wave(2, 0.01, 0, dt);
    // set_joint_torque(joint_ids, 1, joint_param);

    // set_joint_vel(get_square_wave(20, 1, 0, dt));
    // set_joint_pos(get_square_wave(0.5, 1, initial_position, dt));
    // set_joint_pos(get_sin_wave(5, 1, initial_position, dt));

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

void sigintHandler(int sig)
{
  test_running = 0;
  hard_exit();
  printf("signal exit.\n");
  exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
  test_running = 1;
  signal(SIGINT, sigintHandler);
  if (hard_init() != 0)
  {
    return 1;
  }
  joint_move(1, initial_position, 15);
  osal_usleep(500 * 1000);

  osal_thread_create_rt(&thread_control, 204800, &control_thread, NULL);

  while (1)
  {
    osal_usleep(1000);
  }

  return 0;
}
