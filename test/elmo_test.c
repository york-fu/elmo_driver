#include "elmo_interface.h"

uint8_t joint_ids[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
double_t joint_data[12] = {0};
double_t joint_param[12] = {0};

OSAL_THREAD_HANDLE thread_control;

void pos_ctl_sin(double_t A, double_t w, double_t t, double_t b)
{
  joint_param[0] = A * sin(w * t) + b;
  set_joint_position(joint_ids, 1, joint_param);
  // printf("joint_param[0]:%f\n", joint_param[0]);
}

OSAL_THREAD_FUNC_RT control_thread(void *ptr)
{
  (void)ptr;
  double cycle_time = 0.001; // s
  uint32_t count = 0;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (1)
  {
    pos_ctl_sin(30, 2 * M_PI / 4.0, count * cycle_time, 20);
    count++;

    next_time.tv_sec += (next_time.tv_nsec + cycle_time * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + cycle_time * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void to_position(uint8_t id, double_t postion)
{
  double_t delta_pos = 0, vel = 50;
  uint32_t cycle_cnt = 0, total_cnt = 0;
  double cycle_time = 0.001; // s
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);

  get_joint_position(&joint_ids[id - 1], 1, &joint_data[id - 1]);
  delta_pos = postion - joint_data[id - 1];
  total_cnt = (uint32_t)((fabs(delta_pos) / vel) / cycle_time);
  printf("delta_pos:%f  total_cnt:%d  time:%fs\n", delta_pos, total_cnt, total_cnt * cycle_time);
  while (1)
  {
    cycle_cnt++;
    joint_param[id - 1] = joint_data[id - 1] + ((cycle_cnt / (double_t)total_cnt) * delta_pos);
    set_joint_position(&joint_ids[id - 1], 1, &joint_param[id - 1]);
    if (cycle_cnt >= total_cnt)
      break;

    next_time.tv_sec += (next_time.tv_nsec + cycle_time * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + cycle_time * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

int main(int argc, char *argv[])
{
  if (hard_init() != 0)
  {
    return 1;
  }
  to_position(1, 20);

  osal_thread_create_rt(&thread_control, 204800, &control_thread, NULL);

  while (1)
  {
    osal_usleep(1000);
  }

  return 0;
}
