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
#define BIT_17 (1 << 14)
#define BIT_14 (1 << 17)

MotorConfig_t motor_cfg;

int8_t load_config(ECMConfig_t *ec_cfg, MotorConfig_t *m_cfg)
{
  strcpy(ec_cfg->ifname, "enp8s0");
  ec_cfg->dt = 5e-4;

  m_cfg->num = 0;
  m_cfg->circle_unit = 360;
  for (uint16_t i = 0; i < NUM_MOTOR_MAX; i++)
  {
    m_cfg->gear[i] = 1;
    m_cfg->pos_enc_range[i] = BIT_17;
    m_cfg->vel_enc_range[i] = BIT_14;
    m_cfg->pos_offset[i] = 0;
    m_cfg->enable[i] = 0;
  }

  motor_cfg_print(m_cfg, NUM_MOTOR_MAX);
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

  uint8_t mode = 0;
  if (argc > 1)
  {
    mode = atoi(argv[1]);
  }
  printf("Enable mode: %d\n", mode);
  ret = em_enable_all(2000, mode);
  if (ret != 0)
  {
    return -2;
  }

  uint8_t ids[NUM_MOTOR_MAX];
  MotorData_t m_data[NUM_MOTOR_MAX];
  for (uint16_t i = 0; i < NUM_MOTOR_MAX; i++)
  {
    ids[i] = i + 1;
  }

  while (1)
  {
    em_get_data(ids, NUM_MOTOR_MAX, m_data);
    printf("pos: %.3f, vel: %.3f, cur: %.3f\n",
           m_data[0].pos, m_data[0].vel, m_data[0].tau);

    osal_usleep(1000 * 500);
  }

  return 0;
}
