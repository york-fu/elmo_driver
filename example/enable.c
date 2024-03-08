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

MotorConfig_t motor_cfg;

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

  while (1)
  {
    osal_usleep(5000);
  }

  return 0;
}
