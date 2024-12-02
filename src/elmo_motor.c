/**
 * @file elmo_motor.c
 * @author york (york-fu@outlook.com)
 * @brief
 * @version 0.1
 * @date 2021-09-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "elmo_motor.h"

#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_FAULT_BIT 3

#define MODE_CSP (8)
#define MODE_CSV (9)
#define MODE_CST (10)

static MotorConfig_t *motor_cfg;

extern pthread_mutex_t mtx_pdo;
extern ELMORead_t *elmoI;
extern ELMOWrite_t *elmoO;

int8_t em_init(ECMConfig_t *ec_cfg, MotorConfig_t *m_cfg)
{
  motor_cfg = m_cfg;

  int8_t ret = 0;
  ret = elmo_init(ec_cfg, m_cfg);
  if (ret != 0)
  {
    return ret;
  }

  // mtx_pdo = ec_elmo_get_mtx();
  // ec_elmo_get_data_ptr(elmoI, elmoO);

  return 0;
}

int8_t em_deinit()
{
  return elmo_deinit();
}

uint16_t sw2cw(uint16_t state_word)
{
  if (!(state_word & (1 << STATUSWORD_OPERATION_ENABLE_BIT)))
  {
    if (!(state_word & (1 << STATUSWORD_SWITCHED_ON_BIT)))
    {
      if (!(state_word & (1 << STATUSWORD_READY_TO_SWITCH_ON_BIT)))
      {
        if ((state_word & (1 << STATUSWORD_FAULT_BIT)))
        {
          return 0x80; // fault reset
        }
        else
        {
          return 0x06; // shutdown
        }
      }
      else
      {
        return 0x07; // switch on
      }
    }
    else
    {
      return 0x0F; // switch on
    }
  }
  else
  {
    return 0x0F; // switch on
  }

  return 0;
}

static uint16_t sw2cw2(const uint16_t state_word) // state to control
{
  static uint8_t st = 0;
  static uint16_t cnt = 0;

  if (!(state_word & (1 << STATUSWORD_OPERATION_ENABLE_BIT)))
  {
    if (!(state_word & (1 << STATUSWORD_SWITCHED_ON_BIT)))
    {
      if (!(state_word & (1 << STATUSWORD_READY_TO_SWITCH_ON_BIT)))
      {
        if ((state_word & (1 << STATUSWORD_FAULT_BIT)))
        {
          if (st == 0)
            return 0x80; // fault reset
          else
            return 0x06;
        }
        else
        {
          if (st == 0)
          {
            st = 1;
            cnt = 0;
            return 0x06;
          }
          else if (st == 1)
          {
            cnt++;
            return 0x06;
          }
          else
          {
            st = 0;
            cnt = 0;
            return 0x06;
          }
          // shutdown
        }
      }
      else
      {
        if (st == 1)
        {
          if (cnt > 20)
          {
            cnt = 0;
            st = 2;
            return 0x07; // switch on
          }
          else
          {
            cnt++;
            return 0x06;
          }
        }
        else
        {
          cnt = 0;
          st = 1;
          return 0x06;
        }
      }
    }
    else
    {
      if (st == 2)
      {
        if (cnt > 20)
        {
          cnt = 0;
          st = 3;
          return 0x0F; // op
        }
        else
        {
          cnt++;
          return 0x07;
        }
      }
      else
      {
        if (st > 2)
          return 0x0F; // op
        else
          return 0x07; // switch on
      }
    }
  }
  else
  {
    return 0x0F; // switch on
  }

  return 0;
}

int8_t em_enable(uint16_t id, uint8_t mode)
{
  if (id < 1)
  {
    printf("Illegal id: %d\n", id);
    return -1;
  }
  int16_t op_mode = MODE_CST;
  switch (mode)
  {
  case 0:
    op_mode = MODE_CST;
    break;
  case 1:
    op_mode = MODE_CSV;
    break;
  case 2:
    op_mode = MODE_CSP;
    break;
  }
  uint16_t index, sw, cw;
  index = id - 1;
  pthread_mutex_lock(&mtx_pdo);
  sw = elmoI[index].status_word & 0x6f;
  pthread_mutex_unlock(&mtx_pdo);
  cw = sw2cw(sw);
  pthread_mutex_lock(&mtx_pdo);
  elmoO[index].target_position = elmoI[index].position_actual_value;
  elmoO[index].position_offset = 0;
  elmoO[index].velocity_offset = 0;
  elmoO[index].torque_offset = 0;
  elmoO[index].max_torque = 200;
  elmoO[index].mode_of_opration = op_mode;
  elmoO[index].control_word = cw;
  pthread_mutex_unlock(&mtx_pdo);
  if (sw == 0x27)
  {
    printf("Motor %d enabled successfully\n", id);
    return 0;
  }
  return -2;
}

int8_t em_enable_all(uint16_t try_cnt, uint8_t mode)
{
  int8_t result = 0;
  printf("Wait %d all motor enable...\n", motor_cfg->num);
  for (uint16_t i = 1; i <= motor_cfg->num; i++)
  {
    for (uint16_t j = 0; j < try_cnt; j++)
    {
      result = em_enable(i, mode);
      if (result == 0)
        break;
      osal_usleep(1000);
    }
    if (result != 0)
    {
      printf("Wait motor %d enable failed\n", i);
      return -1;
    }
    motor_cfg->enable[i - 1] = 1;
  }
  return 0;
}

int8_t em_disable(uint16_t id)
{
  // to do
  return 0;
}

int8_t em_disable_all(uint16_t try_cnt)
{
  // to do
  // motor_cfg->enable[i - 1] = 0;
  return 0;
}

int8_t em_set_positions(uint8_t *ids, uint8_t num, MotorData_t *data)
{
  uint16_t index;
  pthread_mutex_lock(&mtx_pdo);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    elmoO[index].target_position = (data[index].pos + motor_cfg->pos_offset[index]) * (motor_cfg->pos_enc_range[index] * motor_cfg->gear[index] / motor_cfg->circle_unit);
    elmoO[index].position_offset = data[index].pos_ff * (motor_cfg->pos_enc_range[index] * motor_cfg->gear[index] / motor_cfg->circle_unit);
    elmoO[index].velocity_offset = data[index].vel_ff * (motor_cfg->vel_enc_range[index] * motor_cfg->gear[index] / motor_cfg->circle_unit);
    elmoO[index].torque_offset = data[index].tau_ff * (1000.0 / motor_cfg->rated_current[index]) * 1000;
    elmoO[index].max_torque = data[index].max_tau * (1000.0 / motor_cfg->rated_current[index]) * 1000;
    elmoO[index].mode_of_opration = MODE_CSP;
    elmoO[index].control_word = sw2cw(elmoI[index].status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx_pdo);
  return 0;
}

int8_t em_set_velocities(uint8_t *ids, uint8_t num, MotorData_t *data)
{
  uint16_t index;
  pthread_mutex_lock(&mtx_pdo);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    elmoO[index].target_velocity = data[index].vel * (motor_cfg->vel_enc_range[index] * motor_cfg->gear[index] / motor_cfg->circle_unit);
    elmoO[index].velocity_offset = data[index].vel_ff * (motor_cfg->vel_enc_range[index] * motor_cfg->gear[index] / motor_cfg->circle_unit);
    elmoO[index].torque_offset = data[index].tau_ff * (1000.0 / motor_cfg->rated_current[index]) * 1000;
    elmoO[index].max_torque = data[index].max_tau * (1000.0 / motor_cfg->rated_current[index]) * 1000;
    elmoO[index].mode_of_opration = MODE_CSV;
    elmoO[index].control_word = sw2cw(elmoI[index].status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx_pdo);
  return 0;
}

int8_t em_set_torques(uint8_t *ids, uint8_t num, MotorData_t *data)
{
  uint16_t index;
  pthread_mutex_lock(&mtx_pdo);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    elmoO[index].target_torque = data[index].tau * (1000.0 / motor_cfg->rated_current[index]) * 1000;
    elmoO[index].torque_offset = data[index].tau_ff * (1000.0 / motor_cfg->rated_current[index]) * 1000;
    elmoO[index].max_torque = data[index].max_tau * (1000.0 / motor_cfg->rated_current[index]) * 1000;
    elmoO[index].mode_of_opration = MODE_CST;
    elmoO[index].control_word = sw2cw(elmoI[index].status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx_pdo);
  return 0;
}

int8_t em_get_data(uint8_t *ids, uint8_t num, MotorData_t *data)
{
  uint16_t index;
  pthread_mutex_lock(&mtx_pdo);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    data[i].pos = elmoI[index].position_actual_value * (motor_cfg->circle_unit / (motor_cfg->pos_enc_range[index] * motor_cfg->gear[index])) - motor_cfg->pos_offset[index];
    data[i].vel = elmoI[index].velocity_actual_value * (motor_cfg->circle_unit / (motor_cfg->vel_enc_range[index] * motor_cfg->gear[index]));
    data[i].tau = elmoI[index].torque_actual_value * (motor_cfg->rated_current[index] / 1000.0) / 1000.0;
  }
  pthread_mutex_unlock(&mtx_pdo);
  return 0;
}