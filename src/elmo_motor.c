#include "elmo_motor.h"

#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_FAULT_BIT 3

#define MODE_CSP (8)
#define MODE_CSV (9)
#define MODE_CST (10)

static MotorConfig_t motor_cfg;

static pthread_mutex_t *mtx;
extern ELMORead_t *elmoI;
extern ELMOWrite_t *elmoO;

int8_t em_init(ECMConfig_t ec_cfg)
{
  int8_t ret = 0;
  ret = ec_elmo_init(ec_cfg, &motor_cfg);
  if (ret != 0)
  {
    return ret;
  }

  mtx = ec_elmo_get_mtx();
  // ec_elmo_get_data_ptr(elmoI, elmoO);

  return 0;
}

int8_t em_deinit()
{
  return ec_elmo_deinit();
}

MotorConfig_t em_get_motor_cfg()
{
  return motor_cfg;
}

int8_t em_set_motor_cfg(MotorConfig_t cfg, uint16_t num)
{
  motor_cfg = cfg;
  return 0;
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

int8_t em_enable(uint16_t id)
{
  if (id < 1)
  {
    printf("Illegal id: %d\n", id);
    return -1;
  }
  uint16_t index, sw;
  index = id - 1;
  pthread_mutex_lock(mtx);
  sw = elmoI[index].status_word & 0x6f;
  elmoO[index].target_position = elmoI[index].position_actual_value;
  elmoO[index].position_offset = 0;
  elmoO[index].velocity_offset = 0;
  elmoO[index].torque_offset = 0;
  elmoO[index].max_torque = 1000;
  elmoO[index].mode_of_opration = MODE_CSP;
  elmoO[index].control_word = sw2cw(sw);
  pthread_mutex_unlock(mtx);
  if (sw == 0x27)
  {
    printf("Motor %d enable success\n", id);
    motor_cfg.sw_check = 1;
    return 0;
  }
  return -2;
}

int8_t em_enable_all(uint16_t try_cnt)
{
  int8_t result = 0;
  printf("Wait %d all motor enable...\n", motor_cfg.num);
  osal_usleep(1000);
  for (uint16_t i = 1; i <= motor_cfg.num; i++)
  {
    for (uint16_t j = 0; j < try_cnt; j++)
    {
      result = em_enable(i);
      if (result == 0)
        break;
      osal_usleep(1000);
    }
    if (result != 0)
    {
      printf("Wait motor %d enable failed\n", i);
      return -1;
    }
  }
  return 0;
}

int8_t em_disable(uint16_t id)
{
  // to do
  // motor_cfg.sw_check = 0;
  return 0;
}

int8_t em_disable_all(uint16_t try_cnt)
{
  // to do
  return 0;
}

int8_t em_set_positions(uint8_t *ids, uint8_t num, MotorParam_t *param)
{
  uint16_t index;
  pthread_mutex_lock(mtx);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    elmoO[index].target_position = (param[index].position + motor_cfg.offset[index]) * (motor_cfg.range[index] * motor_cfg.gear / motor_cfg.circle_unit);
    elmoO[index].position_offset = param[index].position_offset * (motor_cfg.range[index] * motor_cfg.gear / motor_cfg.circle_unit);
    elmoO[index].velocity_offset = param[index].velocity_offset * (motor_cfg.range[index] * motor_cfg.gear / motor_cfg.circle_unit);
    elmoO[index].torque_offset = param[index].torque_offset * (1000.0 / motor_cfg.rated_current[index]) * 1000;
    elmoO[index].max_torque = param[index].max_torque * (1000.0 / motor_cfg.rated_current[index]) * 1000;
    elmoO[index].mode_of_opration = MODE_CSP;
    elmoO[index].control_word = sw2cw(elmoI[index].status_word & 0x6f);
  }
  pthread_mutex_unlock(mtx);
  return 0;
}

int8_t em_set_velocities(uint8_t *ids, uint8_t num, MotorParam_t *param)
{
  uint16_t index;
  pthread_mutex_lock(mtx);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    elmoO[index].target_velocity = param[index].velocity * (motor_cfg.range[index] * motor_cfg.gear / motor_cfg.circle_unit);
    elmoO[index].velocity_offset = param[index].velocity_offset * (motor_cfg.range[index] * motor_cfg.gear / motor_cfg.circle_unit);
    elmoO[index].torque_offset = param[index].torque_offset * (1000.0 / motor_cfg.rated_current[index]) * 1000;
    elmoO[index].max_torque = param[index].max_torque * (1000.0 / motor_cfg.rated_current[index]) * 1000;
    elmoO[index].mode_of_opration = MODE_CSV;
    elmoO[index].control_word = sw2cw(elmoI[index].status_word & 0x6f);
  }
  pthread_mutex_unlock(mtx);
  return 0;
}

int8_t em_set_torques(uint8_t *ids, uint8_t num, MotorParam_t *param)
{
  uint16_t index;
  pthread_mutex_lock(mtx);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    elmoO[index].target_torque = param[index].torque * (1000.0 / motor_cfg.rated_current[index]) * 1000;
    elmoO[index].torque_offset = param[index].torque_offset * (1000.0 / motor_cfg.rated_current[index]) * 1000;
    elmoO[index].max_torque = param[index].max_torque * (1000.0 / motor_cfg.rated_current[index]) * 1000;
    elmoO[index].mode_of_opration = MODE_CST;
    elmoO[index].control_word = sw2cw(elmoI[index].status_word & 0x6f);
  }
  pthread_mutex_unlock(mtx);
  return 0;
}

int8_t em_get_data(uint8_t *ids, uint8_t num, MotorParam_t *data)
{
  uint16_t index;
  pthread_mutex_lock(mtx);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    data[i].position = elmoI[index].position_actual_value * (motor_cfg.circle_unit / (motor_cfg.range[index] * motor_cfg.gear)) - motor_cfg.offset[index];
    data[i].velocity = elmoI[index].velocity_actual_value * (motor_cfg.circle_unit / (motor_cfg.range[index] * motor_cfg.gear));
    data[i].torque = elmoI[index].torque_actual_value * (motor_cfg.rated_current[index] / 1000.0) / 1000.0;
  }
  pthread_mutex_unlock(mtx);
  return 0;
}