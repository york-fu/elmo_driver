#include "elmo_motor.h"

extern pthread_mutex_t mtx;
extern uint32 _encoder_range[NUM_SLAVE_MAX];
extern uint32 _rated_current[NUM_SLAVE_MAX];
extern struct ELMORead *elmoI[NUM_SLAVE_MAX];
extern struct ELMOWrite *elmoO[NUM_SLAVE_MAX];

static double position_offset[NUM_SLAVE_MAX] = {0};

int8_t EM_init(const char *ifname, double dt, MotorOptions_t opt)
{
  int8_t ret = 0, err_code = 1;
  ret = elmo_init(ifname, dt, opt);
  if (ret != 0)
  {
    printf("Failed to elmo init! return %d\n", ret);
    return -1;
  }
  return 0;
}

int8_t EM_deInit()
{
  return elmo_deinit();
}

int8_t EM_setPositionsOffset(double *offset, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    position_offset[i] = offset[i];
  }
}

int8_t EM_setPositions(uint8_t *ids, uint8_t num, MotorParam_t *param)
{
  uint16_t index;
  pthread_mutex_lock(&mtx);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    elmoO[index]->target_position = (param[index].position + position_offset[index]) * (_encoder_range[index] / 360.0);
    elmoO[index]->position_offset = param[index].positionOffset * (_encoder_range[index] / 360.0);
    elmoO[index]->velocit_offset = param[index].velocityOffset * (_encoder_range[index] / 360.0);
    elmoO[index]->torque_offset = param[index].torqueOffset * (1000.0 / _rated_current[index]) * 1000;
    elmoO[index]->max_torque = param[index].maxTorque * (1000.0 / _rated_current[index]) * 1000;
    elmoO[index]->mode_of_opration = MODE_CSP;
    elmoO[index]->control_word = ctrlWord(elmoI[index]->status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx);
  return 0;
}

int8_t EM_setVelocities(uint8_t *ids, uint8_t num, MotorParam_t *param)
{
  uint16_t index;
  pthread_mutex_lock(&mtx);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    elmoO[index]->target_velocity = param[index].velocity * (_encoder_range[index] / 360.0);
    elmoO[index]->velocit_offset = param[index].velocityOffset * (_encoder_range[index] / 360.0);
    elmoO[index]->torque_offset = param[index].torqueOffset * (1000.0 / _rated_current[index]) * 1000;
    elmoO[index]->max_torque = param[index].maxTorque * (1000.0 / _rated_current[index]) * 1000;
    elmoO[index]->mode_of_opration = MODE_CSV;
    elmoO[index]->control_word = ctrlWord(elmoI[index]->status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx);
  return 0;
}

int8_t EM_setTorques(uint8_t *ids, uint8_t num, MotorParam_t *param)
{
  uint16_t index;
  pthread_mutex_lock(&mtx);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    elmoO[index]->target_torque = param[index].torque * (1000.0 / _rated_current[index]) * 1000;
    elmoO[index]->torque_offset = param[index].torqueOffset * (1000.0 / _rated_current[index]) * 1000;
    elmoO[index]->max_torque = param[index].maxTorque * (1000.0 / _rated_current[index]) * 1000;
    elmoO[index]->mode_of_opration = MODE_CST;
    elmoO[index]->control_word = ctrlWord(elmoI[index]->status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx);
  return 0;
}

int8_t EM_getData(uint8_t *ids, uint8_t num, MotorParam_t *data)
{
  uint16_t index;
  pthread_mutex_lock(&mtx);
  for (uint8_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    data[i].position = elmoI[index]->position_actual_value * (360.0 / _encoder_range[index]) - position_offset[index];
    data[i].velocity = elmoI[index]->velocity_actual_value * (360.0 / _encoder_range[index]);
    data[i].torque = elmoI[index]->torque_actual_value * (_rated_current[index] / 1000.0) / 1000.0;
  }
  pthread_mutex_unlock(&mtx);
}