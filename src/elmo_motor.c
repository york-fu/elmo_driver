#include "elmo_motor.h"

extern pthread_mutex_t mtx_IOMap;
extern uint8_t _sync_running;
extern uint32_t encoder_range[12];
extern uint32_t rated_current[12];
extern struct ELMOsRead *elmoI[12];
extern struct ELMOsWrite *elmoO[12];

static double_t motorOffset[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int8_t elmoInit()
{
  int8_t ret = 0, err_code = 1;
  ret = ec_elmo_init("enp2s0");
  if (ret != 0)
  {
    printf("elmo init failed! return %d\n", ret);
    return err_code;
  }
  return 0;
}

int8_t elmoDeInit()
{
  _sync_running = 0;
  osal_usleep(2000);
  elmo_set_state(EC_STATE_INIT);
  ec_close();
  return 0;
}

int8_t setJointOffset(double_t *offset, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    motorOffset[i] = offset[i];
  }
}

int8_t setJointPosition(uint8_t *ids, uint8_t id_num, JointParam_t *param)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    elmoO[ids[i] - 1]->target_position = (param[ids[i] - 1].position + motorOffset[ids[i] - 1]) * (encoder_range[ids[i] - 1] / 360.0);
    elmoO[ids[i] - 1]->position_offset = param[ids[i] - 1].positionOffset * (encoder_range[ids[i] - 1] / 360.0);
    elmoO[ids[i] - 1]->velocit_offset = param[ids[i] - 1].velocityOffset * (encoder_range[ids[i] - 1] / 360.0);
    elmoO[ids[i] - 1]->torque_offset = param[ids[i] - 1].torqueOffset * (1000.0 / rated_current[ids[i] - 1]) * 1000;
    elmoO[ids[i] - 1]->mode_of_opration = MODE_CSP;
    elmoO[ids[i] - 1]->control_word = ctrlWord(elmoI[ids[i] - 1]->status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx_IOMap);
  return 0;
}

int8_t setJointVelocity(uint8_t *ids, uint8_t id_num, JointParam_t *param)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    elmoO[ids[i] - 1]->target_velocity = param[ids[i] - 1].velocity * (encoder_range[ids[i] - 1] / 360.0);
    elmoO[ids[i] - 1]->velocit_offset = param[ids[i] - 1].velocityOffset * (encoder_range[ids[i] - 1] / 360.0);
    elmoO[ids[i] - 1]->torque_offset = param[ids[i] - 1].torqueOffset * (1000.0 / rated_current[ids[i] - 1]) * 1000;
    elmoO[ids[i] - 1]->mode_of_opration = MODE_CSV;
    elmoO[ids[i] - 1]->control_word = ctrlWord(elmoI[ids[i] - 1]->status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx_IOMap);
  return 0;
}

int8_t setJointTorque(uint8_t *ids, uint8_t id_num, JointParam_t *param)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    elmoO[ids[i] - 1]->target_torque = param[ids[i] - 1].torque * (1000.0 / rated_current[ids[i] - 1]) * 1000;
    elmoO[ids[i] - 1]->torque_offset = param[ids[i] - 1].torqueOffset * (1000.0 / rated_current[ids[i] - 1]) * 1000;
    elmoO[ids[i] - 1]->mode_of_opration = MODE_CST;
    elmoO[ids[i] - 1]->control_word = ctrlWord(elmoI[ids[i] - 1]->status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx_IOMap);
  return 0;
}

int8_t getJointData(uint8_t *ids, uint8_t id_num, JointParam_t *data)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    data[i].position = elmoI[ids[i] - 1]->position_actual_value * (360.0 / encoder_range[ids[i] - 1]) - motorOffset[ids[i] - 1];
    data[i].velocity = elmoI[ids[i] - 1]->velocity_actual_value * (360.0 / encoder_range[ids[i] - 1]);
    data[i].torque = elmoI[ids[i] - 1]->torque_actual_value * (rated_current[ids[i] - 1] / 1000.0) / 1000.0;
  }
  pthread_mutex_unlock(&mtx_IOMap);
}