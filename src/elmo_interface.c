#include "elmo_interface.h"

extern pthread_mutex_t mtx_IOMap;
extern uint8_t _sync_running;
extern struct MotorsRead *MotorI[12];
extern struct MotorsWrite *MotorO[12];
extern uint32_t encoder_range[12];
extern uint32_t rated_current[12];

int8_t device_init()
{
  int8_t ret = 0;
  ret = ec_elmo_init("enp2s0");
  if (ret != 0)
  {
    printf("elmo init failed!\n");
    return ret;
  }
  return 0;
}

int8_t device_close()
{
  _sync_running = 0;
  osal_usleep(2000);
  set_ec_state(EC_STATE_PRE_OP);
  ec_close();
  return 0;
}

int8_t set_joint_position(uint8_t *ids, uint8_t id_num, JointParam_t *param)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    MotorO[ids[i] - 1]->mode_of_opration = MODE_CSP;
    MotorO[ids[i] - 1]->target_position = param[ids[i] - 1].position * (encoder_range[ids[i] - 1] / 360.0);
    MotorO[ids[i] - 1]->control_word = ctrlWord(MotorI[ids[i] - 1]->status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx_IOMap);
  return 0;
}

int8_t set_joint_velocity(uint8_t *ids, uint8_t id_num, JointParam_t *param)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    MotorO[ids[i] - 1]->mode_of_opration = MODE_CSV;
    MotorO[ids[i] - 1]->target_velocity = param[ids[i] - 1].velocity * (encoder_range[ids[i] - 1] / 360.0);
    MotorO[ids[i] - 1]->control_word = ctrlWord(MotorI[ids[i] - 1]->status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx_IOMap);
  return 0;
}

int8_t set_joint_torque(uint8_t *ids, uint8_t id_num, JointParam_t *param)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    MotorO[ids[i] - 1]->mode_of_opration = MODE_CST;
    MotorO[ids[i] - 1]->target_torque = param[ids[i] - 1].torque * (1000.0 / rated_current[ids[i] - 1]) * 1000;
    MotorO[ids[i] - 1]->control_word = ctrlWord(MotorI[ids[i] - 1]->status_word & 0x6f);
  }
  pthread_mutex_unlock(&mtx_IOMap);
  return 0;
}

int8_t get_joint_position(uint8_t *ids, uint8_t id_num, double_t *postion)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    postion[i] = MotorI[ids[i] - 1]->position_actual_value * (360.0 / encoder_range[ids[i] - 1]);
  }
  pthread_mutex_unlock(&mtx_IOMap);
  return 0;
}

int8_t get_joint_velocity(uint8_t *ids, uint8_t id_num, double_t *velocity)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    velocity[i] = MotorI[ids[i] - 1]->velocity_actual_value * (360.0 / encoder_range[ids[i] - 1]);
  }
  pthread_mutex_unlock(&mtx_IOMap);
  return 0;
}

int8_t get_joint_torque(uint8_t *ids, uint8_t id_num, double_t *torque)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    torque[i] = MotorI[ids[i] - 1]->torque_actual_value * (rated_current[ids[i] - 1] / 1000.0) / 1000.0;
  }
  pthread_mutex_unlock(&mtx_IOMap);
  return 0;
}

int8_t get_joint_data(uint8_t *ids, uint8_t id_num, JointParam_t *data)
{
  pthread_mutex_lock(&mtx_IOMap);
  for (uint8_t i = 0; i < id_num; i++)
  {
    data[i].position = MotorI[ids[i] - 1]->position_actual_value * (360.0 / encoder_range[ids[i] - 1]);
    data[i].velocity = MotorI[ids[i] - 1]->velocity_actual_value * (360.0 / encoder_range[ids[i] - 1]);
    data[i].torque = MotorI[ids[i] - 1]->torque_actual_value * (rated_current[ids[i] - 1] / 1000.0) / 1000.0;
  }
  pthread_mutex_unlock(&mtx_IOMap);
}
