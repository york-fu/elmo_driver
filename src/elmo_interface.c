#include "elmo_interface.h"

extern struct JointsRead *JointI[12];
extern struct JointsWrite *JointO[12];
uint32_t encoder_range[12];

int8_t hard_init()
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

int8_t set_joint_position(uint8_t *ids, uint8_t id_num, double_t *postion)
{
  for (uint8_t i = 0; i < id_num; i++)
  {
    JointO[ids[i] - 1]->control_word = to_ctrl_word(JointI[ids[i] - 1]->status_word & 0x6f);
    JointO[ids[i] - 1]->mode_of_opration = MODE_CSP;
    JointO[ids[i] - 1]->target_position = postion[ids[i] - 1] * (encoder_range[ids[i] - 1] / 360.0);
  }
  return 0;
}

int8_t set_joint_velocity(uint8_t *ids, uint8_t id_num, double_t *velocity)
{
  for (uint8_t i = 0; i < id_num; i++)
  {
    JointO[ids[i] - 1]->control_word = to_ctrl_word(JointI[ids[i] - 1]->status_word & 0x6f);
    JointO[ids[i] - 1]->mode_of_opration = MODE_CSV;
    JointO[ids[i] - 1]->target_velocity = velocity[ids[i] - 1];
  }
  return 0;
}

int8_t set_joint_torque(uint8_t *ids, uint8_t id_num, double_t *torque)
{
  for (uint8_t i = 0; i < id_num; i++)
  {
    JointO[ids[i] - 1]->control_word = to_ctrl_word(JointI[ids[i] - 1]->status_word & 0x6f);
    JointO[ids[i] - 1]->mode_of_opration = MODE_CST;
    JointO[ids[i] - 1]->target_velocity = torque[ids[i] - 1];
  }
  return 0;
}

int8_t get_joint_position(uint8_t *ids, uint8_t id_num, double_t *postion)
{
  for (uint8_t i = 0; i < id_num; i++)
  {
    postion[i] = JointI[ids[i] - 1]->position_actual_value * (360.0 / encoder_range[ids[i] - 1]);
  }
  return 0;
}

int8_t get_joint_velocity(uint8_t *ids, uint8_t id_num, double_t *velocity)
{
  for (uint8_t i = 0; i < id_num; i++)
  {
    velocity[i] = JointI[ids[i] - 1]->velocity_actual_value;
  }
  return 0;
}

int8_t get_joint_torque(uint8_t *ids, uint8_t id_num, double_t *torque)
{
  for (uint8_t i = 0; i < id_num; i++)
  {
    torque[i] = JointI[ids[i] - 1]->torque_actual_value;
  }
  return 0;
}
