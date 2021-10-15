#ifndef _elmo_interface_h_
#define _elmo_interface_h_

#include "ec_elmo.h"
#include "math.h"

typedef struct
{
  double_t position;
  double_t velocity;
  double_t acceleration;
  double_t torque;
} JointParam_t;

int8_t device_init();
int8_t device_close();

int8_t set_joint_position(uint8_t *ids, uint8_t id_num, JointParam_t *param);
int8_t set_joint_velocity(uint8_t *ids, uint8_t id_num, JointParam_t *param);
int8_t set_joint_torque(uint8_t *ids, uint8_t id_num, JointParam_t *param);

int8_t get_joint_position(uint8_t *ids, uint8_t id_num, double_t *postion);
int8_t get_joint_velocity(uint8_t *ids, uint8_t id_num, double_t *velocity);
int8_t get_joint_torque(uint8_t *ids, uint8_t id_num, double_t *torque);
int8_t get_joint_data(uint8_t *ids, uint8_t id_num, JointParam_t *data);

#endif
