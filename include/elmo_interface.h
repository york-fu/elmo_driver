#ifndef _elmo_interface_h_
#define _elmo_interface_h_

#include "ec_elmo.h"
#include "math.h"


int8_t hard_init();

int8_t set_joint_position(uint8_t *ids, uint8_t id_num, double_t *postion);
int8_t set_joint_velocity(uint8_t *ids, uint8_t id_num, double_t *velocity);
int8_t set_joint_torque(uint8_t *ids, uint8_t id_num, double_t *torque);

int8_t get_joint_position(uint8_t *ids, uint8_t id_num, double_t *postion);
int8_t get_joint_velocity(uint8_t *ids, uint8_t id_num, double_t *velocity);
int8_t get_joint_torque(uint8_t *ids, uint8_t id_num, double_t *torque);

#endif
