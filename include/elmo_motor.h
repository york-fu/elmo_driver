#ifndef _elmo_interface_h_
#define _elmo_interface_h_

#ifdef __cplusplus
extern "C" {
#endif

#include "ec_elmo.h"
#include "math.h"

typedef struct
{
  double_t position;
  double_t velocity;
  double_t torque;
  double_t positionOffset;
  double_t velocityOffset;
  double_t torqueOffset;
  double_t acceleration;
} JointParam_t;

int8_t elmoInit();
int8_t elmoDeInit();
int8_t setJointOffset(double_t *offset, uint16_t len);
int8_t setJointPosition(uint8_t *ids, uint8_t id_num, JointParam_t *param);
int8_t setJointVelocity(uint8_t *ids, uint8_t id_num, JointParam_t *param);
int8_t setJointTorque(uint8_t *ids, uint8_t id_num, JointParam_t *param);
int8_t getJointData(uint8_t *ids, uint8_t id_num, JointParam_t *data);

#ifdef __cplusplus
}
#endif

#endif
