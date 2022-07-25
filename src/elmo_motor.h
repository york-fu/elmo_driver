#ifndef _elmo_motor_h_
#define _elmo_motor_h_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ec_elmo.h"
#include "math.h"

typedef struct
{
  double position;
  double velocity;
  double torque;
  double maxTorque;
  double positionOffset;
  double velocityOffset;
  double torqueOffset;
  double acceleration;
} MotorParam_t;

int8_t EM_init(const char *ifname, double dt, MotorOptions_t opt);
int8_t EM_deInit();
int8_t EM_setPositionsOffset(double *offset, uint16_t len);
int8_t EM_setPositions(uint8_t *ids, uint8_t num, MotorParam_t *param);
int8_t EM_setVelocities(uint8_t *ids, uint8_t num, MotorParam_t *param);
int8_t EM_setTorques(uint8_t *ids, uint8_t num, MotorParam_t *param);
int8_t EM_getData(uint8_t *ids, uint8_t num, MotorParam_t *data);

#ifdef __cplusplus
}
#endif

#endif
