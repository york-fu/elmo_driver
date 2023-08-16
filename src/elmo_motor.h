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
    double max_torque;
    double position_offset;
    double velocity_offset;
    double torque_offset;
    double acceleration;
  } MotorParam_t;

  int8_t em_init(ECMConfig_t ec_cfg);
  int8_t em_deinit();

  MotorConfig_t em_get_motor_cfg();
  int8_t em_set_motor_cfg(MotorConfig_t cfg, uint16_t num);

  int8_t em_enable(uint16_t id);
  int8_t em_enable_all(uint16_t try_cnt);
  int8_t em_disable(uint16_t id);
  int8_t em_disable_all(uint16_t try_cnt);

  int8_t em_set_positions(uint8_t *ids, uint8_t num, MotorParam_t *param);
  int8_t em_set_velocities(uint8_t *ids, uint8_t num, MotorParam_t *param);
  int8_t em_set_torques(uint8_t *ids, uint8_t num, MotorParam_t *param);
  int8_t em_get_data(uint8_t *ids, uint8_t num, MotorParam_t *data);

#ifdef __cplusplus
}
#endif

#endif
