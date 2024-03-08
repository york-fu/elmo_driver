/**
 * @file elmo_motor.h
 * @author york (york-fu@outlook.com)
 * @brief
 * @version 0.1
 * @date 2021-09-09
 *
 * @copyright Copyright (c) 2023
 *
 */
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
    double pos;
    double vel;
    double acc;
    double tau;
    double max_tau;
    double pos_ff;
    double vel_ff;
    double tau_ff;
  } MotorData_t;

  /**
   * @brief init
   *
   * @param ec_cfg
   * @return int8_t
   */
  int8_t em_init(ECMConfig_t *ec_cfg, MotorConfig_t *m_cfg);
  /**
   * @brief deinit
   *
   * @return int8_t
   */
  int8_t em_deinit();

  int8_t em_enable(uint16_t id, uint8_t mode);
  int8_t em_enable_all(uint16_t try_cnt, uint8_t mode);
  int8_t em_disable(uint16_t id);
  int8_t em_disable_all(uint16_t try_cnt);

  /**
   * @brief set positions
   *
   * @param ids
   * @param num
   * @param data
   * @return int8_t
   */
  int8_t em_set_positions(uint8_t *ids, uint8_t num, MotorData_t *data);
  /**
   * @brief set velocities
   *
   * @param ids
   * @param num
   * @param data
   * @return int8_t
   */
  int8_t em_set_velocities(uint8_t *ids, uint8_t num, MotorData_t *data);
  /**
   * @brief set torques
   *
   * @param ids
   * @param num
   * @param data
   * @return int8_t
   */
  int8_t em_set_torques(uint8_t *ids, uint8_t num, MotorData_t *data);
  /**
   * @brief get_data
   *
   * @param ids
   * @param num
   * @param data
   * @return int8_t
   */
  int8_t em_get_data(uint8_t *ids, uint8_t num, MotorData_t *data);

#ifdef __cplusplus
}
#endif

#endif
