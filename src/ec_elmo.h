/**
 * @file ec_elmo.h
 * @author york (york-fu@outlook.com)
 * @brief
 * @version 0.1
 * @date 2021-09-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef _ec_elmo_h_
#define _ec_elmo_h_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "ethercat.h"

#define NUM_SLAVE_MAX 64

#pragma pack(1)
typedef struct ELMORead
{
  int32 position_actual_value;
  uint32 digital_inputs;
  int32 velocity_actual_value;
  uint16 status_word;
  int16 torque_actual_value;
  int16 mode_of_opration_display;
} ELMORead_t;

typedef struct ELMOWrite
{
  int32 target_position;
  int32 target_velocity;
  int16 target_torque;
  uint16 max_torque;
  uint16 control_word;
  int16 mode_of_opration;
  int32 position_offset;
  int32 velocity_offset;
  int16 torque_offset;
} ELMOWrite_t;
#pragma pack()

typedef struct
{
  char ifname[16];
  double dt;
} ECMConfig_t;

typedef struct
{
  uint16_t num;
  uint8_t enable;
  double circle_unit;
  uint32_t rated_current[NUM_SLAVE_MAX];
  double gear[NUM_SLAVE_MAX];
  uint32_t range[NUM_SLAVE_MAX];
  double offset[NUM_SLAVE_MAX];
} MotorConfig_t;

int8_t elmo_init(ECMConfig_t *ec_cfg, MotorConfig_t *m_cfg);
int8_t elmo_deinit();

#endif
