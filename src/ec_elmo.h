#ifndef _ec_elmo_h_
#define _ec_elmo_h_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "ethercat.h"

#define NUM_SLAVE_MAX 32

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
  uint8_t sw_check;
  double circle_unit;
  double gear;
  double offset[NUM_SLAVE_MAX];
  uint32_t range[NUM_SLAVE_MAX];
  uint32_t rated_current[NUM_SLAVE_MAX];
} MotorConfig_t;

pthread_mutex_t *ec_elmo_get_mtx();
int8_t ec_elmo_get_data_ptr(ELMORead_t **in, ELMOWrite_t **out);

int8_t ec_elmo_init(ECMConfig_t ec_cfg, MotorConfig_t *m_cfg);
int8_t ec_elmo_deinit();

#endif
