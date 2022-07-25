#ifndef _ec_elmo_h_
#define _ec_elmo_h_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "ethercat.h"

#define NUM_SLAVE_MAX 20

#define MODE_CSP (8)
#define MODE_CSV (9)
#define MODE_CST (10)

#pragma pack(1)
typedef struct ELMORead
{
  int32 position_actual_value;
  uint32 digital_inputs;
  int32 velocity_actual_value;
  uint16 status_word;
  int16 torque_actual_value;
  int16 mode_of_opration_display;
} ELMORead;

typedef struct ELMOWrite
{
  int32 target_position;
  int32 target_velocity;
  int16 target_torque;
  uint16 max_torque;
  uint16 control_word;
  int16 mode_of_opration;
  int32_t position_offset;
  int32_t velocit_offset;
  int16_t torque_offset;
} ELMOWrite;
#pragma pack()

typedef struct
{
  uint16_t size;
  uint32_t *encoder_range;
  uint8_t position_limit;
  double *position_limit_min;
  double *position_limit_max;
} MotorOptions_t;

uint16 ctrlWord(uint16 state_word);
int8 elmo_init(const char *ifname, double dt, MotorOptions_t opt);
int8 elmo_deinit();

#endif
