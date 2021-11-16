#ifndef _ec_elmo_h_
#define _ec_elmo_h_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "soem/ethercat.h"

#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_FAULT_BIT 3

#define MODE_CSP (8)
#define MODE_CSV (9)
#define MODE_CST (10)

#pragma pack(1)
typedef struct ELMOsRead
{
  int32 position_actual_value;
  uint32 digital_inputs;
  int32 velocity_actual_value;
  uint16 status_word;
  int16 torque_actual_value;
  int16 mode_of_opration_display;
} ELMOsRead;

typedef struct ELMOsWrite
{
  int32 target_position;
  int32 target_velocity;
  int16 target_torque;
  int32_t position_offset;
  int32_t velocit_offset;
  int16_t torque_offset;
  uint16 control_word;
  int16 mode_of_opration;
} ELMOsWrite;
#pragma pack()

uint16 ctrlWord(uint16 state_word);
int8_t elmo_set_state(ec_state sta);
int8_t ec_elmo_init(char *ifname);

#endif
