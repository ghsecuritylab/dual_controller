#pragma once

#include "ch.h"
#include "hal.h"

#define NUM_ICU_CAPTURES 3

typedef struct {
  ioline_t a, b, c;
  ICUDriver* icu;
  uint32_t icu_freq;
  uint32_t pwm_freq;
  
  float speed;
  
  int last_segment;
  int direction;
  
  int icu_capture[NUM_ICU_CAPTURES];
  int num_valid_icu_captures;
} hall_sensors_t;

typedef enum {
  HALL_SENSOR_STATE_LOW  = 0b000,
  HALL_SENSOR_STATE_0    = 0b001,
  HALL_SENSOR_STATE_60   = 0b101,
  HALL_SENSOR_STATE_120  = 0b100,
  HALL_SENSOR_STATE_180  = 0b110,
  HALL_SENSOR_STATE_240  = 0b010,
  HALL_SENSOR_STATE_300  = 0b011,
  HALL_SENSOR_STATE_HIGH = 0b111
} hall_sensors_state_e;

void HallSensorsInit(hall_sensors_t*);
hall_sensors_state_e HallSensorsGetState(hall_sensors_t*);

int HallSensorsStateToSegment(hall_sensors_state_e);
float HallSensorsStateToAngle(hall_sensors_state_e);

