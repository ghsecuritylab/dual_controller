#include "hall_sensors.h"
#include "math.h"

void HallSensorsInit(hall_sensors_t* hs) {
  ICUConfig config = {
    .mode = ICU_INPUT_ACTIVE_LOW,
    .frequency = hs->icu_freq,
    .channel = ICU_CHANNEL_1
  };
  
  icuStart(hs->icu, &config);
  
  // Configure timer to xor hall sensor inputs
  hs->icu->tim->CR2 |= STM32_TIM_CR2_TI1S;
  
  icuStartCapture(hs->icu);
}

static void _UpdateSpeedMeasurement(hall_sensors_t* hs) {
  if(hs->num_valid_icu_captures >= NUM_ICU_CAPTURES) {
    int sum_periods = 0, i;
    for(i = 0; i < NUM_ICU_CAPTURES; i++) {
      sum_periods += hs->icu_capture[i];
    }
    
    float speed = hs->icu_freq / (float)sum_periods;
    hs->speed = speed;
  } else {
    hs->speed = 0.f;
  }
}

static bool _PollICU(hall_sensors_t* hs, int* capture) {
  bool new_capture = false;
  
  uint32_t sr = hs->icu->tim->SR;
  if(sr & (STM32_TIM_SR_CC1IF | STM32_TIM_SR_UIF)) {
    if(sr & STM32_TIM_SR_UIF) {
      hs->num_valid_icu_captures = 0;
      hs->speed = 0.f;
    }
    else {
      // Use the speed measurement
      new_capture = true;
      *capture = hs->icu->tim->CCR[1];
    }
    
    if(sr & STM32_TIM_SR_CC1IF) {
      // Reset flags
      hs->icu->tim->SR &= ~ (STM32_TIM_SR_CC1IF | STM32_TIM_SR_UIF);
    }
  }
  
  return new_capture;
}

hall_sensors_state_e HallSensorsGetState(hall_sensors_t* hs) {
  // Poll for an updated speed measurement
  int capture = 0;
  bool new_capture = _PollICU(hs, &capture);
  
  // Measure state
  hall_sensors_state_e state = (palReadLine(hs->a) ? 0b001 : 0b000) |
                               (palReadLine(hs->b) ? 0b010 : 0b000) |
                               (palReadLine(hs->c) ? 0b100 : 0b000);
  
  int segment = HallSensorsStateToSegment(state);
  int segment_jump = (segment - hs->last_segment);
  if(segment_jump > 2) segment_jump -= 6;
  else if(segment_jump < -2) segment_jump += 6;
  
  int direction = (segment_jump > 0) ? 1 : -1;
  
  if(new_capture) {
    if(direction == hs->direction) {
      hs->icu_capture[segment % NUM_ICU_CAPTURES] = capture;
      if(hs->num_valid_icu_captures < NUM_ICU_CAPTURES)
        hs->num_valid_icu_captures++;
    } else {
      hs->num_valid_icu_captures = 0;
    }
    
    _UpdateSpeedMeasurement(hs);
    
    hs->last_segment = segment;
    hs->direction = direction;
  }
  
  
  return state;
}

int HallSensorsStateToSegment(hall_sensors_state_e state) {
  switch(state) {
    case HALL_SENSOR_STATE_60:
      return 1;
    case HALL_SENSOR_STATE_120:
      return 2;
    case HALL_SENSOR_STATE_180:
      return 3;
    case HALL_SENSOR_STATE_240:
      return 4;
    case HALL_SENSOR_STATE_300:
      return 5;
    case HALL_SENSOR_STATE_LOW:
    case HALL_SENSOR_STATE_HIGH:
    case HALL_SENSOR_STATE_0:
    default:
      return 0;
  }
}

float HallSensorsStateToAngle(hall_sensors_state_e state) {
  return HallSensorsStateToSegment(state) * (float)(M_PI / 3.f);
}
