/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"

#include "lwipthread.h"

#include "comms.h"
#include "hall_sensors.h"
#include "motor_control.h"
#include "93AA46AE48.h"

#include "util.h"
#include <string.h>

#define PWM_FREQ 35e3

static hall_sensors_t h1 = { .a = LINE_TIM4_CH3,
                             .b = LINE_TIM4_CH2,
                             .c = LINE_TIM4_CH1,
                             .icu = &ICUD4,
                             .icu_freq = STM32_TIMCLK1 / 100,
                             .pwm_freq = PWM_FREQ };

static motor_control_t m1 = { .driver = &PWMD1,
                              .clock_freq = STM32_TIMCLK2,
                              .pwm_freq = PWM_FREQ,
                              .nfault = LINE_TIM1_BKIN1,
                              .fault_clear = LINE_GPIOG_PIN1,
                              .bridge_enabled = LINE_GPIOG_PIN0,
                              .hall_sensors = &h1
                             };
static motor_control_t m2 = { .driver = &PWMD8,
                              .clock_freq = STM32_TIMCLK2,
                              .pwm_freq = PWM_FREQ,
                              .nfault = LINE_TIM8_BKIN1,
                              .fault_clear = LINE_GPIOF_PIN12,
                              .bridge_enabled = LINE_GPIOF_PIN11
                            };

const float i_scale = (3.f / 4095.f) / 20.f / 0.001f;

static void m1_pwm_cb(PWMDriver *pwmp)  {
  (void)pwmp;
  MotorControlCb(&m1, ((int16_t)ADC1->JDR1) * i_scale,
                      ((int16_t)ADC1->JDR2) * i_scale);
}

static void m2_pwm_cb(PWMDriver *pwmp)  {
  (void)pwmp;
  MotorControlCb(&m2, ((int16_t)ADC2->JDR1) * i_scale,
                      ((int16_t)ADC2->JDR2) * i_scale);
}


static const mc_93aa46ae48_t mc_93aa46ae48 = { .cs = LINE_SPI3_CS,
                                               .sck = LINE_SPI3_SCK,
                                               .miso = LINE_SPI3_MISO,
                                               .mosi = LINE_SPI3_MOSI };
static uint8_t _macaddress[6];
static const struct lwipthread_opts _lwip_opts = { .address = IP4_ADDR_VALUE(192, 168, 1, 10),
                                                   .gateway = IP4_ADDR_VALUE(192, 168, 1, 1),
                                                   .netmask = IP4_ADDR_VALUE(255, 255, 255, 0),
                                                   .addrMode = NET_ADDRESS_STATIC,
                                                   .macaddress = _macaddress };

static void configure_motor_adcs(void) {
  rccEnableADC1(true);
  rccEnableADC2(true);
  
  /// M1
  // Configure the ADC
  ADC1->SMPR2 = (0b010 << 9) | (0b010 << 12);
  ADC1->JSQR = (0b01 << 20) | (3 << 10) | (4 << 15);
  ADC1->CR1 = ADC_CR1_SCAN;
  ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_JEXTEN_1;
  ADC1->JOFR1 = ADC1->JOFR2 = 0x7FF;
  
  // Trigger one sample capture, so the first MC interrupt has data
  ADC1->CR2 |= ADC_CR2_JSWSTART;
  
  /// M2
  // Configure the ADC
  ADC2->SMPR2 = (0b010 << 9) | (0b010 << 12);
  ADC2->JSQR = (0b01 << 20) | (8 << 10) | (9 << 15);
  ADC2->CR1 = ADC_CR1_SCAN;
  ADC2->CR2 = ADC_CR2_ADON | ADC_CR2_JEXTEN_1;
  ADC2->JOFR1 = ADC1->JOFR2 = 0x7FF;
  
  // Trigger one sample capture, so the first MC interrupt has data
  ADC2->CR2 |= ADC_CR2_JSWSTART;
}

static void handle_incoming_command(comms_t* c,
                                    enum incoming_publish_type inpub_type,
                                    cw_unpack_context* uc)
{
  (void)c;
  motor_control_t* mc = NULL;
  if(inpub_type == INPUB_CMD_M1)
    mc = &m1;
  else if(inpub_type == INPUB_CMD_M2)
    mc = &m2;
  else
    return;

  cw_unpack_next(uc);
  if (uc->item.type != CWP_ITEM_MAP)
    return;
  
  int num_items = uc->item.as.map.size;
  int i;
  for(i = 0; i < num_items; i++) {
    cw_unpack_next(uc);
    if (uc->item.type != CWP_ITEM_STR)
      return;
    
    const char* key = uc->item.as.str.start;
    
    cw_unpack_next(uc);
    if(strncmp(key, "enable", uc->item.as.str.length) == 0) {
      if(uc->item.type == CWP_ITEM_BOOLEAN)
        mc->enabled = uc->item.as.boolean;
    }
    if(strncmp(key, "i_target", uc->item.as.str.length) == 0) {
      if(uc->item.type == CWP_ITEM_FLOAT)
        mc->i_target = uc->item.as.real;
    }
  }
}

static systime_t dispatch_telemetry(comms_t* c,
                                    cw_pack_context* pc)
{
  (void)c;
  
  cw_pack_map_size(pc, 10);
  
  cw_pack_str(pc, "i_a", 3);
  cw_pack_float(pc, m1.i_a);
  cw_pack_str(pc, "i_b", 3);
  cw_pack_float(pc, m1.i_b);
  
  cw_pack_str(pc, "i_d", 3);
  cw_pack_float(pc, m1.i_d);
  cw_pack_str(pc, "i_q", 3);
  cw_pack_float(pc, m1.i_q);
  
  cw_pack_str(pc, "theta", 5);
  cw_pack_float(pc, m1.theta);
  cw_pack_str(pc, "sin_theta", 9);
  cw_pack_float(pc, m1.sin_theta);
  cw_pack_str(pc, "cos_theta", 9);
  cw_pack_float(pc, m1.cos_theta);
  
  cw_pack_str(pc, "tach", 4);
  cw_pack_signed(pc, m1.tach.counter);
  
  cw_pack_str(pc, "faulted", 7);
  cw_pack_boolean(pc, m1.faulted);
  
  cw_pack_str(pc, "speed", 5);
  cw_pack_float(pc, h1.speed);
  
  return TIME_MS2I(20);
}

static comms_t _comms = { .inpub_cb = handle_incoming_command,
                          .telem_cb = dispatch_telemetry };

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  
  // Read the external memory to get a MAC address before initializing LWIP
  Mc94AA46AE58Init(&mc_93aa46ae48);
  Mc94AA46AE58GetID(&mc_93aa46ae48, _macaddress);
  lwipInit(&_lwip_opts);

  // Set up the motor controllers
  configure_motor_adcs();
  MotorControlInit(&m1, m1_pwm_cb);
  // MotorControlInit(&m2, m2_pwm_cb);
  
  // Set up comms
  CommsInit(&_comms);
  
  // Shut down the thread
  chThdExit(0);
  
  return -1;
  // Main thread (in this case, just loop)
  // while (true) {
  //   chThdSleepMilliseconds(500);
  // }
}
