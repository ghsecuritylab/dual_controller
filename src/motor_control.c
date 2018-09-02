#include "motor_control.h"

#include "ch.h"
#include "hal.h"
#include "biquad.h"
#include <math.h>
#include <string.h>

#include "freq_response.h"
#include "lwip/udp.h"

static const float max_duty = 0.95f;
static const float adc_sample_duty = 0.97f;

static const float inv_sqrt3 = 0.5773502692f;
static const float sqrt3 = 1.7320508076f;
#define MIN(x,y) ((x)<(y)?(x):(y))

static alphabeta_t clarke(float a, float b) {
  const alphabeta_t ret =  { .alpha = a,
                             .beta = inv_sqrt3 * (a + b + b) };
  return ret;
}

static dq_t park(alphabeta_t ab, float sin_theta, float cos_theta) {
  const dq_t dq = { .d = ab.alpha * cos_theta + ab.beta * sin_theta,
                    .q = ab.beta * cos_theta - ab.alpha * sin_theta };
  return dq;
}

static alphabeta_t inv_park(dq_t dq, float sin_theta, float cos_theta) {
  const alphabeta_t ab = { .alpha = dq.d * cos_theta - dq.q * sin_theta,
                           .beta  = dq.q * cos_theta + dq.d * sin_theta };
  
  return ab;
}

static struct udp_pcb* freq_pcb;
static struct pbuf* freq_pb;

static freq_response_t iq_freq = { .fs = 35e3, .f_min = 10.f, .f_max = 35e3/2, .frequencies_per_decade = 20.f, .wavelengths_per_frequency = 20, .magnitude = 1.f };

void fr_emit(freq_response_t* fr, const freq_response_result_t* result) {
  (void)fr;
  
  memcpy(freq_pb->payload, result, sizeof(freq_response_result_t));
  freq_pb->len = freq_pb->tot_len = sizeof(freq_response_result_t);
  udp_sendto(freq_pcb, freq_pb, IP_ADDR_BROADCAST, 10100);
}

THD_WORKING_AREA(mcTask, 2048);
static THD_FUNCTION(_MCTask, arg) {
  (void)arg;

  chThdSleep(TIME_MS2I(2200));
  
  freq_pcb = udp_new();
  udp_bind(freq_pcb, IP_ADDR_ANY, 10100);
  ip_set_option(freq_pcb, SOF_BROADCAST);
  freq_pb = pbuf_alloc(PBUF_TRANSPORT, 512, PBUF_RAM);
  
  iq_freq.emit = fr_emit;
  
  while(true) {
    FreqResponse_Poll(&iq_freq);
    chThdYield();
  }
}


void MotorControlInit(motor_control_t *m, pwmcallback_t cb) {
  
  // Configure biquads
  // Poles at 0.4 and 1.0
  // Zeros at 0.93 and -1.0
  // Gain 26 at 1kHz (35kHz switching frequency)
  BiquadInit(&m->iq_compensator, 7.6605727f, 0.53624009f, -7.1243326f, -1.4f, 0.4f);
  BiquadInit(&m->id_compensator, 7.6605727f, 0.53624009f, -7.1243326f, -1.4f, 0.4f);
  
  // Set up a task to configure and poll the loop gain measurement
  chThdCreateStatic(mcTask, sizeof(mcTask), LOWPRIO, _MCTask, NULL);
  
  // Initialize the hall sensors
  HallSensorsInit(m->hall_sensors);
  TachInit(&m->tach, HallSensorsGetState(m->hall_sensors));
  
  // Initialize config
  m->config.frequency = m->clock_freq;
  m->config.period = (m->clock_freq / m->pwm_freq);
  m->config.callback = cb;
  m->config.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;
  m->config.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH;
  m->config.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;
  m->config.channels[3].mode = PWM_OUTPUT_ACTIVE_HIGH;
  
  m->max_duty_count = m->config.period * max_duty;
  
  // Activate the timer
  pwmStart(m->driver, (void*)&m->config);
  pwmEnableChannel(m->driver, 3, m->config.period * adc_sample_duty);
  pwmEnablePeriodicNotification(m->driver);
  
}

void MotorControlCb(motor_control_t* m,
                    float i_a,
                    float i_b) {
  // Confirm that we've received a new ADC value since last cycle
  if(!(ADC1->SR & ADC_SR_JEOC))
    m->error_count++;
  ADC1->SR = 0;
  
  // const float theta = 0.f;
  hall_sensors_state_e hall_state = HallSensorsGetState(m->hall_sensors);
  TachUpdate(&m->tach, hall_state);
  const float theta = HallSensorsStateToAngle(hall_state);
  const float sin_theta = sinf(theta);
  const float cos_theta = cosf(theta);
  
  const alphabeta_t i_ab = clarke(i_a, i_b);
  const dq_t i_dq = park(i_ab, sin_theta, cos_theta);
  
  const float id_error = 0.f - i_dq.d;
  const float iq_error = m->i_target - i_dq.q;
  // const float iq_error = m->i_target - FreqResponse_Update(&iq_freq, i_dq.q);
  
  if(m->enabled) {
    // Iterate the compensator
    const dq_t vdq = { .d = BiquadUpdateLim(&m->id_compensator, id_error, -m->max_duty_count, m->max_duty_count),
                       .q = BiquadUpdateLim(&m->iq_compensator, iq_error, -m->max_duty_count, m->max_duty_count) };
    
    // Transform back to abc space
    const alphabeta_t vab = inv_park(vdq, sin_theta, cos_theta);
    const int16_t va = vab.alpha;
    const int16_t vb = (-vab.alpha + sqrt3 * vab.beta) * 0.5f;
    const int16_t vc = (-vab.alpha - sqrt3 * vab.beta) * 0.5f;
    const int16_t min_v = MIN(MIN(va, vb), vc);
    
    // Configure PWM channel
    m->driver->tim->CCR[0] = va - min_v;
    m->driver->tim->CCR[1] = vb - min_v;
    m->driver->tim->CCR[2] = vc - min_v;
    
    // Enable PWM driver
    palWriteLine(m->fault_clear, PAL_HIGH);
  } else {
    // Disable PWM driver
    palWriteLine(m->fault_clear, PAL_LOW);
    
    // Configure PWM channels
    m->driver->tim->CCR[0] = 0;
    m->driver->tim->CCR[1] = 0;
    m->driver->tim->CCR[2] = 0;
    
    // Reset the compensators
    BiquadReset(&m->id_compensator);
    BiquadReset(&m->iq_compensator);
  }
  
  m->i_a = i_a;
  m->i_b = i_b;
  
  m->i_d = i_dq.d;
  m->i_q = i_dq.q;
  
  m->theta = theta;
  m->sin_theta = sin_theta;
  m->cos_theta = cos_theta;
  
  m->faulted = !palReadLine(m->nfault);
}
