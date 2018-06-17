//
// Created by beck on 16/12/2017.
//

#include "stdint.h"
#include "ch.h"
#include "hal.h"

#include "shoot.h"
#include "dbus.h"
#include "keyboard.h"

#define MIN_SHOOT_SPEED 100U
#define MAX_SHOOT_SPEED 900U

RC_Ctl_t* rc;

static uint16_t speed_sp = 0;
static bool safe = false;
static bool Press = false;
static speed_mode_t speed_mode;
static uint8_t shooting_speed = 0;
// static PWMDriver PWMD9;

void pwm9_setWidth(uint16_t width){
  pwmEnableChannelI(&PWMD9, 0, width);
  pwmEnableChannelI(&PWMD9, 1, width);
}

// void pwm9_setWidth(uint16_t width)
// {
//   PWMD9.tim->CCR[0] = width;
//   PWMD9.tim->CCR[1] = width;
// }
// // static PWMDriver PWMD12;
// void pwm12_setWidth(uint16_t width)
// {
//   PWMD12.tim->CCR[0] = width;
//   PWMD12.tim->CCR[1] = width;
// }

/**
 * 2017/12/17 PWM test
 * @return
 */
void shooter_control(uint16_t setpoint)
{
  if(setpoint > MAX_SHOOT_SPEED)
    setpoint = MAX_SHOOT_SPEED;
  else if(setpoint < MIN_SHOOT_SPEED)
    setpoint = MIN_SHOOT_SPEED;

  if(safe || setpoint <= MIN_SHOOT_SPEED)
    speed_sp = setpoint;
}

static const PWMConfig pwm9cfg = {
        100000,   /* 1MHz PWM clock frequency.   */
        1000,      /* Initial PWM period 1ms.    width   */
        NULL,
        {
          {PWM_OUTPUT_ACTIVE_HIGH, NULL},
          {PWM_OUTPUT_ACTIVE_HIGH, NULL},
          {PWM_OUTPUT_DISABLED, NULL},
          {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

static THD_WORKING_AREA(pwm_thd_wa, 512);
static THD_FUNCTION(pwm_thd, arg) {
    (void)arg;

    const float alpha = 0.004f;
    float speed = 0;

    while (!chThdShouldTerminateX())
    {
      switch (rc->rc.s1){
        case RC_S_UP:
        {
          #ifdef SHOOTER_USE_RC
          switch (rc->rc.s2) {
            case RC_S_UP:
              shooting_speed = speed_mode.fast_speed;
             // shooter_control(speed_mode.fast_speed);
              break;
            case RC_S_MIDDLE:
              shooting_speed = speed_mode.slow_speed;
              //shooter_control(speed_mode.slow_speed);
              break;
            case RC_S_DOWN:
              shooting_speed = speed_mode.stop;
              safe = true;
              //shooter_control(speed_mode.stop);
              break;
          }
          #endif
        }break;
        case RC_S_MIDDLE:{
          if(bitmap[KEY_Z] == 1){
            Press = true;
          }
          else{
            if(Press == true){
              if(shooting_speed == speed_mode.slow_speed){
                shooting_speed = speed_mode.fast_speed;
              }
              else{
                shooting_speed = speed_mode.slow_speed;
              }
              Press = false;
            }
          }
        }break;
      }
      shooter_control(shooting_speed);
      speed = alpha * (float)speed_sp + (1-alpha) * speed;
      pwm9_setWidth((uint16_t)speed);
      chThdSleepMilliseconds(5);
    }
}

static void pwm9_start(void){
   pwmStart(&PWMD9, &pwm9cfg);
}

// static void pwm9_start(void)
// {
//   PWMD9.tim = STM32_TIM9;
//   PWMD9.channels = 2;

//   uint32_t psc;
//   uint32_t ccer;
//   rccEnableTIM9(FALSE);
//   rccResetTIM9();

//   PWMD9.clock = STM32_TIMCLK1;

//   PWMD9.tim->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | STM32_TIM_CCMR1_OC1PE |
//                      STM32_TIM_CCMR1_OC2M(6) | STM32_TIM_CCMR1_OC2PE;

//   psc = (PWMD9.clock / pwm9cfg.frequency) - 1;

//   PWMD9.tim->PSC  = psc;
//   PWMD9.tim->ARR  = pwm9cfg.period - 1;
//   PWMD9.tim->CR2  = pwm9cfg.cr2;
//   PWMD9.period = pwm9cfg.period;

//   ccer = 0;
//   switch (pwm9cfg.channels[0].mode & PWM_OUTPUT_MASK) {
//   case PWM_OUTPUT_ACTIVE_LOW:
//     ccer |= STM32_TIM_CCER_CC1P;
//   case PWM_OUTPUT_ACTIVE_HIGH:
//     ccer |= STM32_TIM_CCER_CC1E;
//   default:
//     ;
//   }
//   switch (pwm9cfg.channels[1].mode & PWM_OUTPUT_MASK) {
//   case PWM_OUTPUT_ACTIVE_LOW:
//     ccer |= STM32_TIM_CCER_CC2P;
//   case PWM_OUTPUT_ACTIVE_HIGH:
//     ccer |= STM32_TIM_CCER_CC2E;
//   default:
//     ;
//   }

//   PWMD9.tim->CCER  = ccer;
//   PWMD9.tim->SR    = 0;

//   PWMD9.tim->CR1   = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;

//   PWMD9.state = PWM_READY;
// }

void shooter_start(void)
{
    rc = RC_get();
    pwm9_start();
    speed_mode.fast_speed = 175;
    speed_mode.slow_speed = 110;
    speed_mode.stop = 100;
    #ifndef SHOOTER_SETUP
      pwm9_setWidth(900);
      chThdSleepMilliseconds(7350);

      pwm9_setWidth(100);
      chThdSleepSeconds(3);

      chThdCreateStatic(pwm_thd_wa, sizeof(pwm_thd_wa), NORMALPRIO + 1, pwm_thd, NULL);
    #endif
}
