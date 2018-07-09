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

static bool rune_state;
// static PWMDriver PWMD9;

void pwm9_setWidth(uint16_t width){
  pwmEnableChannelI(&PWMD9, 0, width);
  pwmEnableChannelI(&PWMD9, 1, width);
}

/**
 * 2017/12/17 PWM test
 * @return
 */
static void shooter_control(uint16_t setpoint)
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

      if(rune_state)
        shooter_control(speed_mode.rune_speed); //Use the highest possible speed to shoot the rune
      else
        shooter_control(shooting_speed);

      speed = alpha * (float)speed_sp + (1-alpha) * speed;
      pwm9_setWidth((uint16_t)speed);
      chThdSleepMilliseconds(5);
    }
}

static void pwm9_start(void){
   pwmStart(&PWMD9, &pwm9cfg);
}

void shooter_setRuneState(const uint8_t enable)
{
  rune_state = (enable == DISABLE ? false : true );
}

void shooter_start(void)
{
    rc = RC_get();
    pwm9_start();
    speed_mode.rune_speed = 160;
    speed_mode.fast_speed = 160;
    speed_mode.slow_speed = 110;
    speed_mode.stop = 100;
    #ifndef SHOOTER_SETUP
      pwm9_setWidth(900);
      chThdSleepSeconds(3);

      pwm9_setWidth(100);
      chThdSleepSeconds(3);

      // pwm9_setWidth(110);
      chThdCreateStatic(pwm_thd_wa, sizeof(pwm_thd_wa), NORMALPRIO + 1, pwm_thd, NULL);
    #endif
}
