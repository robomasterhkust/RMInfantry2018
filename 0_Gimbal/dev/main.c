/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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
#include "main.h"

static BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;
static system_init_state_t init_state;

#define ATTITUDE_UPDATE_PERIOD_US 1000000U/ATTITUDE_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_thread_wa, 4096);
static THD_FUNCTION(Attitude_thread, p)
{
  chRegSetThreadName("IMU Attitude Estimator");

  (void)p;

  PIMUStruct pIMU = adis16470_get();
  adis16265_conf_t imu_conf = {0x01, ADIS16470_X_REV,
                                     ADIS16470_Y,
                                     ADIS16470_Z_REV};
  adis16470_init(&imu_conf);
  while(pIMU->state != ADIS16470_READY)
    chThdSleepMilliseconds(100);

  //Check temperature feedback before starting temp controller
  attitude_imu_init(pIMU);
  init_state = INIT_ATTITUDE_COMPLETE;

  uint32_t tick = chVTGetSystemTimeX();

  while(true)
  {
    tick += US2ST(ATTITUDE_UPDATE_PERIOD_US);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      system_setTempWarningFlag();
      pIMU->error |= ADIS16470_LOSE_FRAME;
    }

    if(pIMU->state == ADIS16470_READY)
      attitude_update(pIMU);

    //attitude_update_timestamp(pIMU->stamp); //Update timestamp anyway
  }
}

#define attitude_init() (chThdCreateStatic(Attitude_thread_wa, sizeof(Attitude_thread_wa), \
                          NORMALPRIO + 5, \
                          Attitude_thread, NULL))

/*
 * Watchdog deadline set to more than one second (LSI=40000 / (64 * 1000)).
 */
static const WDGConfig wdgcfg =
{
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(1000)
};

/*
 * Application entry point.
 */
int main(void)
{

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Init sequence 1: central controllers, loggers*/
  shellStart();
  params_init();

  //sdlog_init();
  extiinit();
  system_error_init();
  LASER_ON();

  /* Init sequence 2: sensors, comm, actuators, display*/
  can_processInit();
  barrelHeatLimitControl_init();
  RC_init();

  gimbal_init();
  feeder_init();

  /*
   * Init sequence 3: start all actuator controllers,
   * NOTE: ONLY after verifying the presence of 24V power
   */

  do
  {
    init_state = power_check();
    if(init_state)
      system_setTempErrorFlag();

    chThdSleepMilliseconds(100);

  } while(init_state & (INIT_SEQUENCE_3_RETURN_1 | INIT_SEQUENCE_3_RETURN_2));

  attitude_init();

  while (init_state != INIT_ATTITUDE_COMPLETE)
    chThdSleepMilliseconds(100);//Wait for ADIS16470 Initialization

  gimbal_start();
//  feeder_start();
//  shooter_start();
  rune_init();

  init_state = INIT_COMPLETE;
  wdgStart(&WDGD1, &wdgcfg); //Start the watchdog

  while (true)
  {
    if(!power_failure())
    {
      wdgReset(&WDGD1);
    }
    else
      gimbal_kill();

    chThdSleepMilliseconds(200);
  }

  return 0;
}

system_init_state_t init_state_get(void)
{
  return init_state;
}

/**
  *   @brief Check whether the 24V power is on
  */
uint8_t power_check(void)
{
  GimbalEncoder_canStruct* can = can_getGimbalMotor();

  system_init_state_t result = INIT_DUMMY;
  if(!(can[0].updated))
    result |= INIT_SEQUENCE_3_RETURN_1;
  if(!(can[1].updated))
    result |= INIT_SEQUENCE_3_RETURN_2;

  return result;
  //return INIT_DUMMY;
}

/**
  *   @brief  Monitor the case of a failure on 24V power, indicating the vehicle being killed
  */
bool power_failure(void)
{
  uint32_t error = gimbal_get_error();

  return error & (GIMBAL_PITCH_NOT_CONNECTED | GIMBAL_YAW_NOT_CONNECTED) ==
    (GIMBAL_PITCH_NOT_CONNECTED | GIMBAL_YAW_NOT_CONNECTED);
}
