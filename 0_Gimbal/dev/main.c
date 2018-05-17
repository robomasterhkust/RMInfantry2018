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

#define MPU6500_UPDATE_PERIOD_US 1000000U/MPU6500_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_thread_wa, 4096);
static THD_FUNCTION(Attitude_thread, p)
{
  chRegSetThreadName("IMU Attitude Estimator");

  (void)p;

  PIMUStruct pIMU = imu_get();
  PGyroStruct pGyro = gyro_get();

  static const IMUConfigStruct imu1_conf =
    {&SPID5, MPU6500_ACCEL_SCALE_8G, MPU6500_GYRO_SCALE_1000, MPU6500_AXIS_REV_X};
  imuInit(pIMU, &imu1_conf);

  //static const magConfigStruct mag1_conf =
  //  {IST8310_ADDR_FLOATING, 200, IST8310_AXIS_REV_NO};
  //ist8310_init(&mag1_conf);

  //Check temperature feedback before starting temp controller
  imuGetData(pIMU);
  if(pIMU->temperature > 0.0f)
    tempControllerInit();
  else
    pIMU->errorCode |= IMU_TEMP_ERROR;

/*
  while(pIMU->temperature < 61.0f)
  {
    imuGetData(pIMU);
    chThdSleepMilliseconds(50);
  }

  pIMU->state = IMU_STATE_READY;*/
  attitude_imu_init(pIMU);

  uint32_t tick = chVTGetSystemTimeX();

  while(true)
  {
    tick += US2ST(MPU6500_UPDATE_PERIOD_US);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      pIMU->errorCode |= IMU_LOSE_FRAME;
    }

    if(pIMU->state == IMU_STATE_HEATING && pIMU->temperature > 61.0f)
      pIMU->state = IMU_STATE_READY;
    else if(pIMU->temperature < 55.0f || pIMU->temperature > 70.0f)
      pIMU->errorCode |= IMU_TEMP_WARNING;

    imuGetData(pIMU);
    //ist8310_update();
    attitude_update(pIMU, pGyro);

    if(pIMU->accelerometer_not_calibrated || pIMU->gyroscope_not_calibrated)
    {
      chSysLock();
      chThdSuspendS(&(pIMU->imu_Thd));
      chSysUnlock();
    }
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

  /* Init sequence 2: sensors, comm*/
  attitude_init();
  gyro_init();
  can_processInit();
  RC_init();
  mavlinkComm_init();
  barrelHeatLimitControl_init();

  while(!power_check())
  {
    LEDY_TOGGLE();
    chThdSleepMilliseconds(200);
  }

  /* Init sequence 3: actuators, display*/
  gimbal_init();
  shooter_init();
  feederInit();
  //rune_init();

  wdgStart(&WDGD1, &wdgcfg); //Start the watchdog

  while (true)
  {
    uint32_t error = gimbal_get_error();

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

/**
  *   @brief Check whether the 24V power is on
  */
bool power_check(void)
{
  GimbalEncoder_canStruct* can = can_getGimbalMotor();

  return can->updated;
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
