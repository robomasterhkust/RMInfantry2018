/**
 * Edward ZHANG
 * @file    shellcfg.c
 * @brief   definitions of shell command functions
 */
#include "main.h"
#include "shell.h"
#include <string.h>

#define SERIAL_CMD       &SDU1
#define SERIAL_DATA      &SDU1

static thread_t* matlab_thread_handler = NULL;
/**
 * @brief Transmit uint32_t and float through serial port to host machine
 * @require Initialization of ChibiOS serial driver before using this function
 *
 * @param[in] chp         pointer to a @p BaseSequentialStream implementing object
 * @param[in] txbuf_d     array of 32-bit integers to tramsmit, can be signed or unsigned
 * @param[in] txbuf_f     array of float point numbers to tramsmit
 * @param[in] num_int     number of 32-bit integers to tramsmit
 * @param[in] num_float   number of float point numbers to tramsmit
 *
 * @TODO improve the transmission protocol to enable easier setup for the host machine
 */
#define SYNC_SEQ  0xaabbccdd
static void transmit_matlab
  (BaseSequentialStream* chp,
    uint32_t* const txbuf_d, float* const txbuf_f,
    const uint8_t num_int, const uint8_t num_float)
{
  uint32_t sync = SYNC_SEQ;
  char* byte = (char*)&sync;

  uint8_t i;
  for (i = 0; i < 4; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_d;
  for (i = 0; i < 4*num_int; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_f;
  for (i = 0; i < 4*num_float; i++)
    chSequentialStreamPut(chp, *byte++);
}

#define HOST_TRANSMIT_FREQ  100U
static THD_WORKING_AREA(matlab_thread_wa, 512);
static THD_FUNCTION(matlab_thread, p)
{
  (void)p;
  chRegSetThreadName("matlab tramsmitter");

  int32_t txbuf_d[16];
  float txbuf_f[16];
  BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_DATA;

  //================Initialization of transmission data ==========================//
  PIMUStruct pIMU = adis16470_get();
  float timestamp_prev =  pIMU->stamp / 2000.0f;
  //==============================================================================//

  uint32_t tick = chVTGetSystemTimeX();
  const uint16_t period = US2ST(1000000/HOST_TRANSMIT_FREQ);
  while (!chThdShouldTerminateX())
  {
    tick += period;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    //==============================Set the data to transmit=========================//
    txbuf_f[0] = pIMU->euler_angle[Roll];
    txbuf_f[1] = pIMU->euler_angle[Pitch];
    txbuf_f[2] = pIMU->euler_angle[Yaw];

    //timestamp_prev =  pIMU->stamp / 2000.0f;
    //===============================================================================//

    transmit_matlab(chp, NULL, txbuf_f, 0, 3);
  }
}

/*===========================================================================*/
/* Definitions of shell command functions                                    */
/*===========================================================================*/
static THD_WORKING_AREA(Shell_thread_wa, 1024);
void cmd_test(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
  PIMUStruct pIMU = adis16470_get();

  chprintf(chp, "Roll: %f\r\n", pIMU->euler_angle[X]);
  chprintf(chp, "Pitch: %f\r\n", pIMU->euler_angle[Y]);
  chprintf(chp, "Yaw: %f\r\n", pIMU->euler_angle[Z]);

  chprintf(chp, "AccelX: %f\r\n", pIMU->accelData[X]);
  chprintf(chp, "AccelY: %f\r\n", pIMU->accelData[Y]);
  chprintf(chp, "AccelZ: %f\r\n", pIMU->accelData[Z]);

}

void cmd_error(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint32_t error;

  //Initialization error
  error = init_state_get();
  if(error & INIT_SEQUENCE_3_RETURN_1)
    chprintf(chp,"E:INIT SEQ 3 FAILED -- GIMBAL YAW NOT CONNECTED\r\n");
  if(error & INIT_SEQUENCE_3_RETURN_2)
    chprintf(chp,"E:INIT SEQ 3 FAILED -- GIMBAL PITCH NOT CONNECTED\r\n");

  //Gimbal error
  error = gimbal_get_error();
  if(error & GIMBAL_YAW_NOT_CONNECTED)
    chprintf(chp,"E:GIMBAL YAW CONNECTION LOST\r\n");
  if(error & GIMBAL_PITCH_NOT_CONNECTED)
    chprintf(chp,"E:GIMBAL PITCH CONNECTION LOST\r\n");
  if(error & GIMBAL_CONTROL_LOSE_FRAME)
    chprintf(chp,"W:Gimbal control lose frame\r\n");

  //IMU error
  error = adis16470_get_error();
  PIMUStruct pIMU = adis16470_get();
  if(error & ADIS16470_AXIS_CONF_ERROR)
    chprintf(chp,"E:IMU COORDINATE CONFIGURATION ERROR\r\n");
  if(error & ADIS16470_SENSOR_ERROR)
    chprintf(chp,"E:ADIS16470 SENSOR ERROR: %X\r\n", pIMU->diag_stat);
  if(error & ADIS16470_READING_ERROR)
    chprintf(chp,"E:ADIS16470 READING ERROR\r\n");
  if(error & ADIS16470_UNCONNECTED)
    chprintf(chp,"E:ADIS16470 SENSOR UNCONNECTED\r\n");
  if(error & ADIS16470_ACCEL_NOT_CALIBRATED)
    chprintf(chp,"W:ADIS16470 Accelerometer not calibrated\r\n");
  if(error & ADIS16470_GYRO_NOT_CALIBRATED)
    chprintf(chp,"W:ADIS16470 Gyroscope not calibrated\r\n");
  if(error & ADIS16470_DATA_INVALID)
    chprintf(chp,"W:ADIS16470 has invalid reading\r\n");
  if(error & ADIS16470_LOSE_FRAME)
    chprintf(chp,"W:Attitude estimator lose frame\r\n");
  adis16470_clear_error();

  //feeder error
  error = feeder_get_error();
  if(error & FEEDER_CONNECTION_ERROR)
    chprintf(chp, "E: FEEDER MOTOR NOT CONNECTED\r\n");
  if(error & LIMIT_SWITCH_ERROR_0)
    chprintf(chp, "E: LIMIT SWITCH ERROR_0, MOUNTING\r\n");
  if(error & LIMIT_SWITCH_ERROR_1)
    chprintf(chp, "E: LIMIT SWITCH ERROR_1, CONNECTION\r\n");
  if(error & LIMIT_SWITCH_ERROR_2)
    chprintf(chp, "E: LIMIT SWITCH ERROR_2, CONNECTION\r\n");

  system_clearWarningFlag();
}

void cmd_rune(BaseSequentialStream * chp, int argc, char *argv[])
{
  rune_cmd(ENABLE);
  rune_fire(0.0f, 0.0f);
  chThdSleepSeconds(1);
  rune_cmd(DISABLE);
}

#ifdef SHOOTER_SETUP
  /**
   * @brief Start the data tramsmission to matlab
   * @note caution of data flooding to the serial port
   */
  void cmd_shooter_setup(BaseSequentialStream * chp, int argc, char *argv[])
  {
    if(argc)
    {
      if(!strcmp(argv[0], "set1"))
      {
        pwm9_setWidth(900);
        chprintf(chp,"Set pwm to max\r\n");
      }
      else if(!strcmp(argv[0], "set2"))
      {
        pwm9_setWidth(100);
        chprintf(chp,"Set pwm to min\r\n");
      }
    }
    else
      chprintf(chp,"cmd: set1, set2\r\n");
  }
#endif

/**
 * @brief Start the data tramsmission to matlab
 * @note caution of data flooding to the serial port
 */
void cmd_data(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t sec = 10;

  if(argc && matlab_thread_handler == NULL)
  {
    char *toNumber = argv[0];
    uint32_t finalNum=0;
    while(*toNumber>='0' && *toNumber<='9')
      finalNum=finalNum*10+*(toNumber++)-'0';

    if(finalNum == 0)
      finalNum = 10;

    sec = (finalNum < 60 ? finalNum : 60);

    chprintf(chp,"Data transmission start in %d seconds...\r\n", sec);
    chThdSleepSeconds(sec);

    matlab_thread_handler = chThdCreateStatic(matlab_thread_wa, sizeof(matlab_thread_wa),
        NORMALPRIO + 3,
        matlab_thread, NULL);
  }
  else if(matlab_thread_handler != NULL)
  {
    chThdTerminate(matlab_thread_handler);
    matlab_thread_handler = NULL;
  }
}

void cmd_calibrate(BaseSequentialStream * chp, int argc, char *argv[])
{
  PIMUStruct pIMU = adis16470_get();

  int32_t accelBias[3], gyroBias[3];

  if(pIMU->state == ADIS16470_READY)
  {
    pIMU->state = ADIS16470_CALIBRATING;
    chThdSleepMilliseconds(10);

    adis16470_get_gyro_bias(gyroBias);
    adis16470_get_accel_bias(accelBias);
  }
  else
  {
    chprintf(chp, "IMU initialization not complete \\ Error occured\r\n");
    return;
  }

  if(argc)
  {
    gimbal_kill();
    chprintf(chp, "Calibration in process...\r\n");
    chThdSleepSeconds(2);

    if(!strcmp(argv[0], "accl"))
    {
      adis16470_reset_calibration();
      calibrate_accelerometer(pIMU, accelBias);

      chprintf(chp, "Saving to ADIS16470 flash...\r\n");
      //adis16470_set_calibration_id(pIMU->calibration_id, 0);
      adis16470_bias_update(accelBias, gyroBias);
    }
    else if(!strcmp(argv[0], "gyro"))
    {
      adis16470_reset_calibration();

      calibrate_gyroscope(pIMU, gyroBias);

      chprintf(chp, "Saving to ADIS16470 flash...\r\n");
      //adis16470_set_calibration_id(0, pIMU->calibration_id);
      adis16470_bias_update(accelBias, gyroBias);
    }
    else if(!strcmp(argv[0], "res"))
    {
      chprintf(chp, "Restoring factory calibration\r\n");
      adis16470_reset_calibration();
      chprintf(chp, "Saving to ADIS16470 flash...\r\n");
      adis16470_bias_update(accelBias, gyroBias);
    }
  }
  else
    chprintf(chp,"Calibration: gyro, accl\r\n");

  pIMU->state = ADIS16470_READY;
  chThdResume(&(pIMU->imu_Thd), MSG_OK);
  pIMU->imu_Thd = NULL;
}

/**
 * @brief array of shell commands, put the corresponding command and functions below
 * {"command", callback_function}
 */
static const ShellCommand commands[] =
{
  {"test", cmd_test},
  {"WTF", cmd_error},
  {"cal", cmd_calibrate},
  {"rune", cmd_rune},
  {"\xEE", cmd_data},
  #ifdef PARAMS_USE_USB
    {"\xFD",cmd_param_scale},
    {"\xFB",cmd_param_update},
    {"\xFA",cmd_param_tx},
    {"\xF9",cmd_param_rx},
  #endif
  #ifdef SHOOTER_SETUP
    {"shoot", cmd_shooter_setup},
  #endif
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream *)SERIAL_CMD,
  commands
};

/**
 * @brief start the shell service
 * @require enable the corresponding serial ports in mcuconf.h and board.h
 *          Select the SERIAL_CMD port in main.h
 *
 * @api
 */
void shellStart(void)
{
  //sdStart(SERIAL_CMD, NULL);
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */


  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);

  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  shellInit();

  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);

}
