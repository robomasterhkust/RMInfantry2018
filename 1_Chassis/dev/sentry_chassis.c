/*
 * sentry_chassis.c
 *
 *  Created on: 2 Apr 2018
 *      Author: Alex Wong
 */

#include <canBusProcess.h>
#include "sentry_chassis.h"
#include "canBusProcess.h"
#include "ch.h"
#include "hal.h"
#include "judge.h"
#include "dbus.h"

static volatile senchassisstruct senchassis;

static uint8_t abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX) {
	  *a = ABS_MAX;
	  return 1;
  }

  if (*a < -ABS_MAX){
	  *a = -ABS_MAX;
	  return 1;
  }

  return 0;

}


/*
 * Sentry chassis pos pid with anti integral windup
 */
static float senchassis_pos_pid(pid_profile_t* setting, sen_pid_controller_t* data, float SP, float PV) {

	  uint8_t maxed = 0;
	  data->pv = PV;
	  data->sp = SP;
	  data->error[0] = SP - PV;

	  data->pout = setting->kp * data->error[0];
	  data->dout = setting->kd * (data->error[0] - data->error[1]);

	  data->cv = data->pout + data->dout + (setting->ki * (data->error[0] + data->iout));
	  maxed = abs_limit(&data->cv, setting->cv_limit);

	  data->iout = maxed ? 0 : data->iout + data->error[0];
	  abs_limit(&data->iout, setting->integral_limit);

	  data->error[1] = data->error[0];

	  return data->cv;

}

float SP = 0;


static judge_fb_t* JudgeData;
static GameData_tx txCan;
static RC_Ctl_t* remote;

static inline void chassisdata_txCan(CANDriver *const CANx, const uint16_t SID)
{
  CANTxFrame txmsg;


  txmsg.IDE = CAN_IDE_STD;
  txmsg.SID = SID;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0x08;

  chSysLock();
  txCan.chassis_pos = (int16_t) SP;
  txCan.shooter_heat = (uint16_t) JudgeData->powerInfo.shooterHeat0;
  txCan.shooter_speed = (float) JudgeData->projectileInfo.bulletSpeed;


  memcpy(&(txmsg.data8), &txCan ,8);
  chSysUnlock();

  canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

static THD_WORKING_AREA(senchassis_control_wa, 2048);
static THD_FUNCTION(senchassis_control, p) {

	(void)p;
	uint8_t i = 0;
	int32_t drift = 0;
	int8_t drift_dir = 1;
	int32_t drift_count = 0;
	int32_t drift_count_limit = 4000;
	float drift_amount = 0.5;

	static Gimbal_Send_Dbus_canStruct* RC_cntrl;

	RC_cntrl = can_get_sent_dbus();

	JudgeData = judgeDataGet();

	while (!chThdShouldTerminateX()) {

//		if ((palReadPad(GPIOC,0) && palReadPad(GPIOC,1))){
//			SP += (RC_cntrl->channel0 - STICK_NEUTURAL) * STICK_GAIN;
//
//		} else {
//			SP += 0;
//		}

		if (!(palReadPad(GPIOC,0) && palReadPad(GPIOC,1))) {
			SP += 0;
		} else {
			if (remote->rc.s1 != 0) {
				SP += (RC_cntrl->channel0 - STICK_NEUTURAL) * STICK_GAIN;
			} else {
				SP += drift_amount * drift_dir;
				drift_count++;
				if (drift_count == drift_count_limit) {
					drift_dir = drift_dir * -1;
					drift_count = 0;
				}

			}
		}

		for (i = 0; i < SEN_CHASSIS_MOTOR_NUM; i++) {
			senchassis._motors[i].pos = senchassis.encoders[i].total_ecd;
			senchassis._motors[i].pos_sp = SP / CNT_2_DIS;
			senchassis_pos_pid(&senchassis.pospidprofile, &senchassis._motors[i].pidcontroller,
							   (senchassis._motors[i].inverted ? -senchassis._motors[i].pos : senchassis._motors[i].pos),
							   senchassis._motors[i].pos_sp);
		}
		can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, senchassis._motors[0].pidcontroller.cv,
														  -senchassis._motors[1].pidcontroller.cv, 0, 0);


		chassisdata_txCan(&CAND1, CAN_GIMBAL_TX_GAMEDATA_ID);

		chThdSleep(MS2ST(1));
	}


}

volatile senchassisstruct* get_chassis(void) {

	return &senchassis;

}

void sen_chassis_init (void) {

	memset(&senchassis, 0, sizeof(senchassisstruct));

	remote = RC_get();

	senchassis.encoders = can_getExtraMotor();
	SP = 0;

	senchassis._motors[0].inverted = 1;

	senchassis.pospidprofile.kp = POS_KP;
	senchassis.pospidprofile.ki = POS_KI;
	senchassis.pospidprofile.kd = POS_KD;
	senchassis.pospidprofile.cv_limit = POS_OUTLIM;
	senchassis.pospidprofile.integral_limit = POS_ILIM;

	chThdCreateStatic(senchassis_control_wa, sizeof(senchassis_control_wa),
					  HIGHPRIO, senchassis_control, NULL);

}

