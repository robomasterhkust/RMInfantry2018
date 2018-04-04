/*
 * senloader.c
 *
 *  Created on: 5 Apr 2018
 *      Author: Alex Wong
 */

#include <canBusProcess.h>
#include "senloader.h"
#include "canBusProcess.h"
#include "ch.h"
#include "hal.h"
#include "dbus.h"
#include <can_lld.h>


static sen_motorStruct senloader;
static RC_Ctl_t* control;

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

static uint32_t abs(int32_t num) {
	if (num > 0) {
		return num;
	} else {
		return -num;
	}
}


static float senloader_pos_pid(pid_profile_t* setting, sen_pid_controller_t* data, float SP, float PV) {

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

int32_t SP = 2500;
int32_t STUCK = 1500;
uint8_t DEBUG = 0;

static THD_WORKING_AREA(senloader_control_wa, 2048);
static THD_FUNCTION(senloader_control, p) {

	(void)p;
	static uint8_t i;
	static int32_t SUM;

	while (!chThdShouldTerminateX()) {

//		if (!DEBUG && control->rc.s1 == 2) {
//			SP += abs(senloader._encoder->total_ecd - SP) < 2000 ? BULLET_ROTATION_CNT : 0;
//		}
//		while ((abs(senloader._encoder->total_ecd - SP) < 2000)) {
//			senloader.pos_sp = senloader_pos_pid(&senloader.setting,
//								&senloader.pidcontroller, SP, senloader._encoder->total_ecd);
//			can_motorSetCurrent(LOADER_CAN, 0x200, senloader.pos_sp, 0, 0, 0);
//		SUM = 0;
//		senloader.torque[senloader.torque_last] = senloader._encoder->raw_torque;
//		senloader.torque_last = senloader.torque_last < (TORQUE_REC_LEN - 1) ? senloader.torque_last + 1 : 0;
//		for (i = 0; i < TORQUE_REC_LEN; i++) {
//			SUM += senloader.torque[i];
//		}
//		senloader.torque_avg = SUM / TORQUE_REC_LEN;
//			if (senloader.torque_avg > 1000) {
//				can_motorSetCurrent(LOADER_CAN, 0x200, -800, 0, 0, 0);
//				chThdSleep(MS2ST(200));
//				SP = senloader._encoder->total_ecd;
//			}
//			chThdSleep(MS2ST(1));
//		}

//		if (senloader.torque_avg > 1000) {
//
//			can_motorSetCurrent(LOADER_CAN, 0x200, -200, 0, 0, 0);
//			chThdSleep(MS2ST(400));
//
//		} else {
//
//			if (!DEBUG && control->rc.s1 == 2) {
//
//				can_motorSetCurrent(LOADER_CAN, 0x200, 600, 0, 0, 0);
//
//			}
//
//		}

//		if (!DEBUG && control->rc.s1 == 2) {
//			can_motorSetCurrent(LOADER_CAN, 0x200, 600, 0, 0, 0);
//		} else {
//			can_motorSetCurrent(LOADER_CAN, 0x200, 0, 0, 0, 0);
//		}
//
//		chThdSleep(MS2ST(20));



		if (!DEBUG && control->rc.s1 == 2) {

			for (i = 0; i < 50; i++) {
				senloader.speed_sp = senloader_pos_pid(&senloader.setting, &senloader.pidcontroller,
																   SP, senloader._encoder->raw_speed);
				can_motorSetCurrent(LOADER_CAN, 0x200, senloader.speed_sp, 0, 0, 0);
				chThdSleep(MS2ST(1));
			}

			if (senloader._encoder->raw_speed < STUCK) {
				can_motorSetCurrent(LOADER_CAN, 0x200, -600, 0, 0, 0);
				chThdSleep(MS2ST(300));
			}


		} else {

			can_motorSetCurrent(LOADER_CAN, 0x200, 0, 0, 0, 0);

		}

		chThdSleep(MS2ST(1));

	}


}

void sen_loader_init (void) {

	control = RC_get();

	memset(&senloader, 0, sizeof(sen_motorStruct));

	senloader._encoder = can_getLoaderMotor();
	SP = 2500;
	STUCK = 1750;

	senloader.inverted = 0;

	senloader.setting.kp = POS_KP;
	senloader.setting.ki = POS_KI;
	senloader.setting.kd = POS_KD;
	senloader.setting.cv_limit = POS_OUTLIM;
	senloader.setting.integral_limit = POS_ILIM;

	senloader._encoder->torque_limit = LOADER_TORQUE_LIM;

	chThdCreateStatic(senloader_control_wa, sizeof(senloader_control_wa),
					  NORMALPRIO, senloader_control, NULL);

}

