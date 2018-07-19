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
#include "mavlink_comm.h"


static sen_motorStruct senloader;
static GameData_rx* judge_fb;
static RC_Ctl_t* control;
static mavlink_attitude_t* mavv;

sen_motorStruct* returnLoader(void) {

	return &senloader;

}

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

int32_t SP = 3000;
int32_t STUCK = 1500;
uint8_t DEBUG = 1;

static THD_WORKING_AREA(senloader_control_wa, 2048);
static THD_FUNCTION(senloader_control, p) {

	(void)p;
	static uint8_t i;
	static int32_t SUM;

	while (!chThdShouldTerminateX()) {

//		if ((!DEBUG && control->rc.s1 == 2 ||
//			mavv->roll != 0) &&
//			judge_fb->shooter_heat < (SENTRY_HEAT_LIMIT - SENTRY_HEAT_BUFFER)) {
		if(control->rc.s1 == 2) {

			for (i = 0; i < 50; i++) {
				senloader.speed_sp = senloader_pos_pid(&senloader.setting, &senloader.pidcontroller,
																   SP, senloader._encoder->raw_speed);

				can_motorSetCurrent(LOADER_CAN, 0x200, 0, 0, senloader.speed_sp, 0);
				chThdSleep(MS2ST(1));
			}

			if (senloader._encoder->raw_speed < STUCK) {
				can_motorSetCurrent(LOADER_CAN, 0x200, 0, 0, -2500, 0);
				chThdSleep(MS2ST(500));
			}


		} else {

			senloader.speed_sp = 0;
			can_motorSetCurrent(LOADER_CAN, 0x200, 0, 0, 0, 0);

		}

		chThdSleep(MS2ST(1));

	}


}

void sen_loader_init (void) {

	control = RC_get();

	mavv = mavlinkComm_attitude_subscribe();

	judge_fb = can_getChassisdata();

	memset(&senloader, 0, sizeof(sen_motorStruct));

	senloader._encoder = can_getLoaderMotor();
	SP = 2000;
	STUCK = 400;

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

