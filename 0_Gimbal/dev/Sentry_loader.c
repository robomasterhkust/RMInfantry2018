/*
 * Sentry_loader.c
 *
 *  Created on: 4 Apr 2018
 *      Author: Alex Wong
 */

#include <canBusProcess.h>
#include "Sentry_loader.h"
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

int32_t SP = 0;
uint8_t DEBUG = 1;

static THD_WORKING_AREA(senloader_control_wa, 2048);
static THD_FUNCTION(senloader_control, p) {

	(void)p;

	while (!chThdShouldTerminateX()) {

		if (senloader._encoder->raw_torque > senloader._encoder->torque_limit
			|| senloader._encoder->raw_torque < -senloader._encoder->torque_limit) {
			senloader._encoder->over_torque = 1;
			  //can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);
		  }

		if (senloader._encoder->over_torque) {
			can_motorSetCurrent(LOADER_CAN, 0x200, -(LOADER_TORQUE_LIM - 100), 0, 0, 0);
			chThdSleep(MS2ST(100));
		} else {
			if (!DEBUG) {
				SP += abs(senloader._encoder->total_ecd - SP) < 1000 ? BULLET_ROTATION_CNT : 0;
			}
			senloader.pos_sp = senloader_pos_pid(&senloader.setting,
								&senloader.pidcontroller, SP, senloader._encoder->total_ecd);
			can_motorSetCurrent(LOADER_CAN, 0x200, senloader.pos_sp, 0, 0, 0);
		}



		chThdSleep(MS2ST(1));
	}


}

void sen_loader_init (void) {

	control = RC_get();

	memset(&senloader, 0, sizeof(sen_motorStruct));

	senloader._encoder = can_getLoaderMotor();
	SP = 0;

	senloader.inverted = 0;

	senloader.setting.kp = POS_KP;
	senloader.setting.ki = POS_KI;
	senloader.setting.kd = POS_KD;
	senloader.setting.cv_limit = POS_OUTLIM;
	senloader.setting.integral_limit = POS_ILIM;

	senloader._encoder = can_getLoaderMotor();

	senloader._encoder->torque_limit = LOADER_TORQUE_LIM;

	chThdCreateStatic(senloader_control_wa, sizeof(senloader_control_wa),
					  NORMALPRIO, senloader_control, NULL);

}
