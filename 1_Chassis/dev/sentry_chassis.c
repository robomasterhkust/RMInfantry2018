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


int32_t SP = 0;

static THD_WORKING_AREA(senchassis_control_wa, 2048);
static THD_FUNCTION(senchassis_control, p) {

	(void)p;
	uint8_t i = 0;

	static Gimbal_Send_Dbus_canStruct* RC_cntrl;

	RC_cntrl = can_get_sent_dbus();

	while (!chThdShouldTerminateX()) {

		//chSysLock();
		SP += (RC_cntrl->channel0 - STICK_NEUTURAL) * STICK_GAIN;
		for (i = 0; i < SEN_CHASSIS_MOTOR_NUM; i++) {
			senchassis._motors[i].pos = senchassis.encoders[i].total_ecd;
			senchassis._motors[i].pos_sp = SP / CNT_2_DIS;
			senchassis_pos_pid(&senchassis.pospidprofile, &senchassis._motors[i].pidcontroller,
							   (senchassis._motors[i].inverted ? -senchassis._motors[i].pos : senchassis._motors[i].pos),
							   senchassis._motors[i].pos_sp);
		}
		can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, senchassis._motors[0].pidcontroller.cv,
														  -senchassis._motors[1].pidcontroller.cv, 0, 0);
		//chSysUnlock();

		chThdSleep(MS2ST(1));
	}


}

void sen_chassis_init (void) {

	memset(&senchassis, 0, sizeof(senchassisstruct));

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

