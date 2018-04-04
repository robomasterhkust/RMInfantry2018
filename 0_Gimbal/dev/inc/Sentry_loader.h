/*
 * Sentry_loader.h
 *
 *  Created on: 4 Apr 2018
 *      Author: Alex Wong
 */

#ifndef SENTRY_LOADER_H_
#define SENTRY_LOADER_H_

#define LOADER_CAN  				&CAND1
#define LOADER_CAN_EID  			0x200

#define POS_KP						1.0f
#define POS_KI						0.0f
#define POS_KD						0.0f
#define POS_ILIM					50000.0f
#define POS_OUTLIM					5000.0f

#define LOADER_TORQUE_LIM			100000

#define BULLET_ROTATION_CNT			32768

typedef struct pid_profile_t{

	float kp;
	float ki;
	float kd;

	uint32_t cv_limit;
	uint32_t integral_limit;

}pid_profile_t;

typedef struct sen_pid_controller_t{

	  //float kp;
	  //float ki;
	  //float kd;

	  float sp;
	  float pv;
	  float error[2];

	  float pout;
	  float iout;
	  float dout;

	  float cv;

	  //uint32_t cv_limit;

	  //uint32_t integral_limit;

}sen_pid_controller_t;

typedef struct sen_motorStruct{

	float speed_sp;
	float speed;
	float pos_sp;
	float pos;
	float torque_lim;
	sen_pid_controller_t pidcontroller;
	pid_profile_t setting;
	Loader_canStruct* _encoder;
	uint8_t	inverted;

}sen_motorStruct;

void sen_loader_init (void);

#endif /* INC_SENTRY_LOADER_H_ */
