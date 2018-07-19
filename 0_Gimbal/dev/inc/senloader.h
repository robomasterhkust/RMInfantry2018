/*
 * senloader.h
 *
 *  Created on: 5 Apr 2018
 *      Author: Alex Wong
 */

#ifndef INC_SENLOADER_H_
#define INC_SENLOADER_H_

#define LOADER_CAN  				&CAND1
#define LOADER_CAN_EID  			0x1FF

#define POS_KP						2.0f
#define POS_KI						0.0f
#define POS_KD						0.0f
#define POS_ILIM					50000.0f
#define POS_OUTLIM					4000.0f

#define LOADER_TORQUE_LIM			4000

#define BULLET_ROTATION_CNT			32768

#define SENTRY_HEAT_LIMIT			360
#define SENTRY_HEAT_BUFFER			220

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

#define TORQUE_REC_LEN		100

typedef struct sen_motorStruct{

	float speed_sp;
	float speed;
	float pos_sp;
	float pos;
	float torque_lim;
	int32_t torque[TORQUE_REC_LEN];
	uint8_t torque_last;
	float torque_avg;
	sen_pid_controller_t pidcontroller;
	pid_profile_t setting;
	Loader_canStruct* _encoder;
	uint8_t	inverted;

}sen_motorStruct;

sen_motorStruct* returnLoader(void);

void sen_loader_init (void);

#endif /* INC_SENLOADER_H_ */
