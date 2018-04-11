/*
 * sentry_chassis.h
 *
 *  Created on: 2 Apr 2018
 *      Author: Alex Wong
 */

#ifndef SENTRY_CHASSIS_H_
#define SENTRY_CHASSIS_H_

#define SEN_CHASSIS_MOTOR_NUM		2

#define CHASSIS_CAN  				&CAND2
#define CHASSIS_CAN_EID  			0x200

#define POS_KP						1.0f
#define POS_KI						0.02f
#define POS_KD						50.0f
#define POS_ILIM					50000.0f
#define POS_OUTLIM					10000.0f

#define WHEEL_DIA					50 //mm
#define CNT_2_DIS					0.0010091979

#define STICK_NEUTURAL				1024
#define STICK_DELTA					660
#define STICK_GAIN					0.005

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
	float pos_offset;
	float pos_limit;
	sen_pid_controller_t pidcontroller;
	uint8_t	inverted;

}sen_motorStruct;

typedef struct senchassisstruct{

	ChassisEncoder_canStruct* encoders;
	sen_motorStruct _motors[SEN_CHASSIS_MOTOR_NUM];
	pid_profile_t pospidprofile;

	float pos;
	float pos_sp;

	bool right_flag;
	bool left_flag;

}senchassisstruct;

volatile senchassisstruct* get_chassis(void);

void sen_chassis_init (void);

#endif /* SENTRY_CHASSIS_H_ */
