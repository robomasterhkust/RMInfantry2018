/*
 * sentry_chassis.h
 *
 *  Created on: 2 Apr 2018
 *      Author: Alex Wong
 */

#ifndef SENTRY_CHASSIS_H_
#define SENTRY_CHASSIS_H_

#define SEN_CHASSIS_MOTOR_NUM		2

#define CHASSIS_CAN  				&CAND1
#define CHASSIS_CAN_EID  			0x200

#define POS_KP						8000000.0f
#define POS_KI						3.0f
#define POS_KD						10000.0f
#define POS_ILIM					50000.0f
#define POS_OUTLIM					10000.0f

#define WHEEL_DIA					0.05 //m
#define GEAR_RATIO				19
#define WHEEL_CIRCUM			(float) WHEEL_DIA * 3.1415926539
#define RAW2LINSPEED(x)		(x / 60) * WHEEL_CIRCUM	/ GEAR_RATIO	//x = raw speed from c620, gives m/s
#define ENC2LINDIST(x)    (x / 8192) * WHEEL_CIRCUM	/ GEAR_RATIO
#define MAXLINSPEED				1.2

#define STICK_NEUTURAL				1024
#define STICK_DELTA					660
#define STICK_GAIN					0.002

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
