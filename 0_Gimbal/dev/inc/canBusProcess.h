#ifndef _CAN_BUS_PROCESS_H_
#define _CAN_BUS_PROCESS_H_

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "hal.h"


#define GIMBAL_MOTOR_NUM  2U
#define CHASSIS_MOTOR_NUM 4U
/* CAN Bus 1 or 2 */
#define CAN_FEEDER_FEEDBACK_MSG_ID                  0x207
#define CAN_GIMBAL_YAW_FEEDBACK_MSG_ID              0x205
#define CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID            0x206

#define CAN_DBUS_ID                                 0x001
#define CAN_GIMBAL_RX_GAMEDATA_ID					0x002

#define CAN_NUC_GIMBAL_ENCODER_TXID									0x100		//100HZ
#define CAN_NUC_GIMBAL_IMU_TXID											0x101		//100HZ
#define CAN_NUC_POS_DATA_XY_TXID										0x102		//50HZ
#define CAN_NUC_POS_DATA_ZA_TXID										0x103		//50HZ
#define CAN_NUC_CHASSIS_DATA_TXID										0x104		//EVENT DRIVEN

#define CAN_NUC_CHASSIS_CONTROL_RXID								0x110		//100HZ
#define CAN_NUC_GIMBAL_CONTROL_RXID									0x111		//100HZ

#define CAN_ENCODER_RANGE           8192            // 0x2000

typedef enum
{
  GIMBAL_YAW = 0,
  GIMBAL_PITCH
}gimbal_num_t;

typedef struct {
  uint16_t raw_angle;
  int16_t  raw_current;
  int16_t  current_setpoint;

  int32_t round_count;
  float radian_angle; // Continuous

  bool updated;
} GimbalEncoder_canStruct;

typedef struct {
  uint16_t raw_angle;
  int16_t  raw_speed;
  int16_t act_current;
  uint8_t temperature;

  int32_t round_count;
  int32_t total_ecd;
  float radian_angle; // Continuous

  bool updated;
} ChassisEncoder_canStruct;

typedef struct {

	uint16_t raw_angle;
	int16_t  raw_speed;
	int16_t  raw_torque;

	int32_t round_count;
	int32_t total_ecd;
	int32_t torque_limit;
	int8_t  over_torque;

}Loader_canStruct;

typedef struct gimbalStatus_t {

	float pitchEncoder;				//CAN_NUC_GIMBAL_ENCODER_TXID
	float yawEncoder;
	float pitchAttitude;			//CAN_NUC_GIMBAL_IMU_TXID
	float yawAttitude;

}gimbalStatus_t;

typedef struct posStruct_t{	//UWB

	float x;									//CAN_NUC_POS_DATA_XY_TXID
	float y;
	float z;									//CAN_NUC_POS_DATA_ZA_TXID
	float angle;

}posStruct_t;

typedef struct chassisStruct_t{

	int32_t railPos;		//cm, CAN_NUC_CHASSIS_DATA_TXID
	uint8_t endStop;		//open : 0, left : 1, right : 2
	uint8_t hitPos;			//not hit : 0, front hit : 1, back hit : 2

}chassisStruct_t;

typedef struct sentryControl_t{

	uint8_t fireBullet;			//Standby : 0, Fire : 1, CAN_NUC_CHASSIS_CONTROL_RXID
	float chassisVelocity;	//-1 to 1 m/s
	float yawVelocity;			//CAN_NUC_GIMBAL_CONTROL_RXID
	float pitchVelocity;

}sentryControl_t;

typedef struct{
    uint16_t channel0;
    uint16_t channel1;
    uint8_t  s1;
    uint8_t  s2;
    uint16_t key_code;
} dbus_tx_canStruct;

typedef struct{

	int16_t		chassis_pos;
	uint16_t	shooter_heat;
	float		shooter_speed;

} GameData_rx;

volatile GameData_rx* can_getChassisdata(void);
volatile GimbalEncoder_canStruct* can_getGimbalMotor(void);
volatile ChassisEncoder_canStruct* can_getChassisMotor(void);
volatile Loader_canStruct* can_getLoaderMotor(void);
sentryControl_t* returnSentryControl(void);

void can_processInit(void);
void can_motorSetCurrent(CANDriver *const CANx,
  const uint16_t EID,
  const int16_t cm1_iq,
  const int16_t cm2_iq,
  const int16_t cm3_iq,
  const int16_t cm4_iq);

#endif
