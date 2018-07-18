#ifndef _CAN_BUS_PROCESS_H_
#define _CAN_BUS_PROCESS_H_

#include "stdint.h"
#include "stdbool.h"
#include "hal.h"
#include "string.h"
#include "adis16265.h"

#define GIMBAL_MOTOR_NUM  2U
#define CHASSIS_MOTOR_NUM 4U
#define EXTRA_MOTOR_NUM   2U

/* CAN Bus 1 or 2 */
#define CAN_RM_MOTOR_ID1													  0x1FF
#define CAN_RM_MOTOR_ID2														0x200
#define CAN_CHASSIS_FR_FEEDBACK_MSG_ID              0x201
#define CAN_CHASSIS_FL_FEEDBACK_MSG_ID              0x202
#define CAN_CHASSIS_BL_FEEDBACK_MSG_ID              0x203
#define CAN_CHASSIS_BR_FEEDBACK_MSG_ID              0x204
#define CAN_GIMBAL_YAW_FEEDBACK_MSG_ID              0x205
#define CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID            0x206

#define CAN_GIMBAL_SEND_DBUS_ID                     0x001
#define CAN_GIMBAL_TX_GAMEDATA_ID					          0x002

#define CAN_NUC_GIMBAL_ENCODER_TXID									0x100		//100HZ
#define CAN_NUC_GIMBAL_IMU_TXID											0x101		//100HZ
#define CAN_NUC_POS_DATA_XY_TXID										0x102		//50HZ
#define CAN_NUC_POS_DATA_ZA_TXID										0x103		//50HZ
#define CAN_NUC_CHASSIS_DATA_TXID										0x104		//EVENT DRIVEN

#define CAN_NUC_CHASSIS_CONTROL_RXID								0x110		//100HZ
#define CAN_NUC_GIMBAL_CONTROL_RXID									0x111		//100HZ

#define CAN_ENCODER_RANGE           8192            // 0x2000
#define CAN_ENCODER_RADIAN_RATIO    7.669904e-4f    // 2*M_PI / 0x2000

typedef enum
{
  GIMBAL_YAW = 0,
  GIMBAL_PITCH
}gimbal_num_t;

typedef enum
{
//  FRONT_LEFT = 1,
//  FRONT_RIGHT = 3,
//  BACK_LEFT = 0,
//  BACK_RIGHT = 2
    FRONT_RIGHT = 0,
    FRONT_LEFT = 1,
    BACK_LEFT = 2,
    BACK_RIGHT = 3
}chassis_num_t;

typedef struct {
    uint16_t raw_angle;
    int16_t  raw_current;
    int16_t  current_setpoint;

    uint16_t last_raw_angle;
    uint16_t offset_raw_angle;
    int32_t round_count;
    int32_t total_ecd;
    float radian_angle; // Continuous

    bool updated;
} GimbalEncoder_canStruct;

typedef struct {
    uint16_t raw_angle;
    int16_t  raw_speed;
    int16_t act_current;
    uint8_t temperature;
    uint16_t last_raw_angle;
    uint16_t offset_raw_angle;
    uint32_t msg_count;
    int32_t round_count;
    int32_t total_ecd;
    float radian_angle; // Continuous

    bool updated;
} ChassisEncoder_canStruct;

typedef struct{
    uint16_t channel0;
    uint16_t channel1;
    uint8_t  s1;
    uint8_t  s2;
    uint16_t key_code;
} Gimbal_Send_Dbus_canStruct;

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

	int16_t		chassis_pos;
	uint16_t	shooter_heat;
	float		shooter_speed;

} GameData_tx;

volatile GimbalEncoder_canStruct* can_getGimbalMotor(void);
volatile ChassisEncoder_canStruct* can_getChassisMotor(void);
volatile ChassisEncoder_canStruct* can_getExtraMotor(void);
volatile Gimbal_Send_Dbus_canStruct* can_get_sent_dbus(void);

void can_processInit(void);
void can_motorSetCurrent(CANDriver *const CANx,
  const uint16_t EID,
  const int16_t cm1_iq,
  const int16_t cm2_iq,
  const int16_t cm3_iq,
  const int16_t cm4_iq);


#endif

