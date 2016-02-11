#ifndef _HOST_COMM_ID_H_
#define _HOST_COMM_ID_H_

/* 
 *   NOTE: This header file is shared among all 
 *   robomaster computer(s) & MCU(s) on the can bus
 *   =======SHOULD CHECK THE VERSION NUMBER BEFORE USE======
 *   =======VERSION: 2018.11.23=============================
 */

#define CAN_HOST_SYNC_H2G_ID  0x105
#define CAN_HOST_SYNC_G2H_ID  0x106
#define CAN_GIMBAL_INFO_ID    0x107
#define CAN_HEARTBEAT_G_ID    0x120
#define CAN_PING_ID           0x150

#define GIMBAL_INFO_ANGVEL_PSC     900
#define GIMBAL_INFO_ANG_PSC      10000

typedef struct
{
    uint16_t timeStamp_16;
    int16_t  yaw;
    int16_t  pitch;
    int16_t  gimbal_pitch_angle;
} __attribute__((packed)) gimbal_info0_t;

typedef struct
{
    uint16_t timeStamp_16;
    int16_t  ang_vel[3];
} __attribute__((packed)) gimbal_info1_t;

#endif