#ifndef _HOST_COMM_ID_H_
#define _HOST_COMM_ID_H_

/*
 *   NOTE: This header file is shared among all
 *   robomaster computer(s) & MCU(s) on the can bus
 *   =======SHOULD CHECK THE VERSION NUMBER BEFORE USE======
 *   =======VERSION: 2018.12.15=============================
 */

#define CAN_SYNC_H2G_ID       0x105
#define CAN_SYNC_G2H_ID       0x106
#define CAN_GIMBAL_INFO_ID    0x107
#define CAN_HEARTBEAT_G_ID    0x120
#define CAN_GIMBAL_CMD_ID     0x121
#define CAN_PING_H2G_ID       0x150
#define CAN_PING_G2H_ID       0x151

#define GIMBAL_INFO_ANGVEL_PSC     900
#define GIMBAL_INFO_ANG_PSC      10000
#define GIMBAL_CMD_ANGVEL_PSC      900

typedef enum
{
    CTRL_MODE_IDLE  = 0,
    CTRL_MODE_AIM_0 = 1,
} gimbal_ctrl_mode_t;

typedef struct
{
    //Frame 1
    uint32_t timeStamp_32;
} __attribute__((packed)) heartbeat_t;

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

typedef struct
{
    uint16_t timeStamp_16;
    int16_t  yaw;
    int16_t  pitch;
    uint8_t  ctrl_mode;
    uint8_t  reserve;
} __attribute__((packed)) can_gimbal_cmd_t;

typedef struct
{
    heartbeat_t    heartbeat;
    gimbal_info0_t gimbal_info0;
    gimbal_info1_t gimbal_info1;
} __attribute__((packed)) can_protocol_t;

#endif
