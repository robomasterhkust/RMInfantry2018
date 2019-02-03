#ifndef _UART_PROTOCOL_H_
#define _UART_PROTOCOL_H_

/*
 *   NOTE: This header file is shared among
 *   robomaster computer(s) & MCU(s)
 *   =======SHOULD CHECK THE VERSION NUMBER BEFORE USE======
 *   =======VERSION: 2019.01.23=============================
 */
#define UART_START_BYTE        0xAA

#define UART_HEARTBEAT_G_ID    0x00
#define UART_SYNC_H2G_ID       0x01
#define UART_SYNC_G2H_ID       0x02
#define UART_GIMBAL_INFO_ID    0x05
#define UART_GIMBAL_CMD_ID     0x06
#define UART_PING_H2G_ID       0xFE
#define UART_PING_G2H_ID       0xFF

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
    uint8_t start;
    uint8_t type;
    uint8_t len;
} __attribute__((packed)) uart_header_t;

typedef uint8_t  uart_crc_t;
typedef struct
{
    uint32_t dt;
    int16_t  error;
    uint8_t  status;
} __attribute__((packed)) uart_sync_t;

typedef struct
{
    uint32_t timeStamp_32;
} __attribute__((packed)) uart_heartbeat_t;

typedef struct
{
    uint16_t timeStamp_16;
    int16_t  yaw;
    int16_t  pitch;
    int16_t  gimbal_pitch_angle;
    int16_t  ang_vel[3];
} __attribute__((packed)) uart_gimbal_info_t;

typedef struct
{
    uint16_t timeStamp_16;
    int16_t  yaw_velCmd;
    int16_t  pitch_velCmd;
    uint8_t  ctrl_mode;
    uint8_t  reserve;
} __attribute__((packed)) uart_gimbal_cmd_t;

typedef struct
{
    uart_heartbeat_t    heartbeat;
    uart_gimbal_info_t  gimbal_info;
} __attribute__((packed)) uart_protocol_t;

#endif
