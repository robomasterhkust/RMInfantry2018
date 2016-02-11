#ifndef _HOST_COMM_H_
#define _HOST_COMM_H_

#define HOST_CAN    &CAND2

#define CAN_HOST_HEARTBEAT_FREQ     1
#define CAN_HOST_GIMBALINFO_FREQ    5

typedef struct{
    uint8_t   ctrl_mode;    //data8[0]
    uint8_t   shoot_cmd;    //data8[1]
    systime_t timeStamp;    //data16[1]
    float     cmd_yaw;      //data16[2]
    float     cmd_pitch;    //data16[3]
} host_gimbal_cmd_t;

int32_t hostComm_getSync(void);
void    hostComm_sync(const CANRxFrame *const rxmsg);
void    hostComm_processGimbalCmd
    (   host_gimbal_cmd_t* const gimbal_cmd,
        const CANRxFrame *const  rxmsg);
void    hostComm_txGimbalInfo(void);
void    hostComm_init(void);

#endif