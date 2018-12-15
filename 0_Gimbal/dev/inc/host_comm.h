#ifndef _HOST_COMM_H_
#define _HOST_COMM_H_

#include "host_comm_id.h"

#define HOST_CAN    &CAND2

#define CAN_HOST_HEARTBEAT_FREQ       1U
#define CAN_HOST_GIMBALINFO_FREQ    500U

typedef struct{
    systime_t cv_rx_timer; //Disable self-aiming control if the communication is dead
    systime_t timeStamp;

    uint8_t   ctrl_mode;
    uint8_t   shoot_cmd;
    float     yaw;
    float     pitch;
} gimbal_cmd_t;

int32_t                hostComm_getSync(void);
void                   hostComm_sync(const CANRxFrame *const rxmsg);
void                   hostComm_processGimbalCmd(const CANRxFrame *const  rxmsg);
volatile gimbal_cmd_t* hostComm_getGimbalCmd(void);
void                   hostComm_txGimbalInfo(void);
systime_t              hostComm_restoreChVT(const uint32_t rx_time);
void                   hostComm_init(void);

#endif
