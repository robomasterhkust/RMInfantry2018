#include "ch.h"
#include "hal.h" 

#include "host_comm.h"
#include "host_comm_id.h"
#include "gimbal.h"
#include "mpu6500.h"
#include "adis16265.h"

static bool    sync_ok = false;
static int32_t timeStamp_sync;

PIMUStruct      pIMU;
GimbalStruct* gimbal;

int32_t hostComm_getSync(void)
{
    return timeStamp_sync;
}

void hostComm_sync(const CANRxFrame *const rxmsg)
{
    if(!rxmsg->data8[6])
    {
        sync_ok = false;

        uint32_t host_time = (uint32_t)(rxmsg->data32[0]);
        uint16_t       udt = rxmsg->data16[2];
        int16_t dt = *((int16_t*)&udt);

        if(dt)
            timeStamp_sync += dt/200;
        else
            timeStamp_sync = host_time - ST2MS(chVTGetSystemTimeX());

        CANTxFrame txmsg;

        txmsg.IDE = CAN_IDE_STD;
        txmsg.EID = CAN_HOST_SYNC_G2H_ID;
        txmsg.RTR = CAN_RTR_DATA;
        txmsg.DLC = 0x04;

        txmsg.data32[0] = (uint32_t)(ST2MS(chVTGetSystemTimeX()) + timeStamp_sync);

        canTransmit(HOST_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
    }
    else
        sync_ok = true;

}

void hostComm_processGimbalCmd(host_gimbal_cmd_t* const gimbal_cmd,
        const CANRxFrame *const  rxmsg)
{

}

void hostComm_txGimbalInfo(void)
{  
    CANTxFrame txmsg;
    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = CAN_GIMBAL_INFO_ID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    gimbal_info0_t gimbal_info0;
    gimbal_info0.timeStamp_16 = (ST2MS(chVTGetSystemTimeX()) + timeStamp_sync) & 0x0000FFFF;
    gimbal_info0.yaw   = (int16_t)((pIMU->euler_angle[Yaw] - 2*M_PI*pIMU->rev)*GIMBAL_INFO_ANG_PSC);
    gimbal_info0.pitch = (int16_t)(pIMU->euler_angle[Pitch] * GIMBAL_INFO_ANG_PSC);
    gimbal_info0.gimbal_pitch_angle = (int16_t)(gimbal->pitch_angle_enc*GIMBAL_INFO_ANG_PSC);
    
    memcpy(txmsg.data8, &gimbal_info0 , 8);
    canTransmit(HOST_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));

    gimbal_info1_t gimbal_info1;
    gimbal_info1.timeStamp_16 = (ST2MS(chVTGetSystemTimeX()) + timeStamp_sync) & 0x0000FFFF;
    gimbal_info1.ang_vel[X] = (int16_t)(pIMU->gyroData[X]*GIMBAL_INFO_ANGVEL_PSC);
    gimbal_info1.ang_vel[Y] = (int16_t)(pIMU->gyroData[Y]*GIMBAL_INFO_ANGVEL_PSC);
    gimbal_info1.ang_vel[Z] = (int16_t)(pIMU->gyroData[Z]*GIMBAL_INFO_ANGVEL_PSC);
    
    txmsg.EID++;
    memcpy(txmsg.data8, &gimbal_info1 , 8);
    canTransmit(HOST_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

void hostComm_txHeartbeat(void)
{
    //Send heartbeat data
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = CAN_HEARTBEAT_G_ID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x04;

    txmsg.data32[0] = (uint32_t)(ST2MS(chVTGetSystemTimeX()) + timeStamp_sync);

    canTransmit(HOST_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

#define CAN_HOST_HEARTBEAT_MS    1000/CAN_HOST_HEARTBEAT_FREQ
#define CAN_HOST_GIMBALINFO_MS   1000/CAN_HOST_GIMBALINFO_FREQ
static THD_WORKING_AREA(host_thread_wa, 512);
static THD_FUNCTION(host_thread, p){
    (void) p;
    chRegSetThreadName("host machine communication");
    
    uint32_t tick = 0;
    while(!chThdShouldTerminateX())
    {
        if(!(tick % CAN_HOST_HEARTBEAT_MS))
            hostComm_txHeartbeat();
        if(sync_ok && !(tick % 200))
            hostComm_txGimbalInfo();

        tick++;
        chThdSleepMilliseconds(1);
    }
}

void hostComm_init(void)
{
    pIMU   = imu_get();
    gimbal = gimbal_get();

    chThdCreateStatic(host_thread_wa, sizeof(host_thread_wa),
                   NORMALPRIO, host_thread, NULL);
}