#include "ch.h"
#include "hal.h"

#include "host_comm.h"
#include "gimbal.h"
#include "mpu6500.h"
#include "adis16265.h"

static bool                     sync_ok = false;
static int32_t                  timeStamp_sync;
static volatile gimbal_cmd_t*   gimbal_cmd;

static PIMUStruct      pIMU;
static GimbalStruct* gimbal;

#define ST2MS_DIV (CH_CFG_ST_FREQUENCY/1000U)

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
            timeStamp_sync = host_time - chVTGetSystemTimeX()/ST2MS_DIV;

        CANTxFrame txmsg;

        txmsg.IDE = CAN_IDE_STD;
        txmsg.EID = CAN_SYNC_G2H_ID;
        txmsg.RTR = CAN_RTR_DATA;
        txmsg.DLC = 0x04;

        txmsg.data32[0] = (uint32_t)(chVTGetSystemTimeX()/ST2MS_DIV + timeStamp_sync);

        canTransmit(HOST_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
    }
    else
        sync_ok = true;

}

systime_t hostComm_restoreChVT(const uint32_t rx_time)
{
    uint32_t timestamp_ms = chVTGetSystemTimeX()/ST2MS_DIV + timeStamp_sync;

    uint16_t timestamp_low16  = (uint16_t)(timestamp_ms);
    uint16_t timestamp_high16 = (uint16_t)(timestamp_ms >> 16);

    uint32_t rx_time_32 = 0;

    if(rx_time - timestamp_low16 > 32767 && timestamp_high16)
        rx_time_32 = ((timestamp_high16-1) << 16) | rx_time;
    else if(rx_time - timestamp_low16 < -32767)
        rx_time_32 = ((timestamp_high16+1) << 16) | rx_time;
    else
        rx_time_32 = (timestamp_high16 << 16) | rx_time;

    return (rx_time_32 + timeStamp_sync)*ST2MS_DIV;
}

void hostComm_processGimbalCmd(const CANRxFrame *const  rxmsg)
{
    can_gimbal_cmd_t can_gimbal_cmd;
    memcpy(&can_gimbal_cmd, rxmsg->data8, 8);

    chSysLock();
    gimbal_cmd->cv_rx_timer = chVTGetSystemTimeX();

    gimbal_cmd->timeStamp = hostComm_restoreChVT(can_gimbal_cmd.timeStamp_16);
    gimbal_cmd->ctrl_mode = can_gimbal_cmd.ctrl_mode;
    gimbal_cmd->yaw       = (float)(can_gimbal_cmd.yaw)/GIMBAL_CMD_ANGVEL_PSC;
    gimbal_cmd->pitch     = (float)(can_gimbal_cmd.pitch)/GIMBAL_CMD_ANGVEL_PSC;
    chSysUnlock();
}

void hostComm_txGimbalInfo(void)
{
    CANTxFrame txmsg;
    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = CAN_GIMBAL_INFO_ID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    gimbal_info0_t gimbal_info0;
    gimbal_info0.timeStamp_16 = (uint16_t)(chVTGetSystemTimeX()/ST2MS_DIV + timeStamp_sync);
    gimbal_info0.yaw   = (int16_t)((pIMU->euler_angle[Yaw] - 2*M_PI*pIMU->rev)*GIMBAL_INFO_ANG_PSC);
    gimbal_info0.pitch = (int16_t)(pIMU->euler_angle[Pitch] * GIMBAL_INFO_ANG_PSC);
    gimbal_info0.gimbal_pitch_angle = (int16_t)(gimbal->pitch_angle_enc*GIMBAL_INFO_ANG_PSC);

    memcpy(txmsg.data8, &gimbal_info0 , 8);
    canTransmit(HOST_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));

    gimbal_info1_t gimbal_info1;
    gimbal_info1.timeStamp_16 = (uint16_t)(chVTGetSystemTimeX()/ST2MS_DIV + timeStamp_sync);
    gimbal_info1.ang_vel[X] = (int16_t)(pIMU->gyroData[X]*GIMBAL_INFO_ANGVEL_PSC);
    gimbal_info1.ang_vel[Y] = (int16_t)(pIMU->gyroData[Y]*GIMBAL_INFO_ANGVEL_PSC);
    gimbal_info1.ang_vel[Z] = (int16_t)(pIMU->gyroData[Z]*GIMBAL_INFO_ANGVEL_PSC);

    txmsg.EID++;
    memcpy(txmsg.data8, &gimbal_info1 , 8);
    canTransmit(HOST_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

static void hostComm_txHeartbeat(void)
{
    //Send heartbeat data
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = CAN_HEARTBEAT_G_ID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x04;

    txmsg.data32[0] = (uint32_t)(chVTGetSystemTimeX()/ST2MS_DIV + timeStamp_sync);

    canTransmit(HOST_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

#define CAN_HOST_HEARTBEAT_MS    (1000U/CAN_HOST_HEARTBEAT_FREQ)
#define CAN_HOST_GIMBALINFO_MS   (1000U/CAN_HOST_GIMBALINFO_FREQ)
static THD_WORKING_AREA(host_thread_wa, 512);
static THD_FUNCTION(host_thread, p){
    (void) p;
    chRegSetThreadName("host machine communication");

    uint32_t tick = 0;
    while(!chThdShouldTerminateX())
    {
        if(!(tick % CAN_HOST_HEARTBEAT_MS))
            hostComm_txHeartbeat();
        if(sync_ok && !(tick % CAN_HOST_GIMBALINFO_MS))
            hostComm_txGimbalInfo();

        tick++;
        chThdSleepMilliseconds(1);
    }
}

void hostComm_init(void)
{
    pIMU   = imu_get();
    gimbal = gimbal_get();
    gimbal_cmd = gimbal_getCVCmd();

    chThdCreateStatic(host_thread_wa, sizeof(host_thread_wa),
                   NORMALPRIO, host_thread, NULL);
}
