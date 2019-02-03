#include "ch.h"
#include "hal.h"

#include "uart_host.h"
#include "UartProtocol.h"
#include "gimbal.h"
#include "math.h"

static bool                     sync_ok = false;
static int32_t                  timeStamp_sync;

static PIMUStruct      pIMU;
static GimbalStruct* gimbal;
static volatile gimbal_cmd_t* gimbal_cmd;

static uint8_t txbuf[30];
static uint8_t rxbuf[30];
static uint8_t* rxData;
static thread_reference_t uart_receive_thread_handler = NULL;

#define ST2MS_DIV (CH_CFG_ST_FREQUENCY/1000U)

void uart_rxend_cb(UARTDriver* uartp)
{
    chSysLockFromISR();
    chThdResumeI(&uart_receive_thread_handler, MSG_OK);
    chSysUnlockFromISR();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
  NULL,NULL,uart_rxend_cb,NULL,NULL,
  UART_OSDK_BR,
  0,
  0,
  0
};

void uartHost_txGimbalInfo(void)
{
    uint8_t byte_offset = 0;
    uint8_t len = sizeof(uart_header_t)+sizeof(uart_gimbal_info_t)+sizeof(uart_crc_t);
    uart_header_t* header = (uart_header_t*)(txbuf + byte_offset);
    header->start = UART_START_BYTE;
    header->type  = UART_GIMBAL_INFO_ID;
    header->len   = len;
    byte_offset += sizeof(uart_header_t);

    uart_gimbal_info_t* gimbal_info = (uart_gimbal_info_t*)(txbuf + byte_offset);
    gimbal_info->timeStamp_16 = (uint16_t)(chVTGetSystemTimeX()/ST2MS_DIV + timeStamp_sync);
    gimbal_info->yaw   = (int16_t)((pIMU->euler_angle[Yaw] - 2*M_PI*pIMU->rev)*GIMBAL_INFO_ANG_PSC);
    gimbal_info->pitch = (int16_t)(pIMU->euler_angle[Pitch] * GIMBAL_INFO_ANG_PSC);
    gimbal_info->gimbal_pitch_angle = (gimbal->pitch_angle_enc*GIMBAL_INFO_ANG_PSC);
    gimbal_info->timeStamp_16 = (uint16_t)(chVTGetSystemTimeX()/ST2MS_DIV + timeStamp_sync);
    gimbal_info->ang_vel[X] = (int16_t)(pIMU->gyroData[X]*GIMBAL_INFO_ANGVEL_PSC);
    gimbal_info->ang_vel[Y] = (int16_t)(pIMU->gyroData[Y]*GIMBAL_INFO_ANGVEL_PSC);
    gimbal_info->ang_vel[Z] = (int16_t)(pIMU->gyroData[Z]*GIMBAL_INFO_ANGVEL_PSC);

    uartStopSend(UART_HOST);
    uartStartSend(UART_HOST, len, txbuf);
}

static systime_t uartHost_restoreChVT(const uint32_t rx_time)
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

static bool uartHost_sync(const uart_sync_t* sync)
{
    if(sync->status == 1)
        return true;

    int16_t sync_error = sync->error;
    if(sync_error)
        timeStamp_sync += sync_error/200;
    else
        timeStamp_sync = sync->dt - chVTGetSystemTimeX()/ST2MS_DIV;

    uint8_t byte_offset = 0;
    uint8_t len = sizeof(uart_header_t)+sizeof(uart_sync_t)+sizeof(uart_crc_t);
    uart_header_t* header = (uart_header_t*)(txbuf + byte_offset);
    header->start = UART_START_BYTE;
    header->type  = UART_SYNC_G2H_ID;
    header->len   = len;
    byte_offset += sizeof(uart_header_t);

    uart_sync_t* feedback = (uart_sync_t*)(txbuf + byte_offset);
    feedback->dt = (uint32_t)(chVTGetSystemTimeX()/ST2MS_DIV + timeStamp_sync);

    uartStopSend(UART_HOST);
    uartStartSend(UART_HOST, len, txbuf);
    chThdSleepMilliseconds(2);

    return false;
}

void uartHost_processGimbalCmd(const uart_gimbal_cmd_t* cmd)
{
    chSysLock();
    gimbal_cmd->cv_rx_timer = chVTGetSystemTimeX();

    gimbal_cmd->timeStamp = uartHost_restoreChVT(cmd->timeStamp_16);
    gimbal_cmd->ctrl_mode = cmd->ctrl_mode;
    gimbal_cmd->yaw       = (float)(cmd->yaw_velCmd)/GIMBAL_CMD_ANGVEL_PSC;
    gimbal_cmd->pitch     = (float)(cmd->pitch_velCmd)/GIMBAL_CMD_ANGVEL_PSC;
    chSysUnlock();
}

static THD_WORKING_AREA(uart_host_tx_wa, 128);
static THD_FUNCTION(uart_host_tx, p){
    (void) p;
    chRegSetThreadName("host communication TX");

    systime_t tick = chVTGetSystemTimeX();
    while(!chThdShouldTerminateX())
    {
        tick+= (CH_CFG_ST_FREQUENCY/UART_HOST_GIMBALINFO_FREQ);
        if(chVTGetSystemTimeX() < tick)
            chThdSleepUntil(tick);
        else
        {
            tick = chVTGetSystemTimeX();
        }

        if(sync_ok)
            uartHost_txGimbalInfo();
    }
}

static THD_WORKING_AREA(uart_host_rx_wa, 128);
static THD_FUNCTION(uart_host_rx, p){
    (void) p;
    chRegSetThreadName("host communication RX");

    size_t rx_size;
    while(!chThdShouldTerminateX())
    {
        uartStopReceive(UART_HOST);
        rxbuf[0] = 0;
        if(sync_ok)
        {
            uartStartReceive(UART_HOST,
                sizeof(uart_header_t)+sizeof(uart_gimbal_cmd_t)+sizeof(uart_crc_t), rxbuf);
            chSysLock();
            chThdSuspendS(&uart_receive_thread_handler);
            chSysUnlock();
            if(rxbuf[0] != UART_START_BYTE)
            {
                //set error flag
            }
            else
                uartHost_processGimbalCmd((uart_gimbal_cmd_t*)rxData);
        }
        else
        {
            uartStartReceive(UART_HOST,
                sizeof(uart_header_t)+sizeof(uart_sync_t)+sizeof(uart_crc_t), rxbuf);
            chSysLock();
            chThdSuspendS(&uart_receive_thread_handler);
            chSysUnlock();
            if(rxbuf[0] != UART_START_BYTE)
            {
                //set error flag
            }
            else
                sync_ok = uartHost_sync((uart_sync_t*)rxData);
        }
    }
}

void uartHost_init(void)
{
    gimbal = gimbal_get();
    gimbal_cmd = gimbal_getCVCmd();
    pIMU = imu_get();
    rxData = rxbuf + sizeof(uart_header_t);

    uartStart(UART_HOST, &uart_cfg);

    chThdCreateStatic(uart_host_tx_wa, sizeof(uart_host_tx_wa), NORMALPRIO - 4, uart_host_tx ,NULL);
    chThdCreateStatic(uart_host_rx_wa, sizeof(uart_host_rx_wa), NORMALPRIO + 4, uart_host_rx ,NULL);
}
