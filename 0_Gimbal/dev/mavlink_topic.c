#include "ch.h"
#include "hal.h"

#include "infantry/mavlink.h"
#include "mavlink_helpers.h"
#include "mavlink_comm.h"

static float tx_occupation = 0.0f;
static systime_t tx_end = 0; //Estimated end_of_transmission time, used to prevent collision

/* definitions of mavlink messages*/
#if (MAVLINK_USE_HEARTBEAT == TRUE) && !defined(__DOXYGEN__)
  static mavlink_heartbeat_t* heartbeat_tx = NULL;
  static THD_WORKING_AREA(heartbeat_tx_wa, 512);
  static mavlink_heartbeat_t heartbeat;
  static bool heartbeat_received = false;
  static bool heartbeat_subscribed = false;

  mavlink_heartbeat_t* mavlinkComm_heartbeat_subscribe(void)
  {
    heartbeat_subscribed = true;
    return &heartbeat;
  }

  bool mavlinkComm_heartbeat_check(void)
  {
    bool result = heartbeat_received;
    heartbeat_received = false;
    return result;
  }

  static THD_FUNCTION(heartbeat_tx_func, p)
  {
    uint16_t freq = *(uint16_t*)p;
    chRegSetThreadName("mavlink heartbeat transmitter");

    mavlink_message_t message;
    const uint16_t buf_len = MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_HEARTBEAT_LEN;
    uint8_t buf[buf_len];

    const uint16_t period = US2ST(1e6 / freq);
    systime_t tick = chVTGetSystemTimeX();
    while(!chThdShouldTerminateX())
    {
      tick += period;
      systime_t end_tick = (tick > tx_end ? tick : tx_end);
      if(end_tick > chVTGetSystemTimeX())
        chThdSleepUntil(end_tick);
      else
        tick = chVTGetSystemTimeX();

      mavlink_msg_heartbeat_pack(0, 200, &message,
              heartbeat_tx->type,
              heartbeat_tx->base_mode,
              heartbeat_tx->custom_mode,
              heartbeat_tx->system_status);

      mavlink_msg_to_send_buffer(buf, &message);

      uartStopSend(UART_MAVLINK);
      uartStartSend(UART_MAVLINK, buf_len, buf);

      tx_end = chVTGetSystemTimeX() + US2ST(buf_len * 1e7 / UART_MAVLINK_BR) + 40; //Prevent collision
    }
  }

  mavlinkComm_publish_result_t mavlinkComm_heartbeat_publish(mavlink_heartbeat_t* const message,
    const uint16_t transmit_freq)
  {
    float occupation =
      tx_occupation + (float)transmit_freq*(MAVLINK_MSG_ID_HEARTBEAT_LEN + 9)/(0.1f * (float)UART_MAVLINK_BR);

    if(tx_occupation > 1.0f)
      return MAVLINK_COMM_PUBLISH_BANDWIDTH_OVERLOAD;
    if(heartbeat_tx != NULL)
      return MAVLINK_COMM_PUBLISH_FULL;

    heartbeat_tx = message;
    chThdCreateStatic(heartbeat_tx_wa, sizeof(heartbeat_tx_wa), NORMALPRIO - 7, heartbeat_tx_func, &transmit_freq);
  }
#endif
/**/

/* definitions of mavlink messages*/
#if (MAVLINK_USE_ATTITUDE == TRUE) && !defined(__DOXYGEN__)
  static mavlink_attitude_t* attitude_tx = NULL;
  static THD_WORKING_AREA(attitude_tx_wa, 512);
  static mavlink_attitude_t attitude;
  static bool attitude_received = false;
  static bool attitude_subscribed = false;

  mavlink_attitude_t* mavlinkComm_attitude_subscribe(void)
  {
    attitude_subscribed = true;
    return &attitude;
  }

  bool mavlinkComm_attitude_check(void)
  {
    bool result = attitude_received;
    attitude_received = false;
    return result;
  }

  static THD_FUNCTION(attitude_tx_func, p)
  {
    uint16_t freq = *(uint16_t*)p;
    chRegSetThreadName("mavlink attitude transmitter");

    mavlink_message_t message;
    const uint16_t buf_len = MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_HEARTBEAT_LEN;
    uint8_t buf[buf_len];

    const uint16_t period = US2ST(1e6 / freq);
    systime_t tick = chVTGetSystemTimeX();
    while(!chThdShouldTerminateX())
    {
      tick += period;
      systime_t end_tick = (tick > tx_end ? tick : tx_end);
      if(end_tick > chVTGetSystemTimeX())
        chThdSleepUntil(end_tick);
      else
        tick = chVTGetSystemTimeX();

      mavlink_msg_attitude_pack(0, 200, &message,
              attitude_tx->time_boot_ms,
              attitude_tx->roll,
              attitude_tx->pitch,
              attitude_tx->yaw,
              attitude_tx->rollspeed,
              attitude_tx->pitchspeed,
              attitude_tx->yawspeed);

      mavlink_msg_to_send_buffer(buf, &message);

      uartStopSend(UART_MAVLINK);
      uartStartSend(UART_MAVLINK, buf_len, buf);

      tx_end = chVTGetSystemTimeX() + US2ST(buf_len * 1e7 / UART_MAVLINK_BR) + 40; //Prevent collision
    }
  }

  mavlinkComm_publish_result_t mavlinkComm_attitude_publish(mavlink_attitude_t* const message,
    const uint16_t transmit_freq)
  {
    float occupation =
      tx_occupation + (float)transmit_freq*(MAVLINK_MSG_ID_HEARTBEAT_LEN + 9)/(0.1f * (float)UART_MAVLINK_BR);

    if(tx_occupation > 1.0f)
      return MAVLINK_COMM_PUBLISH_BANDWIDTH_OVERLOAD;
    if(attitude_tx != NULL)
      return MAVLINK_COMM_PUBLISH_FULL;

    attitude_tx = message;
    chThdCreateStatic(attitude_tx_wa, sizeof(attitude_tx_wa), NORMALPRIO - 7, attitude_tx_func, &transmit_freq);
  }
#endif
/**/

inline void _mavlinkComm_topic_decode(mavlink_message_t* const message)
{
  switch(message->msgid)
  {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
      #if (MAVLINK_USE_HEARTBEAT == TRUE) && !defined(__DOXYGEN__)
        if(heartbeat_subscribed)
        {
          chSysLock();
          heartbeat_received = true;
          mavlink_msg_heartbeat_decode(message, &heartbeat);
          chSysUnlock();
        }
      #endif
      break;
    }
    case MAVLINK_MSG_ID_ATTITUDE:
    {
      #if (MAVLINK_USE_ATTITUDE == TRUE) && !defined(__DOXYGEN__)
        if(attitude_subscribed)
        {
          chSysLock();
          attitude_received = true;
          mavlink_msg_attitude_decode(message, &attitude);
          chSysUnlock();
        }
      #endif
      break;
    }
  }
}
