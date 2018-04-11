#pragma once
// MESSAGE RAIL_SPEED_TARGET PACKING

#define MAVLINK_MSG_ID_RAIL_SPEED_TARGET 36

MAVPACKED(
typedef struct __mavlink_rail_speed_target_t {
 float move_speed; /*< */
}) mavlink_rail_speed_target_t;

#define MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN 4
#define MAVLINK_MSG_ID_RAIL_SPEED_TARGET_MIN_LEN 4
#define MAVLINK_MSG_ID_36_LEN 4
#define MAVLINK_MSG_ID_36_MIN_LEN 4

#define MAVLINK_MSG_ID_RAIL_SPEED_TARGET_CRC 53
#define MAVLINK_MSG_ID_36_CRC 53



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RAIL_SPEED_TARGET { \
    36, \
    "RAIL_SPEED_TARGET", \
    1, \
    {  { "move_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rail_speed_target_t, move_speed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RAIL_SPEED_TARGET { \
    "RAIL_SPEED_TARGET", \
    1, \
    {  { "move_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rail_speed_target_t, move_speed) }, \
         } \
}
#endif

/**
 * @brief Pack a rail_speed_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param move_speed 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rail_speed_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float move_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN];
    _mav_put_float(buf, 0, move_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN);
#else
    mavlink_rail_speed_target_t packet;
    packet.move_speed = move_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAIL_SPEED_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_CRC);
}

/**
 * @brief Pack a rail_speed_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param move_speed 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rail_speed_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float move_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN];
    _mav_put_float(buf, 0, move_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN);
#else
    mavlink_rail_speed_target_t packet;
    packet.move_speed = move_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAIL_SPEED_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_CRC);
}

/**
 * @brief Encode a rail_speed_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rail_speed_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rail_speed_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rail_speed_target_t* rail_speed_target)
{
    return mavlink_msg_rail_speed_target_pack(system_id, component_id, msg, rail_speed_target->move_speed);
}

/**
 * @brief Encode a rail_speed_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rail_speed_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rail_speed_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rail_speed_target_t* rail_speed_target)
{
    return mavlink_msg_rail_speed_target_pack_chan(system_id, component_id, chan, msg, rail_speed_target->move_speed);
}

/**
 * @brief Send a rail_speed_target message
 * @param chan MAVLink channel to send the message
 *
 * @param move_speed 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rail_speed_target_send(mavlink_channel_t chan, float move_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN];
    _mav_put_float(buf, 0, move_speed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAIL_SPEED_TARGET, buf, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_CRC);
#else
    mavlink_rail_speed_target_t packet;
    packet.move_speed = move_speed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAIL_SPEED_TARGET, (const char *)&packet, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_CRC);
#endif
}

/**
 * @brief Send a rail_speed_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rail_speed_target_send_struct(mavlink_channel_t chan, const mavlink_rail_speed_target_t* rail_speed_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rail_speed_target_send(chan, rail_speed_target->move_speed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAIL_SPEED_TARGET, (const char *)rail_speed_target, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_CRC);
#endif
}

#if MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rail_speed_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float move_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, move_speed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAIL_SPEED_TARGET, buf, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_CRC);
#else
    mavlink_rail_speed_target_t *packet = (mavlink_rail_speed_target_t *)msgbuf;
    packet->move_speed = move_speed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAIL_SPEED_TARGET, (const char *)packet, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_CRC);
#endif
}
#endif

#endif

// MESSAGE RAIL_SPEED_TARGET UNPACKING


/**
 * @brief Get field move_speed from rail_speed_target message
 *
 * @return 
 */
static inline float mavlink_msg_rail_speed_target_get_move_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a rail_speed_target message into a struct
 *
 * @param msg The message to decode
 * @param rail_speed_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_rail_speed_target_decode(const mavlink_message_t* msg, mavlink_rail_speed_target_t* rail_speed_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rail_speed_target->move_speed = mavlink_msg_rail_speed_target_get_move_speed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN? msg->len : MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN;
        memset(rail_speed_target, 0, MAVLINK_MSG_ID_RAIL_SPEED_TARGET_LEN);
    memcpy(rail_speed_target, _MAV_PAYLOAD(msg), len);
#endif
}
