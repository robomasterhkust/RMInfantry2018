#pragma once
// MESSAGE GIMBAL_ATTITUDE PACKING

#define MAVLINK_MSG_ID_GIMBAL_ATTITUDE 30

MAVPACKED(
typedef struct __mavlink_gimbal_attitude_t {
 float pitch; /*< Pitch angle (rad, -pi..+pi)*/
 float yaw; /*< Yaw angle (rad, -pi..+pi)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
}) mavlink_gimbal_attitude_t;

#define MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN 16
#define MAVLINK_MSG_ID_GIMBAL_ATTITUDE_MIN_LEN 16
#define MAVLINK_MSG_ID_30_LEN 16
#define MAVLINK_MSG_ID_30_MIN_LEN 16

#define MAVLINK_MSG_ID_GIMBAL_ATTITUDE_CRC 120
#define MAVLINK_MSG_ID_30_CRC 120



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_ATTITUDE { \
    30, \
    "GIMBAL_ATTITUDE", \
    4, \
    {  { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_attitude_t, yaw) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_attitude_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_attitude_t, yawspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_ATTITUDE { \
    "GIMBAL_ATTITUDE", \
    4, \
    {  { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_attitude_t, yaw) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_attitude_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_attitude_t, yawspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pitch, float yaw, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_float(buf, 8, pitchspeed);
    _mav_put_float(buf, 12, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN);
#else
    mavlink_gimbal_attitude_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_CRC);
}

/**
 * @brief Pack a gimbal_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pitch,float yaw,float pitchspeed,float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_float(buf, 8, pitchspeed);
    _mav_put_float(buf, 12, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN);
#else
    mavlink_gimbal_attitude_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_CRC);
}

/**
 * @brief Encode a gimbal_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_attitude_t* gimbal_attitude)
{
    return mavlink_msg_gimbal_attitude_pack(system_id, component_id, msg, gimbal_attitude->pitch, gimbal_attitude->yaw, gimbal_attitude->pitchspeed, gimbal_attitude->yawspeed);
}

/**
 * @brief Encode a gimbal_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_attitude_t* gimbal_attitude)
{
    return mavlink_msg_gimbal_attitude_pack_chan(system_id, component_id, chan, msg, gimbal_attitude->pitch, gimbal_attitude->yaw, gimbal_attitude->pitchspeed, gimbal_attitude->yawspeed);
}

/**
 * @brief Send a gimbal_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_attitude_send(mavlink_channel_t chan, float pitch, float yaw, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_float(buf, 8, pitchspeed);
    _mav_put_float(buf, 12, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_ATTITUDE, buf, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_CRC);
#else
    mavlink_gimbal_attitude_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a gimbal_attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_attitude_send_struct(mavlink_channel_t chan, const mavlink_gimbal_attitude_t* gimbal_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_attitude_send(chan, gimbal_attitude->pitch, gimbal_attitude->yaw, gimbal_attitude->pitchspeed, gimbal_attitude->yawspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_ATTITUDE, (const char *)gimbal_attitude, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pitch, float yaw, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_float(buf, 8, pitchspeed);
    _mav_put_float(buf, 12, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_ATTITUDE, buf, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_CRC);
#else
    mavlink_gimbal_attitude_t *packet = (mavlink_gimbal_attitude_t *)msgbuf;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_ATTITUDE UNPACKING


/**
 * @brief Get field pitch from gimbal_attitude message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_gimbal_attitude_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field yaw from gimbal_attitude message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_gimbal_attitude_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitchspeed from gimbal_attitude message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_gimbal_attitude_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yawspeed from gimbal_attitude message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_gimbal_attitude_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a gimbal_attitude message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_attitude_decode(const mavlink_message_t* msg, mavlink_gimbal_attitude_t* gimbal_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_attitude->pitch = mavlink_msg_gimbal_attitude_get_pitch(msg);
    gimbal_attitude->yaw = mavlink_msg_gimbal_attitude_get_yaw(msg);
    gimbal_attitude->pitchspeed = mavlink_msg_gimbal_attitude_get_pitchspeed(msg);
    gimbal_attitude->yawspeed = mavlink_msg_gimbal_attitude_get_yawspeed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN;
        memset(gimbal_attitude, 0, MAVLINK_MSG_ID_GIMBAL_ATTITUDE_LEN);
    memcpy(gimbal_attitude, _MAV_PAYLOAD(msg), len);
#endif
}
