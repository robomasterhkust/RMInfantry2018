#pragma once
// MESSAGE CHASSIS_ATTITUDE PACKING

#define MAVLINK_MSG_ID_CHASSIS_ATTITUDE 31

MAVPACKED(
typedef struct __mavlink_chassis_attitude_t {
 float x; /*< */
 float y; /*< */
 float z; /*< */
 float vx; /*< */
 float vy; /*< */
 float vz; /*< */
}) mavlink_chassis_attitude_t;

#define MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN 24
#define MAVLINK_MSG_ID_CHASSIS_ATTITUDE_MIN_LEN 24
#define MAVLINK_MSG_ID_31_LEN 24
#define MAVLINK_MSG_ID_31_MIN_LEN 24

#define MAVLINK_MSG_ID_CHASSIS_ATTITUDE_CRC 122
#define MAVLINK_MSG_ID_31_CRC 122



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHASSIS_ATTITUDE { \
    31, \
    "CHASSIS_ATTITUDE", \
    6, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chassis_attitude_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chassis_attitude_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_chassis_attitude_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_chassis_attitude_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_chassis_attitude_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_chassis_attitude_t, vz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHASSIS_ATTITUDE { \
    "CHASSIS_ATTITUDE", \
    6, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chassis_attitude_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chassis_attitude_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_chassis_attitude_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_chassis_attitude_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_chassis_attitude_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_chassis_attitude_t, vz) }, \
         } \
}
#endif

/**
 * @brief Pack a chassis_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x 
 * @param y 
 * @param z 
 * @param vx 
 * @param vy 
 * @param vz 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN);
#else
    mavlink_chassis_attitude_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHASSIS_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_CRC);
}

/**
 * @brief Pack a chassis_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x 
 * @param y 
 * @param z 
 * @param vx 
 * @param vy 
 * @param vz 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,float vx,float vy,float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN);
#else
    mavlink_chassis_attitude_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHASSIS_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_CRC);
}

/**
 * @brief Encode a chassis_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chassis_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chassis_attitude_t* chassis_attitude)
{
    return mavlink_msg_chassis_attitude_pack(system_id, component_id, msg, chassis_attitude->x, chassis_attitude->y, chassis_attitude->z, chassis_attitude->vx, chassis_attitude->vy, chassis_attitude->vz);
}

/**
 * @brief Encode a chassis_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chassis_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chassis_attitude_t* chassis_attitude)
{
    return mavlink_msg_chassis_attitude_pack_chan(system_id, component_id, chan, msg, chassis_attitude->x, chassis_attitude->y, chassis_attitude->z, chassis_attitude->vx, chassis_attitude->vy, chassis_attitude->vz);
}

/**
 * @brief Send a chassis_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param x 
 * @param y 
 * @param z 
 * @param vx 
 * @param vy 
 * @param vz 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chassis_attitude_send(mavlink_channel_t chan, float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_ATTITUDE, buf, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_CRC);
#else
    mavlink_chassis_attitude_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a chassis_attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chassis_attitude_send_struct(mavlink_channel_t chan, const mavlink_chassis_attitude_t* chassis_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chassis_attitude_send(chan, chassis_attitude->x, chassis_attitude->y, chassis_attitude->z, chassis_attitude->vx, chassis_attitude->vy, chassis_attitude->vz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_ATTITUDE, (const char *)chassis_attitude, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chassis_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_ATTITUDE, buf, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_CRC);
#else
    mavlink_chassis_attitude_t *packet = (mavlink_chassis_attitude_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE CHASSIS_ATTITUDE UNPACKING


/**
 * @brief Get field x from chassis_attitude message
 *
 * @return 
 */
static inline float mavlink_msg_chassis_attitude_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from chassis_attitude message
 *
 * @return 
 */
static inline float mavlink_msg_chassis_attitude_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from chassis_attitude message
 *
 * @return 
 */
static inline float mavlink_msg_chassis_attitude_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vx from chassis_attitude message
 *
 * @return 
 */
static inline float mavlink_msg_chassis_attitude_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vy from chassis_attitude message
 *
 * @return 
 */
static inline float mavlink_msg_chassis_attitude_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vz from chassis_attitude message
 *
 * @return 
 */
static inline float mavlink_msg_chassis_attitude_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a chassis_attitude message into a struct
 *
 * @param msg The message to decode
 * @param chassis_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_chassis_attitude_decode(const mavlink_message_t* msg, mavlink_chassis_attitude_t* chassis_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    chassis_attitude->x = mavlink_msg_chassis_attitude_get_x(msg);
    chassis_attitude->y = mavlink_msg_chassis_attitude_get_y(msg);
    chassis_attitude->z = mavlink_msg_chassis_attitude_get_z(msg);
    chassis_attitude->vx = mavlink_msg_chassis_attitude_get_vx(msg);
    chassis_attitude->vy = mavlink_msg_chassis_attitude_get_vy(msg);
    chassis_attitude->vz = mavlink_msg_chassis_attitude_get_vz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN? msg->len : MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN;
        memset(chassis_attitude, 0, MAVLINK_MSG_ID_CHASSIS_ATTITUDE_LEN);
    memcpy(chassis_attitude, _MAV_PAYLOAD(msg), len);
#endif
}
