/** @file
 *  @brief MAVLink comm protocol generated from sentry.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_SENTRY_H
#define MAVLINK_SENTRY_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_SENTRY.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 138, 8, 0, 0, 0}, {30, 120, 16, 0, 0, 0}, {31, 122, 24, 0, 0, 0}, {32, 50, 4, 0, 0, 0}, {34, 138, 8, 0, 0, 0}, {35, 107, 4, 0, 0, 0}, {36, 53, 4, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_SENTRY

// ENUM DEFINITIONS


/** @brief  */
#ifndef HAVE_ENUM_ROBO_TYPE
#define HAVE_ENUM_ROBO_TYPE
typedef enum ROBO_TYPE
{
   ROBO_TYPE_SOLDIER=0, /*  | */
   ROBO_TYPE_SENTRY=1, /*  | */
   ROBO_TYPE_ENGINEER=2, /*  | */
   ROBO_TYPE_HERO=3, /*  | */
   ROBO_TYPE_DRONE=4, /*  | */
   ROBO_TYPE_ENUM_END=5, /*  | */
} ROBO_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_ROBO_STATE
#define HAVE_ENUM_ROBO_STATE
typedef enum ROBO_STATE
{
   ROBO_STATE_UNINIT=0, /* Uninitialized system, state is unknown. | */
   ROBO_STATE_BOOT=1, /* System is booting up. | */
   ROBO_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
   ROBO_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
   ROBO_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
   ROBO_STATE_CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
   ROBO_STATE_EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
   ROBO_STATE_POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
   ROBO_STATE_ENUM_END=8, /*  | */
} ROBO_STATE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_heartbeat.h"
#include "./mavlink_msg_gimbal_attitude.h"
#include "./mavlink_msg_chassis_attitude.h"
#include "./mavlink_msg_rail_position.h"
#include "./mavlink_msg_gimbal_speed_target.h"
#include "./mavlink_msg_rail_position_target.h"
#include "./mavlink_msg_rail_speed_target.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_GIMBAL_ATTITUDE, MAVLINK_MESSAGE_INFO_CHASSIS_ATTITUDE, MAVLINK_MESSAGE_INFO_RAIL_POSITION, MAVLINK_MESSAGE_INFO_GIMBAL_SPEED_TARGET, MAVLINK_MESSAGE_INFO_RAIL_POSITION_TARGET, MAVLINK_MESSAGE_INFO_RAIL_SPEED_TARGET}
# define MAVLINK_MESSAGE_NAMES {{ "CHASSIS_ATTITUDE", 31 }, { "GIMBAL_ATTITUDE", 30 }, { "GIMBAL_SPEED_TARGET", 34 }, { "HEARTBEAT", 0 }, { "RAIL_POSITION", 32 }, { "RAIL_POSITION_TARGET", 35 }, { "RAIL_SPEED_TARGET", 36 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_SENTRY_H
