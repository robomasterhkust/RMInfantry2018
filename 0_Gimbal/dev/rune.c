#include "ch.h"
#include "hal.h"

#include "rune.h"
#include "dbus.h"
#include "gimbal.h"
#include "feeder.h"
#include "shoot.h"

static GimbalStruct* gimbal;
static PIMUStruct pIMU;
static bool rune_state = false;

void rune_init(void)
{
  gimbal = gimbal_get();
  pIMU = imu_get();
}

void rune_cmd(uint8_t cmd)
{
  #ifdef RUNE_FIRE_SAFE
    //if()
    //  shooter_control(RUNE_FIRE_POWER);
    //else
    //  shooter_control(0);
  #endif
  rune_state = cmd == DISABLE ? false : true;
  gimbal_setRune(cmd);
}

void rune_fire(const float yaw, const float pitch)
{
  gimbal->pitch_atti_cmd = pitch;
  gimbal->yaw_atti_cmd = yaw + (pIMU->euler_angle[Yaw] - gimbal->d_yaw);

  float pitch_error = gimbal->pitch_atti_cmd - pIMU->euler_angle[Pitch],
        yaw_error   = gimbal->yaw_atti_cmd   - pIMU->euler_angle[Yaw];

  uint8_t count = 0;
  while(count++ < 8)
  {
    if(pitch_error > RUNE_MAX_ERROR || pitch_error < -RUNE_MAX_ERROR ||
       yaw_error > RUNE_MAX_ERROR || yaw_error < -RUNE_MAX_ERROR)
       count = 0;

    pitch_error = gimbal->pitch_atti_cmd - pIMU->euler_angle[Pitch];
    yaw_error   = gimbal->yaw_atti_cmd   - pIMU->euler_angle[Yaw];

    chThdSleepMilliseconds(5);
  }

  feeder_singleShot();
  gimbal_Follow();
}
