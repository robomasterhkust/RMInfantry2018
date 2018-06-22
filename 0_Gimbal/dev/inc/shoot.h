#ifndef _SHOOT_H_
#define _SHOOT_H_

#define SHOOTER_USE_RC
// #define SHOOTER_SETUP

void pwm9_setWidth(uint16_t width);

// void pwm12_setWidth(uint16_t width);

void shooter_control(uint16_t setpoint);
void shooter_start(void);

typedef struct
{
	uint8_t fast_speed;
	uint8_t slow_speed;
	uint8_t stop;
}speed_mode_t;

#endif
