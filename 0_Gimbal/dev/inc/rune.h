#ifndef _RUNE_H_
#define _RUNE_H_

#define RUNE_FIRE_SAFE
#define RUNE_FIRE_POWER 160U

#define RUNE_MAX_ERROR 0.03f

#ifdef __cplusplus
extern "C" {
#endif

void rune_init(void);
void rune_cmd(uint8_t cmd);
void rune_fire(const float yaw, const float pitch);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //_RUNE_H_
