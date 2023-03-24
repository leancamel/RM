#ifndef RC_HANDOFF_H
#define RC_HANDODD_H
#include "main.h"

uint8_t rc_ch4_data_process(int16_t ch);
bool_t switch_is_fric_on(uint16_t ch);
bool_t switch_is_shoot(uint16_t ch);

#endif