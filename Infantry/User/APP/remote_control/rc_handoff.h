#ifndef RC_HANDOFF_H
#define RC_HANDODD_H
#include "main.h"

#define CH4_NO_ACTION 2
#define CH4_SWITCH_UP 1
#define CH4_SWITCH_DOWN 0

uint8_t rc_ch4_data_process(int16_t ch);

#endif