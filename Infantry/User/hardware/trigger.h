#ifndef TRIGGER_H
#define TRIGGER_H
#include "main.h"

#define FRIC_UP 1580
#define FRIC_DOWN 1500
#define FRIC_OFF 1000

void trigger_PWM_Init(void);
void fric_off(void);
void fric1_on(uint16_t cmd);
void fric2_on(uint16_t cmd);

#endif
