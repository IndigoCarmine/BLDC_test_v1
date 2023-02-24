#include "table.h"
#include "dma.h"

#ifndef __3PHASE_PWM_H__
#define __3PHASE_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif


void start_pwm();
void stop_pwm();

//scaler is a value between 0 and 1.
//direction is 0 or 1, 0 is normal, 1 is reverse
void update_table(uint8_t scaler,uint8_t direction);


void set_friqency(int freq);

#ifdef __cplusplus
}   
#endif

#endif