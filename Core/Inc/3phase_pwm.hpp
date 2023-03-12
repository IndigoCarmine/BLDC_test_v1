#pragma once

#include "table.h"
#include "dma.h"



void start_pwm(TIM_HandleTypeDef &htim);
void stop_pwm(TIM_HandleTypeDef &htim);

//scaler is a value between 0 and 1.
//direction is 0 or 1, 0 is normal, 1 is reverse
void update_table(uint8_t scaler,uint8_t direction);


void set_friqency(int freq, TIM_HandleTypeDef &htim);


