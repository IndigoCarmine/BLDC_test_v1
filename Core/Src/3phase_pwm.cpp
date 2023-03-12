#include "3phase_pwm.hpp"

#include "table.h"
#include "dma.h"

uint8_t A_table[TABLE_SIZE]={0};
uint8_t B_table[TABLE_SIZE]={0};

uint8_t using_A_table = 0;


void start_pwm(TIM_HandleTypeDef &htim)
{
    //start PWM
	HAL_TIM_PWM_Start_DMA(&htim, TIM_CHANNEL_1, (uint32_t*)(A_table+U_POINT), WAVE_SIZE);
    htim.State = HAL_TIM_STATE_READY;
    HAL_TIM_PWM_Start_DMA(&htim, TIM_CHANNEL_2, (uint32_t*)(A_table+V_POINT), WAVE_SIZE);
    htim.State = HAL_TIM_STATE_READY;
    HAL_TIM_PWM_Start_DMA(&htim, TIM_CHANNEL_3,(uint32_t*)( A_table+W_POINT), WAVE_SIZE);
    using_A_table = 1;
}

void stop_pwm(TIM_HandleTypeDef &htim)
{
    //stop PWM
    HAL_TIM_PWM_Stop_DMA(&htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_DMA(&htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop_DMA(&htim, TIM_CHANNEL_3);
}



//scaler is a value between 0 and 1.
//table is the table to update.
//direction is 0 or 1, 0 is normal, 1 is reverse
void unsafe_update_table(float scaler, uint8_t* table,TIM_HandleTypeDef* htim,uint8_t direction)
{
    //generate new table
    for(int i=0; i<TABLE_SIZE; i++){
        table[i] = sine_table[i] * scaler;
    }
    //change DMA to use new table
    if(direction == 0){
        PWM_DMA_Change(htim, TIM_CHANNEL_1, (uint32_t*)(table + U_POINT), WAVE_SIZE);
        PWM_DMA_Change(htim, TIM_CHANNEL_2, (uint32_t*)(table + V_POINT), WAVE_SIZE);
        PWM_DMA_Change(htim, TIM_CHANNEL_3, (uint32_t*)(table + W_POINT), WAVE_SIZE);
    }else{
        PWM_DMA_Change(htim, TIM_CHANNEL_1,(uint32_t*)( table + V_POINT), WAVE_SIZE);
        PWM_DMA_Change(htim, TIM_CHANNEL_2, (uint32_t*)(table + U_POINT), WAVE_SIZE);
        PWM_DMA_Change(htim, TIM_CHANNEL_3, (uint32_t*)(table + W_POINT), WAVE_SIZE);
    }
}


//scaler is a value between 0 and 1.
//direction is 0 or 1, 0 is normal, 1 is reverse
void update_table(uint8_t scaler,uint8_t direction,TIM_HandleTypeDef &htim)
{
    if(using_A_table){
        //table A is being used, so update table B
        unsafe_update_table(scaler, B_table, &htim,direction);
        using_A_table = 0;
    }else{
        //table B is being used, so update table A
        unsafe_update_table(scaler, A_table, &htim,direction);
        using_A_table = 1;
    }
}

void set_friqency(int freq,TIM_HandleTypeDef &htim){
	//170MHz
	__HAL_TIM_SET_PRESCALER(&htim, 1000000/180*16/freq);
}
