#include "3phase_pwm.h"

#include "table.h"
#include "dma.h"

uint8_t A_table[TABLE_SIZE]={0};
uint8_t B_table[TABLE_SIZE]={0};

bool using_A_table = false;

extern TIM_HandleTypeDef htim1;


void start_pwm()
{
    //start PWM
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, A_table+U_POINT, WAVE_SIZE);
    htim1.State = HAL_TIM_STATE_READY;
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, A_table+V_POINT, WAVE_SIZE);
    htim1.State = HAL_TIM_STATE_READY;
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, A_table+W_POINT, WAVE_SIZE);
    using_A_table = true;
}

void stop_pwm()
{
    //stop PWM
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
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
        PWM_DMA_Change(htim, TIM_CHANNEL_1, table + U_POINT, WAVE_SIZE);
        PWM_DMA_Change(htim, TIM_CHANNEL_2, table + V_POINT, WAVE_SIZE);
        PWM_DMA_Change(htim, TIM_CHANNEL_3, table + W_POINT, WAVE_SIZE);
    }else{
        PWM_DMA_Change(htim, TIM_CHANNEL_1, table + V_POINT, WAVE_SIZE);
        PWM_DMA_Change(htim, TIM_CHANNEL_2, table + U_POINT, WAVE_SIZE);
        PWM_DMA_Change(htim, TIM_CHANNEL_3, table + W_POINT, WAVE_SIZE);
    }
}


//scaler is a value between 0 and 1.
//direction is 0 or 1, 0 is normal, 1 is reverse
void update_table(uint8_t scaler,uint8_t direction)
{
    if(using_A_table){
        //table A is being used, so update table B
        unsafe_update_table(scaler, B_table, &htim1,direction);
        using_A_table = false;
    }else{
        //table B is being used, so update table A
        unsafe_update_table(scaler, A_table, &htim1,direction);
        using_A_table = true;
    }
}

void set_friqency(int freq){
	//170MHz
	__HAL_TIM_SET_PRESCALER(&htim1, 1000000/180*16/freq);
}