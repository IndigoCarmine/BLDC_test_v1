#include "table.h"
#include "dma.h"

uint8_t A_table[427]={0};
uint8_t B_table[427]={0};

bool using_A_table = false;

extern TIM_HandleTypeDef htim1;


void start_pwm()
{
    //start PWM
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, A_table, TABLE_SIZE);
    htim1.State = HAL_TIM_STATE_READY;
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, A_table, TABLE_SIZE);
    htim1.State = HAL_TIM_STATE_READY;
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, A_table, TABLE_SIZE);
    using_A_table = true;
}

void stop_pwm()
{
    //stop PWM
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
}




void update_table(uint8_t scalar)
{
    if(using_A_table){
        //table A is being used, so update table B
        for(int i=0; i<TABLE_SIZE; i++){
            B_table[i] = sine_table[i] * scalar;
        }
        using_A_table = false;
        //change DMA to use table B
        PWM_DMA_Change(&htim1, TIM_CHANNEL_1, B_table + U_POINT, TABLE_SIZE);
        PWM_DMA_Change(&htim1, TIM_CHANNEL_2, B_table + V_POINT, TABLE_SIZE);
        PWM_DMA_Change(&htim1, TIM_CHANNEL_3, B_table + W_POINT, TABLE_SIZE);
    }else{
        //table B is being used, so update table A
        for(int i=0; i<TABLE_SIZE; i++){
            A_table[i] = sine_table[i] * scalar;
        }
        using_A_table = true;
        //change DMA to use table A
        PWM_DMA_Change(&htim1, TIM_CHANNEL_1, A_table + U_POINT, TABLE_SIZE);
        PWM_DMA_Change(&htim1, TIM_CHANNEL_2, A_table + V_POINT, TABLE_SIZE);
        PWM_DMA_Change(&htim1, TIM_CHANNEL_3, A_table + W_POINT, TABLE_SIZE);
    }
}


