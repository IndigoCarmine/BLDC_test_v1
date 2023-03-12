//ref: A study on Simplified Position Sensorless Control of Permanent Magnet Synchronous Moter 
// Satoshi SUMITA

#pragma once


#include "main.h"
#include "table.h"
#include "data.h"


//get valtage from ADC, and convert to current
//Attention: this function cannot use now, because the shunt resistor is not determined.
//TODO: #1 determine the shunt resistor...
float get_current(ADC_HandleTypeDef &hadc){
    //get current from ADC
    uint8_t ad = HAL_ADC_GetValue(&hadc);
    //3.3v is the reference voltage
    //4096 is the max value of ADC (12bit max)
    return ad*3.3/4096;
}




//get current vector from ADC and sine table
//ref: A study on Simplified Position Sensorless Control of Permanent Magnet Synchronous Moter
Vector2D get_current_vector(ADC_HandleTypeDef &hadc, TIM_HandleTypeDef &pwm_timer){
    //get current.
    float ad = get_current(hadc);

    //get voltage from Tim1 DMA
    uint16_t Vdc = sine_table[pwm_timer.Instance->CNT+U_POINT]-sine_table[pwm_timer.Instance->CNT + V_POINT];
    uint16_t Vqc = sine_table[pwm_timer.Instance->CNT+U_POINT+V_POINT/4]-sine_table[pwm_timer.Instance->CNT + V_POINT+V_POINT/4];
    
    //calculate current vector
    Vector2D cv{(2/3)*ad*Vdc, (2/3)*ad*Vqc};

    return cv;
    
}

