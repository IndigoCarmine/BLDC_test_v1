//ref: A study on Simplified Position Sensorless Control of Permanent Magnet Synchronous Moter 
// Satoshi SUMITA

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "table.h"


struct CurrentVector{
    int Idc;
    int Iqc;

};

extern ADC_HandleTypeDef hdac2;
extern TIM_HandleTypeDef htim1;



CurrentVector get_current_vector(void){
    //get current from ADC
    uint8_t ad = HAL_ADC_GetValue(&hadc2);

    //get voltage from Tim1 DMA
    uint16_t Vdc = sine_table[htim1.Instance->CNT+U_POINT]-sine_table[htim1.Instance->CNT + V_POINT];
    uint16_t Vqc = sine_table[htim1.Instance->CNT+U_POINT+V_POINT/4]-sine_table[htim1.Instance->CNT + V_POINT+V_POINT/4];
    
    //calculate current vector
    CurrentVector cv{(2/3)*ad*Vdc, (2/3)*ad*Vqc};

    return cv;
    
}

#ifdef __cplusplus
}
#endif

#endif
