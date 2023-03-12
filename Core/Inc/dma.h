/*
 * dma.h
 *
 *  Created on: Feb 9, 2023
 *      Author: taman
 */
#pragma once

#include "main.h"

//強制的にPWMのDMAを利用し、さらに強制的にメモリーのポインタを変更しています。
//HALを利用していないため、他の関数との競合が発生する可能性があります。このコードをリファクタリングする人、ごめんなさい。
void PWM_DMA_Change(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);

