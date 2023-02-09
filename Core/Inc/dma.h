/*
 * dma.h
 *
 *  Created on: Feb 9, 2023
 *      Author: taman
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#include "main.h"

//WETな関数です。すいません。
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
//  /* Clear the DMAMUX synchro overrun flag */
//  hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;
//
//  if (hdma->DMAmuxRequestGen != 0U)
//  {
//    /* Clear the DMAMUX request generator overrun flag */
//    hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
//  }
//
//  /* Clear all flags */
//  hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1FU));

  /* Configure DMA Channel data length */
  hdma->Instance->CNDTR = DataLength;

  /* Memory to Peripheral */
  if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Channel destination address */
    hdma->Instance->CPAR = DstAddress;

    /* Configure DMA Channel source address */
    hdma->Instance->CMAR = SrcAddress;
  }
  /* Peripheral to Memory */
  else
  {
    /* Configure DMA Channel source address */
    hdma->Instance->CPAR = SrcAddress;

    /* Configure DMA Channel destination address */
    hdma->Instance->CMAR = DstAddress;
  }
}



//強制的にPWMのDMAを利用し、さらに強制的にメモリーのポインタを変更しています。(現状はChannel1用です。)
void PWM_DMA_Start(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length){
	__HAL_LOCK(htim->hdma[TIM_DMA_ID_CC1]);
//    __HAL_DMA_DISABLE(htim->hdma[TIM_DMA_ID_CC1]);
    DMA_SetConfig(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1,Length);
//    __HAL_DMA_ENABLE(htim->hdma[TIM_DMA_ID_CC1]);
//	__HAL_UNLOCK(htim->hdma[TIM_DMA_ID_CC1]);
}

//TODO:HALを利用することなく最適化する必要があるが、応急的に利用します。
void PWM_DMA_Start_Old(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length){
	HAL_TIM_PWM_Start_DMA(htim,Channel,pData, Length);
	htim->State = HAL_TIM_STATE_READY;
	htim->hdma[TIM_DMA_ID_CC1]->State = HAL_DMA_STATE_READY;
	htim->ChannelState[TIM_CHANNEL_1]= HAL_TIM_CHANNEL_STATE_READY;
}

#endif /* INC_DMA_H_ */
