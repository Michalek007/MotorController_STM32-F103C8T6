/*
 * dshot.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Michał
 */

#include "dshot.h"
#include "tim.h"
#include "usart.h"


static uint16_t dshotDmaBuffer[DSHOT_BIT_COUNT] = {700, 700, 300, 700, 300, 700, 300, 700, 300, 700, 300, 700, 300, 700, 300, 700};

void DShot_Init(void){

}

uint16_t DShot_BuildPacket(uint16_t throttle, uint8_t telemetry)
{
	throttle &= 0x7FF; // throttle is 11 bit (2047 max)
    throttle <<= 1; // shift left to make space for telemetry and crc
    throttle |= (0x01 & telemetry);
    uint16_t crc = 0;
    for(uint8_t i=0; i<3; i++){
    	crc ^= (throttle >> (i * 4)) & 0xF;
    }
    return (throttle << 4) | (crc & 0xF);
}

static void DShot_PrepareDmaBuffer(uint16_t dshotPacket)
{
    for (uint8_t i=0; i<DSHOT_BIT_COUNT; i++){
        if (dshotPacket & (1 << (15 - i))){
        	dshotDmaBuffer[i] = (uint16_t)(0.7f * (float)htim2.Instance->ARR);  // logic 1: ~70% of ARR
        }
        else{
        	dshotDmaBuffer[i] = (uint16_t)(0.3f * (float)htim2.Instance->ARR);;  // logic 0: ~30% of ARR
        }
    }
}

void DShot_SendPacket(uint16_t throttle, uint8_t telemetry){
	uint16_t packet = DShot_BuildPacket(throttle, telemetry);
	DShot_PrepareDmaBuffer(packet);
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)dshotDmaBuffer, DSHOT_BIT_COUNT);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    	UART_Print("Done DMA PWM\n");
    }
}

void DShot_Deserialize(DShotPacket *dshotPacket, uint8_t *data, uint8_t size){
	if (size != DSHOT_BIT_COUNT){
		return;
	}
	uint16_t dshotData = 0;
	for (uint8_t i=0; i<size; i++){
		if (data[i] >= 60){
			dshotData |= (1<<(15-i));
		}
	}
	dshotPacket->throttle = (dshotData >> 5) & 0x7FF;
	dshotPacket->telemetry = (dshotData >> 4) & 0x01;
	dshotPacket->crc = dshotData & 0xF;
}

uint8_t DShot_ValidateCrc(DShotPacket *dshotPacket){
	uint16_t data = (dshotPacket->throttle << 1) | dshotPacket->telemetry;
    uint16_t crc = 0;
    for(uint8_t i=0; i<3; i++){
    	crc ^= (data >> (i * 4)) & 0xF;
    }
    if (crc == dshotPacket->crc){
    	return 1;
    }
    return 0;
}
