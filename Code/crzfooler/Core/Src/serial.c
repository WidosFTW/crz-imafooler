/*
 * serial.c
 *
 *  Created on: May 12, 2025
 *      Author: widos
 */

#include "main.h"
#define TurnOnThreshold 8800
uint8_t TX_TPU_Buffer[6] = { 0 };
uint8_t RX_TPU_Buffer[6] = { 0 };

float BatteryVoltage = 0;
uint8_t synced = 0;

volatile uint16_t VoltTX;

void serial_intercept() {
	uint8_t tmp = 0;
	uint8_t currentByte = 0;
	uint8_t crc = 0;
	uint16_t VoltRx;

	if (!synced) {
		do {
			HAL_UART_Receive(&huart2, &tmp, 1, 10);
		} while (tmp != 0xE6);
		HAL_UART_Transmit(&huart2, &TX_TPU_Buffer[0], 1, HAL_MAX_DELAY);
		TX_TPU_Buffer[0] = tmp;
		currentByte = 1;
	} else {
		currentByte = 0;
	}

	for (uint8_t i = currentByte; i < 6; i++) {
		HAL_UART_Receive(&huart2, &RX_TPU_Buffer[i], 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, &TX_TPU_Buffer[i], 1, HAL_MAX_DELAY);
		TX_TPU_Buffer[i] = RX_TPU_Buffer[i];
	}

	synced++;
	if (synced == 4)
		synced = 0;

	VoltRx = (TX_TPU_Buffer[1] << 8) | TX_TPU_Buffer[2];
	if (VoltRx > TurnOnThreshold) {
		if (PPP_enable) {
			BatteryVoltage = 85.0f;
			HAL_GPIO_WritePin(uC_PPP_En_GPIO_Port, uC_PPP_En_Pin, 1);
		} else {
			BatteryVoltage = 115.0f;
			HAL_GPIO_WritePin(uC_PPP_En_GPIO_Port, uC_PPP_En_Pin, 0);
		}
		VoltTX = (uint16_t) ((BatteryVoltage - 9.55f) / 0.00802f);
		VoltTX &= 0xFF70;
		// Encode voltage back
		TX_TPU_Buffer[1] = (VoltTX >> 8) & 0xFF;
		TX_TPU_Buffer[2] = VoltTX & 0xFF;
	} else {

	}
	// Checksum
	crc = 0;
	for (int i = 0; i < 5; i++) {
		crc += TX_TPU_Buffer[i];
	}
	crc = (~crc + 1) & 0x7F;
	TX_TPU_Buffer[5] = crc;

}
