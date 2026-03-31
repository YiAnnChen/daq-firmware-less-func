#ifndef _SENSOR_H
#define _SENSOR_H

#include <stdlib.h>
#include "main.h"
#include "SEGGER_RTT.h"


// water flow rate
void Sensor_Flow_ProcessPulse(uint32_t capture_value);
float Sensor_Flow_GetRate(void);

// Temperature sensors onewire communication implementation
void DS18B20_WriteByte(UART_HandleTypeDef* huart, uint8_t data);
uint8_t DS18B20_ReadByte(UART_HandleTypeDef* huart);
uint8_t DS18B20_Init(UART_HandleTypeDef* huart);
void DS18B20_SampleTemp(UART_HandleTypeDef* huart);
int DS18B20_ReadTemp(UART_HandleTypeDef* huart);


// 
#endif