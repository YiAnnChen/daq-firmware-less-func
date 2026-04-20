/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <stdbool.h>
#include <string.h>
#include "can_addr_def.h"
#include "can_log.h"

//received voltage storage
volatile uint16_t g_inv_left_voltage = 0; //unit: 0.1V, from 0x10
volatile uint16_t g_inv_right_voltage = 0;//unit: 0.1V, from 0x11
volatile uint16_t g_bms_voltage = 0; //unit: 0.1V, from 0x40

//set true once at least one voltage data arrives 
volatile bool g_voltage_received = false;

//DAQ outgoing buffer and enable flag
uint8_t DAQData_to_DataLogger[8]={0};
volatile bool g_daq_enabled = true;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void) {
  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 24;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN1) {
    /* USER CODE BEGIN CAN1_MspInit 0 */

    /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2(); // Remap CAN1 to PB8/PB9

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    /* USER CODE BEGIN CAN1_MspInit 1 */

    /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle) {
  if (canHandle->Instance == CAN1) {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */

    /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    /* USER CODE BEGIN CAN1:USB_HP_CAN1_TX_IRQn disable */
    /**
     * Uncomment the line below to disable the "USB_HP_CAN1_TX_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn); */
    /* USER CODE END CAN1:USB_HP_CAN1_TX_IRQn disable */

    /* USER CODE BEGIN CAN1:USB_LP_CAN1_RX0_IRQn disable */
    /**
     * Uncomment the line below to disable the "USB_LP_CAN1_RX0_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn); */
    /* USER CODE END CAN1:USB_LP_CAN1_RX0_IRQn disable */

    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    /* USER CODE BEGIN CAN1_MspDeInit 1 */

    /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t bsp_can1_filter_config(void) {
  CAN_FilterTypeDef f = {0};

  //Bank 0: INV + BMS voltages
  f.FilterBank = 0;
  f.FilterActivation = ENABLE;
  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  f.FilterMode = CAN_FILTERMODE_IDLIST;
  f.FilterScale = CAN_FILTERSCALE_16BIT;
  f.FilterIdHigh = (CA_INV_LEFT_VOLTAGE << 5) ; // Standard ID is left-aligned in the 16-bit filter register
  f.FilterIdLow = (CA_INV_RIGHT_VOLTAGE << 5);
  f.FilterMaskIdHigh = (CA_BMS_DATA1 << 5);
  f.FilterMaskIdLow = (CA_BMS_DATA2 << 5);
  if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK) return 0;

  //Bank 1: DAQ enable
  f.FilterBank = 1;
  f.FilterIdHigh = (CA_DAQ_EN << 5);
  f.FilterIdLow      = (CA_DAQ_EN << 5);
  f.FilterMaskIdHigh = (CA_DAQ_EN << 5);
  f.FilterMaskIdLow  = (CA_DAQ_EN << 5);
  return (HAL_CAN_ConfigFilter(&hcan, &f) == HAL_OK);
}

void CAN_SendMsg(uint16_t msgID, uint8_t* Data) {
  CAN_TxHeaderTypeDef TxHeader = {0};
  TxHeader.StdId = msgID;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  uint32_t TxMailbox;
  uint32_t t0 = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < 1) {
    if (HAL_GetTick() - t0 > 10) {
      // Timeout after 10ms
      return;
    }
    (void)HAL_CAN_AddTxMessage(&hcan, &TxHeader, Data, &TxMailbox); // Attempt to add message to mailbox
  }
}

static CAN_RxHeaderTypeDef sRxHeader;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  uint8_t data[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &sRxHeader, data) != HAL_OK) return;
  if (sRxHeader.IDE != CAN_ID_STD ) return;
  
  switch((uint16_t) sRxHeader.StdId){
    case CA_INV_LEFT_VOLTAGE:
      g_inv_left_voltage = ((uint16_t)data[0] << 8) | data[1];
      g_voltage_received = true;
      SEGGER_RTT_printf(0, "[CAN] INV_L = %d (x0.1V)\n", g_inv_left_voltage);
      break;
    case CA_INV_RIGHT_VOLTAGE:
      g_inv_right_voltage = ((uint16_t)data[0] << 8) | data[1];
      g_voltage_received = true;
      SEGGER_RTT_printf(0, "[CAN] INV_R = %d (x0.1V)\n", g_inv_right_voltage);
      break;
    case CA_BMS_DATA1:
      g_bms_voltage = ((uint16_t)data[0] << 8) | data[1];
      g_voltage_received = true;
      SEGGER_RTT_printf(0, "[CAN] BMS   = %d (x0.1V)\n", g_bms_voltage);
      break;
    case CA_BMS_DATA2:
      //  Reserved for future BMS fields
       break;
    case CA_DAQ_EN:
      g_daq_enabled = (data[CA_DAQ_EN_IDX] != 0);
      SEGGER_RTT_printf(0, "[CAN] DAQ_EN = %d\n", (int)g_daq_enabled);
      break;

    default:
       break;
  }

 }
/* USER CODE END 1 */
