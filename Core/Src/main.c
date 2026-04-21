/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"

#include "can.h"
#include "gpio.h"
#include "tim.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

#include "SEGGER_RTT.h"
#include "Sensors.h"
#include "can_addr_def.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_STEP_PCT 1              // percentage increase per step (1 %)
#define PWM_STEP_INTERVAL_MS 100    // ms between steps  -> 100 steps x 100ms = 10 s total ramp
#define DAQ_REPORT_INTERVAL_MS 500  // how often to broadcast DAQ_DATA frame
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint16_t g_inv_left_voltage;
extern volatile uint16_t g_inv_right_voltage;
extern volatile uint16_t g_bms_voltage;
extern volatile bool g_voltage_received;
extern volatile bool g_daq_enabled;
extern uint8_t DAQData_to_DataLogger[8];
extern uint8_t FAN_PWM;
extern uint8_t PUMP_PWM;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DAQ_SendReport(void) {
  DAQData_to_DataLogger[CA_DAQ_PRE_TEMP] = 0;
  DAQData_to_DataLogger[CA_DAQ_POST_TEMP] = 0;
  DAQData_to_DataLogger[CA_DAQ_PRESSURE_H] = 0;
  DAQData_to_DataLogger[CA_DAQ_PRESSURE_L] = 0;
  DAQData_to_DataLogger[CA_DAQ_FLOW_RATE_H] = 0;
  DAQData_to_DataLogger[CA_DAQ_FLOW_RATE_L] = 0;
  DAQData_to_DataLogger[CA_DAQ_FAN_PWM] = FAN_PWM;
  DAQData_to_DataLogger[CA_DAQ_PUMP_PWM] = PUMP_PWM;

  if (g_daq_enabled) {
    CAN_SendMsg(CA_DAQ_DATA, DAQData_to_DataLogger);
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SEGGER_RTT_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  SEGGER_RTT_printf(0, "DAQ start\n");

  bsp_can1_filter_config();
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // start RX interrupt

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // PUMP
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // FAN

  SetFanDuty(10);
  SetPumpDuty(10);

  // MODE 1: Wait for at least one voltage frame from inverter / BMS
  SEGGER_RTT_printf(0, "DAQ waiting for voltage frames...\n");
  while (!g_voltage_received) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // blink LED while waiting
    HAL_Delay(200);
  }
  SEGGER_RTT_printf(0, "DAQ voltage received! INV_L=%d INV_R=%d BMS=%d (x0.1V)\n", g_inv_left_voltage,
                    g_inv_right_voltage, g_bms_voltage);

  // MODE 2: Ramp PWM from 0 % to 100 %
  SEGGER_RTT_printf(0, "DAQ starting PWM ramp...\n");

  uint8_t current_pwm = 10;
  uint32_t last_step_tick = HAL_GetTick();
  uint32_t last_rpt_tick = HAL_GetTick();

  while (current_pwm < 100) {
    uint32_t now = HAL_GetTick();

    // Increment PWM one step every PWM_STEP_INTERVAL_MS
    if (now - last_step_tick >= PWM_STEP_INTERVAL_MS) {
      current_pwm += PWM_STEP_PCT;
      if (current_pwm > 100) current_pwm = 100;
      SetFanDuty(current_pwm);
      SetPumpDuty(current_pwm);
      last_step_tick = now;
      SEGGER_RTT_printf(0, "DAQ PWM updated: %d%%\n", current_pwm);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    // Report over CAN at a slower rate during ramp
    if (now - last_rpt_tick >= DAQ_REPORT_INTERVAL_MS) {
      DAQ_SendReport();
      last_rpt_tick = now;
    }
  }
  SEGGER_RTT_printf(0, "DAQ ramp complete – PWM at 100%%\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_rpt_tick2 = HAL_GetTick();
  while (1) {
    /* USER CODE END WHILE */
    uint32_t now = HAL_GetTick();
    if (now - last_rpt_tick2 >= DAQ_REPORT_INTERVAL_MS) {
      DAQ_SendReport();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      SEGGER_RTT_printf(0, "DAQ INV_L=%d INV_R=%d BMS=%d FAN=%d%% PUMP=%d%%\n", g_inv_left_voltage,
                        g_inv_right_voltage, g_bms_voltage, FAN_PWM, PUMP_PWM);
      last_rpt_tick2 = now;
    }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
