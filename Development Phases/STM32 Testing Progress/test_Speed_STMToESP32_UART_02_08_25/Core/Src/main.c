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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h" // For strlen
#include "stdio.h"  // For sprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim5; // NEW: Timer handle for the optical sensor

UART_HandleTypeDef huart2; // For PC Putty
UART_HandleTypeDef huart3; // For ESP32

/* USER CODE BEGIN PV */
// UART message for status
char *uart_startup_msg = "STM32F407: CAN Receiver & UART Bridge to ESP32 (Multi-Sensor)...\r\n";
char uart_buffer[120]; // General purpose buffer for UART prints (for Putty)
char esp32_data_buffer[150]; // Buffer for data sent to ESP32 (expanded for new data)

// CAN message variables for reception
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8]; // Buffer for received data

// Global flags for status/errors (optional, for main loop interaction)
volatile uint8_t can_rx_flag = 0;
volatile uint32_t can_error_flags = 0;

// Variables to store received sensor data
float received_temperature = 0.0f;
float received_humidity = 0.0f;
uint8_t received_ir_state = 0; // 0: No obstacle, 1: Obstacle
uint16_t received_sound_value = 0; // 0-1023 analog value
uint8_t received_sequence_num = 0;

// Thresholds for alerts (same as before)
#define TEMP_ALERT_THRESHOLD 30.0f
#define SOUND_ALERT_THRESHOLD 520
#define IR_OBSTACLE_DETECTED 1

// --- NEW VARIABLES FOR OPTICAL SENSOR ---
// Speed, Acceleration, RPM and Idling time
volatile float current_speed_kmh = 0.0f;
volatile float previous_speed_kmh = 0.0f;
volatile float acceleration_mps2 = 0.0f;
volatile uint32_t current_rpm = 0;
volatile uint32_t idling_timer_ms = 0;
volatile uint32_t last_pulse_time_ms = 0;
volatile uint32_t last_speed_update_time = 0;

// Constants for calculations
#define PULSES_PER_REVOLUTION 20 // Assumed pulses per revolution for the H2010 sensor
#define WHEEL_DIAMETER_CM 6.5f   // Assumed wheel diameter in cm
#define SECONDS_PER_IDLE_CHECK 10.0f // Check for idling every 10 seconds

// Input capture variables
volatile uint32_t ic_val1 = 0;
volatile uint32_t ic_val2 = 0;
volatile uint32_t difference = 0;
volatile uint8_t is_first_captured = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM5_Init(void); // NEW: Timer Init
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function to send string over UART (for PC Putty - USART2)
void UART_Transmit_String(UART_HandleTypeDef *huart, char *str)
{
    HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

// Function to send string over UART to ESP32 (USART3)
void UART_Transmit_To_ESP32(char *str)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

// Function to toggle an LED
void Toggle_LED(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}

// Function to print raw CAN data to UART (for debugging)
void PrintRawCanData(uint8_t *data, uint8_t len)
{
    char temp_str[10];
    UART_Transmit_String(&huart2, "Raw Data: ");
    for (int i = 0; i < len; i++)
    {
        sprintf(temp_str, "0x%02X ", data[i]);
        UART_Transmit_String(&huart2, temp_str);
    }
    UART_Transmit_String(&huart2, "\r\n");
}

// --- NEW: Timer Input Capture Callback ---
/**
  * @brief  Input Capture callback.
  * @param  htim pointer to a TIM_HandleTypeDef structure.
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (is_first_captured == 0)
        {
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            is_first_captured = 1;
        }
        else if (is_first_captured == 1)
        {
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            // Calculate the time difference (in timer ticks)
            if (ic_val2 > ic_val1)
            {
                difference = ic_val2 - ic_val1;
            }
            else
            {
                difference = (0xFFFFFFFF - ic_val1) + ic_val2;
            }

            // Frequency (Hz) = Timer Clock / (Prescaler+1) / Difference
            // In our case, Timer Clock / (Prescaler+1) = 96MHz / (9599+1) = 10kHz
            // so 1 tick = 0.1ms
            float pulse_interval_ms = (float)difference / 10.0f;

            // Update RPM if the interval is not too long (i.e., not stopped)
            if (pulse_interval_ms < 500.0f) // A reasonable threshold to avoid false readings
            {
                current_rpm = (uint32_t)(60000.0f / (PULSES_PER_REVOLUTION * pulse_interval_ms));
            }
            else
            {
                current_rpm = 0;
            }

            // Calculate speed in km/h from RPM
            // Circumference (m) = pi * diameter (m)
            // Speed (m/s) = RPM * Circumference (m) / 60
            // Speed (km/h) = Speed (m/s) * 3.6
            float wheel_circumference_m = (float)3.14159f * (WHEEL_DIAMETER_CM / 100.0f);
            current_speed_kmh = ((float)current_rpm * wheel_circumference_m / 60.0f) * 3.6f;

            // Reset for next capture
            is_first_captured = 0;
            ic_val1 = 0;
            ic_val2 = 0;
        }
    }
}


// ====================================================================
// CAN Interrupt Callback Implementations
// ====================================================================

/**
  * @brief  Rx FIFO 0 Message Pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  * the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // Process the received message
            sprintf(uart_buffer, "CAN Rx Int! ID: 0x%lX, DLC: %lu. ", RxHeader.StdId, RxHeader.DLC);
            UART_Transmit_String(&huart2, uart_buffer);
            PrintRawCanData(RxData, RxHeader.DLC); // Print raw data for debugging to Putty

            // --- Multi-Sensor Data Parsing ---
            if (RxHeader.StdId == 0x457 && RxHeader.DLC >= 7)
            {
                // DHT11 Data
                received_temperature = (float)RxData[0] + (float)RxData[1] / 10.0f;
                received_humidity = (float)RxData[2] + (float)RxData[3] / 10.0f;

                // IR Sensor State
                received_ir_state = RxData[4]; // 0 or 1

                // Sound Sensor Value (reconstruct 10-bit analog value)
                received_sound_value = ((uint16_t)RxData[6] << 8) | RxData[5];

                // Sequence Number
                if (RxHeader.DLC >= 8) {
                    received_sequence_num = RxData[7];
                }

                // Print all received sensor data to Putty
                sprintf(uart_buffer, "DHT11: T=%.1f C, H=%.1f %%. IR: %s. Sound: %u. (Seq: %u)\r\n",
                        received_temperature, received_humidity,
                        (received_ir_state == IR_OBSTACLE_DETECTED ? "OBSTACLE" : "CLEAR"),
                        received_sound_value,
                        received_sequence_num);
                UART_Transmit_String(&huart2, uart_buffer);

                // --- ALERT CHECKS (for Putty) ---
                if (received_ir_state == IR_OBSTACLE_DETECTED) {
                    UART_Transmit_String(&huart2, "--> ALERT! Obstacle ahead. Be cautious! Potential crash detected!\r\n");
                }
                if (received_sound_value > SOUND_ALERT_THRESHOLD) {
                    UART_Transmit_String(&huart2, "--> ALERT! Crash detected!!!\r\n");
                }
                if (received_temperature > TEMP_ALERT_THRESHOLD) {
                    UART_Transmit_String(&huart2, "--> ALERT! High temperature detected! FIRE\r\n");
                }
                // --- END ALERT CHECKS ---


                // --- NEW: CALCULATE ACCELERATION/BRAKING AND IDLING ---
                // Calculate time difference since last speed update
                uint32_t current_time_ms = HAL_GetTick();
                if (current_time_ms - last_speed_update_time >= 500) // Update every 500ms
                {
                    // Calculate Acceleration
                    float time_diff_s = (float)(current_time_ms - last_speed_update_time) / 1000.0f;
                    float current_speed_mps = current_speed_kmh / 3.6f;
                    float previous_speed_mps = previous_speed_kmh / 3.6f;
                    acceleration_mps2 = (current_speed_mps - previous_speed_mps) / time_diff_s;
                    previous_speed_kmh = current_speed_kmh;
                    last_speed_update_time = current_time_ms;
                }

                // Check for idling (car is stopped)
                if (current_rpm == 0)
                {
                    idling_timer_ms += (current_time_ms - last_pulse_time_ms);
                }
                else
                {
                    idling_timer_ms = 0;
                    last_pulse_time_ms = current_time_ms;
                }


                // --- NEW: SEND ALL DATA TO ESP32 ---
                // Format: T:XX.X,H:YY.Y,IR:Z,S:AAAA,Seq:BBB,Speed:SS.S,Acc:A.A,RPM:RRRR,Idle:IIII\n
                // Corrected format specifier for RPM and Idle time (which are uint32_t / unsigned long)
                sprintf(esp32_data_buffer, "T:%.1f,H:%.1f,IR:%u,S:%u,Seq:%u,Speed:%.1f,Acc:%.1f,RPM:%lu,Idle:%lu\n",
                        received_temperature, received_humidity,
                        received_ir_state, received_sound_value,
                        received_sequence_num,
                        current_speed_kmh,
                        acceleration_mps2,
                        current_rpm,
                        idling_timer_ms / 1000); // Convert milliseconds to seconds
                UART_Transmit_To_ESP32(esp32_data_buffer); // Send to ESP32!
                UART_Transmit_String(&huart2, "(Data sent to ESP32)\r\n"); // Confirm on Putty
            }

            can_rx_flag = 1; // Signal main loop
            Toggle_LED(GPIOD, GPIO_PIN_15); // Toggle Blue LED (LD6) on reception
        }
    }
}

/**
  * @brief  Error CAN callback. (Unchanged)
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  * the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error_code = HAL_CAN_GetError(hcan);
    can_error_flags = error_code;

    UART_Transmit_String(&huart2, "CAN Error! Code: 0x");
    sprintf(uart_buffer, "%lX - ", error_code);
    UART_Transmit_String(&huart2, uart_buffer);

    if (error_code & HAL_CAN_ERROR_EWG) { UART_Transmit_String(&huart2, "Error Warning! "); }
    if (error_code & HAL_CAN_ERROR_EPV) { UART_Transmit_String(&huart2, "Error Passive! "); }
    if (error_code & HAL_CAN_ERROR_BOF) {
        UART_Transmit_String(&huart2, "Bus-Off! Attempting recovery...\r\n");
        if (HAL_CAN_Stop(hcan) != HAL_OK) Error_Handler();
        if (HAL_CAN_Start(hcan) != HAL_OK) Error_Handler();
        UART_Transmit_String(&huart2, "CAN Re-initialized after Bus-Off.\r\n");
    }
    if (error_code & HAL_CAN_ERROR_STF) { UART_Transmit_String(&huart2, "Stuff Error! "); }
    if (error_code & HAL_CAN_ERROR_FOR) { UART_Transmit_String(&huart2, "Form Error! "); }
    if (error_code & HAL_CAN_ERROR_ACK) { UART_Transmit_String(&huart2, "Acknowledge Error! "); }
    if (error_code & HAL_CAN_ERROR_BR) { UART_Transmit_String(&huart2, "Bit Recessive Error! "); }
    if (error_code & HAL_CAN_ERROR_BD) { UART_Transmit_String(&huart2, "Bit Dominant Error! "); }
    if (error_code & HAL_CAN_ERROR_CRC) { UART_Transmit_String(&huart2, "CRC Error! "); }
    if (error_code & HAL_CAN_ERROR_RX_FOV0) { UART_Transmit_String(&huart2, "RX FIFO0 Overrun! "); }
    UART_Transmit_String(&huart2, "\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start the CAN peripheral
      if (HAL_CAN_Start(&hcan1) != HAL_OK)
      {
          UART_Transmit_String(&huart2, "CAN Start Failed!\r\n");
          Error_Handler();
      } else {
          UART_Transmit_String(&huart2, "CAN Started in Normal Mode.\r\n");
      }

      // Activate CAN Notifications (Interrupts)
      if (HAL_CAN_ActivateNotification(&hcan1,
                                       CAN_IT_RX_FIFO0_MSG_PENDING |
                                       CAN_IT_TX_MAILBOX_EMPTY     |
                                       CAN_IT_BUSOFF               |
                                       CAN_IT_ERROR_WARNING        |
                                       CAN_IT_ERROR_PASSIVE        |
                                       CAN_IT_LAST_ERROR_CODE
                                      ) != HAL_OK)
      {
          UART_Transmit_String(&huart2, "CAN Notification Activation Failed!\r\n");
          Error_Handler();
      } else {
          UART_Transmit_String(&huart2, "CAN Notifications Activated.\r\n");
      }


      // Configure CAN Filter to accept messages from Arduino (ID 0x457 for combined sensor data)
      CAN_FilterTypeDef sFilterConfig;
      sFilterConfig.FilterBank = 0;
      sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
      sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
      sFilterConfig.FilterIdHigh = 0x457 << 5;
      sFilterConfig.FilterIdLow = 0x0000;
      sFilterConfig.FilterMaskIdHigh = 0xFFF << 5;
      sFilterConfig.FilterMaskIdLow = 0x0000;
      sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
      sFilterConfig.FilterActivation = ENABLE;
      sFilterConfig.SlaveStartFilterBank = 14;

      if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
      {
          UART_Transmit_String(&huart2, "CAN Filter Config Failed!\r\n");
          Error_Handler();
      } else {
          UART_Transmit_String(&huart2, "CAN Filter Configured to accept 0x457.\r\n");
      }

      // NEW: Start the Timer in Input Capture Mode and enable its interrupt
      if (HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1) != HAL_OK)
      {
          UART_Transmit_String(&huart2, "Timer 5 Input Capture Start Failed!\r\n");
          Error_Handler();
      }
      else
      {
          UART_Transmit_String(&huart2, "Timer 5 Input Capture Started.\r\n");
      }


      UART_Transmit_String(&huart2, uart_startup_msg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Toggle_LED(GPIOD, GPIO_PIN_12); // Green LED heartbeat
	  HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 9599;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
