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
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Structure to hold a CAN message, including header and data */
typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CanMessage_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define CAN IDs used by the Arduino sender
#define CAN_ID_DHT11   0x124
#define CAN_ID_PIR     0x125
#define CAN_ID_SOUND   0x126
#define CAN_ID_ORIGINAL 0x123 // Original ID from your STM32 code, kept for compatibility

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Queue for storing received CAN messages */
#define CAN_MESSAGE_QUEUE_SIZE 10
CanMessage_t canMessageQueue[CAN_MESSAGE_QUEUE_SIZE];
volatile int queueHead = 0;
volatile int queueTail = 0;
volatile int queueCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Enqueues a CAN message into the circular buffer queue.
  * @param msg: The CAN message to enqueue.
  * @retval None
  */
void enqueueCanMessage(CanMessage_t msg) {
    if (queueCount < CAN_MESSAGE_QUEUE_SIZE) {
        canMessageQueue[queueHead] = msg;
        queueHead = (queueHead + 1) % CAN_MESSAGE_QUEUE_SIZE;
        queueCount++;
    }
    // If queue is full, message is dropped. Consider error handling if critical.
}

/**
  * @brief Dequeues a CAN message from the circular buffer queue.
  * @param None
  * @retval The dequeued CAN message. Returns a zero-initialized message if queue is empty.
  */
CanMessage_t dequeueCanMessage(void) {
    CanMessage_t msg = {0}; // Initialize with zeros
    if (queueCount > 0) {
        msg = canMessageQueue[queueTail];
        queueTail = (queueTail + 1) % CAN_MESSAGE_QUEUE_SIZE;
        queueCount--;
    }
    return msg;
}

/**
  * @brief Checks if the CAN message queue is empty.
  * @param None
  * @retval 1 if empty, 0 otherwise.
  */
int isCanQueueEmpty(void)
{
    return queueCount == 0;
}

/**
  * @brief Callback function for CAN RX FIFO 0 Message Pending Interrupt.
  * This function is called automatically by the HAL when a message is received.
  * @param hcan: Pointer to the CAN handle.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CanMessage_t receivedMsg; // Local variable to store the message temporarily

  // Get the received CAN message from FIFO0
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &receivedMsg.header, receivedMsg.data) == HAL_OK)
  {
    // Enqueue the received message for processing in the main loop
    enqueueCanMessage(receivedMsg);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  CAN_FilterTypeDef sFilterConfig0; // Filter for original ID 0x123
  CAN_FilterTypeDef sFilterConfig1; // Filter for DHT11 ID 0x124
  CAN_FilterTypeDef sFilterConfig2; // Filter for PIR ID 0x125
  CAN_FilterTypeDef sFilterConfig3; // Filter for Sound ID 0x126
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Debugging: Print APB1 clock frequency to UART
  SystemCoreClockUpdate();
  uint32_t apb1_freq = HAL_RCC_GetPCLK1Freq();
  char buf[50];
  int len_clock = snprintf(buf, sizeof(buf), "APB1 Clock: %lu Hz\r\n", apb1_freq);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, len_clock, HAL_MAX_DELAY);
  HAL_Delay(100); // Small delay to ensure message is sent

  // --- Configure CAN Filters for all expected IDs ---

  // Filter 0: For original ID 0x123 (if still used by another sender)
  sFilterConfig0.FilterBank = 0;
  sFilterConfig0.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig0.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig0.FilterIdHigh = (CAN_ID_ORIGINAL << 5); // Shift ID to match filter register format
  sFilterConfig0.FilterIdLow = 0x0000;
  sFilterConfig0.FilterMaskIdHigh = (0x7FF << 5); // Mask to match exact 11-bit standard ID
  sFilterConfig0.FilterMaskIdLow = 0x0000;
  sFilterConfig0.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO0
  sFilterConfig0.FilterActivation = ENABLE;
  sFilterConfig0.SlaveStartFilterBank = 14; // Start bank for slave CAN (if applicable, often 14 for CAN1)

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig0) != HAL_OK)
  {
    Error_Handler();
  }

  // Filter 1: For DHT11 ID 0x124
  sFilterConfig1.FilterBank = 1; // Use a different filter bank
  sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig1.FilterIdHigh = (CAN_ID_DHT11 << 5);
  sFilterConfig1.FilterIdLow = 0x0000;
  sFilterConfig1.FilterMaskIdHigh = (0x7FF << 5);
  sFilterConfig1.FilterMaskIdLow = 0x0000;
  sFilterConfig1.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO0
  sFilterConfig1.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1) != HAL_OK)
  {
    Error_Handler();
  }

  // Filter 2: For PIR ID 0x125
  sFilterConfig2.FilterBank = 2; // Use another different filter bank
  sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig2.FilterIdHigh = (CAN_ID_PIR << 5);
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = (0x7FF << 5);
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO0
  sFilterConfig2.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig2) != HAL_OK)
  {
    Error_Handler();
  }

  // Filter 3: For Sound ID 0x126
  sFilterConfig3.FilterBank = 3; // Use another different filter bank
  sFilterConfig3.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig3.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig3.FilterIdHigh = (CAN_ID_SOUND << 5);
  sFilterConfig3.FilterIdLow = 0x0000;
  sFilterConfig3.FilterMaskIdHigh = (0x7FF << 5);
  sFilterConfig3.FilterMaskIdLow = 0x0000;
  sFilterConfig3.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO0
  sFilterConfig3.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig3) != HAL_OK)
  {
    Error_Handler();
  }

  // Start CAN peripheral
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  // Activate CAN RX0 message pending interrupt
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Process messages from the CAN queue if not empty
    if (!isCanQueueEmpty()) {
        CanMessage_t msgToProcess = dequeueCanMessage();
        char uart_buffer[100]; // Buffer for UART messages
        int len_uart;

        // --- Handle different CAN IDs ---
        if (msgToProcess.header.StdId == CAN_ID_ORIGINAL) { // Original data from Arduino (0x123)
            len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "CAN ID: 0x%lX, DLC: %u, Data: ",
                                (unsigned long)msgToProcess.header.StdId, (unsigned int)msgToProcess.header.DLC);
            if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
            }

            for (int i = 0; i < msgToProcess.header.DLC; i++)
            {
              len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "0x%02X ", msgToProcess.data[i]);
              if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
              }
            }
            len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "\r\n");
            if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
            }
        }
        else if (msgToProcess.header.StdId == CAN_ID_DHT11) { // DHT11 data (0x124)
            if (msgToProcess.header.DLC >= 4) { // Ensure enough data bytes for Temp and Hum
                // Unpack temperature and humidity (scaled by 100)
                int16_t temperature_raw = (msgToProcess.data[0] << 8) | msgToProcess.data[1];
                int16_t humidity_raw = (msgToProcess.data[2] << 8) | msgToProcess.data[3];

                // Convert back to float by dividing by 100.0
                float temperature = (float)temperature_raw / 100.0;
                float humidity = (float)humidity_raw / 100.0;

                len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "DHT11 ID: 0x%lX, Temp: %.2f C, Hum: %.2f %%\r\n",
                                    (unsigned long)msgToProcess.header.StdId, temperature, humidity);
                if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
                }
            } else {
                 len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "DHT11 ID: 0x%lX, Error: Not enough data for DHT11\r\n",
                                     (unsigned long)msgToProcess.header.StdId);
                 if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                     HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
                 }
            }
        }
        else if (msgToProcess.header.StdId == CAN_ID_PIR) { // PIR Sensor data (0x125)
            if (msgToProcess.header.DLC >= 1) { // Ensure at least one data byte
                uint8_t pir_state = msgToProcess.data[0];
                if (pir_state == 1) {
                    len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "PIR ID: 0x%lX, Status: Motion Detected\r\n",
                                        (unsigned long)msgToProcess.header.StdId);
                } else {
                    len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "PIR ID: 0x%lX, Status: No Motion\r\n",
                                        (unsigned long)msgToProcess.header.StdId);
                }
                if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
                }
            } else {
                len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "PIR ID: 0x%lX, Error: Not enough data for PIR\r\n",
                                    (unsigned long)msgToProcess.header.StdId);
                if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
                }
            }
        }
        else if (msgToProcess.header.StdId == CAN_ID_SOUND) { // Sound Sensor data (0x126)
            if (msgToProcess.header.DLC >= 1) { // Ensure at least one data byte
                uint8_t sound_state = msgToProcess.data[0];
                if (sound_state == 1) {
                    len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "SOUND ID: 0x%lX, Status: Sound Detected\r\n",
                                        (unsigned long)msgToProcess.header.StdId);
                } else {
                    // Arduino only sends '1' for detection, but if '0' were sent, this handles it.
                    len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "SOUND ID: 0x%lX, Status: No Sound\r\n",
                                        (unsigned long)msgToProcess.header.StdId);
                }
                if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
                }
            } else {
                len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "SOUND ID: 0x%lX, Error: Not enough data for Sound\r\n",
                                    (unsigned long)msgToProcess.header.StdId);
                if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
                }
            }
        }
        else { // Handle any other unknown CAN IDs
            len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "Unknown CAN ID: 0x%lX, DLC: %u\r\n",
                                (unsigned long)msgToProcess.header.StdId, (unsigned int)msgToProcess.header.DLC);
            if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, HAL_MAX_DELAY);
            }
        }
        // --- End of CAN ID handling ---
    }
    HAL_Delay(5); // Small delay to prevent busy-waiting and allow other tasks if any
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /*
   * For 500 KBPS with 8MHz PCLK1 (HSI default on some STM32F4/F7, check your specific MCU):
   * Time Quanta (TQ) = (Prescaler + 1) * APB1_Clock_Period
   * Bit Time = (1 + TimeSeg1 + TimeSeg2) * TQ
   *
   * Example: APB1 = 8 MHz
   * Prescaler = 4 (results in 5 TQ)
   * TQ = (4 + 1) / 8,000,000 = 5 / 8,000,000 = 0.625 us
   *
   * TimeSeg1 = 6 TQ
   * TimeSeg2 = 1 TQ
   * SyncJumpWidth = 1 TQ
   *
   * Total TQ per bit = 1 (Sync Seg) + 6 (TimeSeg1) + 1 (TimeSeg2) = 8 TQ
   *
   * Bit Rate = 1 / (Bit Time) = 1 / (8 * TQ) = 1 / (8 * 0.625 us) = 1 / 5 us = 200,000 bps = 200 KBPS
   *
   * To achieve 500 KBPS with 8MHz APB1:
   * Total TQ per bit = 8,000,000 / 500,000 = 16 TQ
   *
   * Let's try:
   * Prescaler = 0 (results in 1 TQ) -> TQ = 1 / 8,000,000 = 0.125 us
   * TimeSeg1 = 13 TQ
   * TimeSeg2 = 2 TQ
   * Total TQ = 1 + 13 + 2 = 16 TQ
   * Bit Rate = 1 / (16 * 0.125 us) = 1 / 2 us = 500,000 bps = 500 KBPS
   *
   * So, Prescaler = 0, TimeSeg1 = CAN_BS1_13TQ, TimeSeg2 = CAN_BS2_2TQ
   * This assumes APB1 clock is 8MHz. Double-check your SystemClock_Config.
   * If your APB1 is higher, you'll need to adjust the prescaler.
   */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
