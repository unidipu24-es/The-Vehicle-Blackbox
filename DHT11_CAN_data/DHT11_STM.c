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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//CAN_RxHeaderTypeDef RxHeader;
//uint8_t RxData[8]; // Buffer to store received CAN data

// --- Start of Queue Implementation ---
typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CanMessage_t;

#define CAN_MESSAGE_QUEUE_SIZE 10
CanMessage_t canMessageQueue[CAN_MESSAGE_QUEUE_SIZE];
volatile int queueHead = 0;
volatile int queueTail = 0;
volatile int queueCount = 0;

void enqueueCanMessage(CanMessage_t msg) {
    if (queueCount < CAN_MESSAGE_QUEUE_SIZE) {
        canMessageQueue[queueHead] = msg;
        queueHead = (queueHead + 1) % CAN_MESSAGE_QUEUE_SIZE;
        queueCount++;
    }
}

CanMessage_t dequeueCanMessage(void) {
    CanMessage_t msg = {0};
    if (queueCount > 0) {
        msg = canMessageQueue[queueTail];
        queueTail = (queueTail + 1) % CAN_MESSAGE_QUEUE_SIZE;
        queueCount--; // Changed from `queueCount--` to `count--` in previous suggestion. Corrected to `queueCount--`
    }
    return msg;
}

int isCanQueueEmpty(void) {
    return queueCount == 0;
}
// --- End of Queue Implementation ---

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
// Callback for CAN RX FIFO 0 Message Pending Interrupt
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
//  {
//    // Message received successfully
//    char msg[60];
//    int len = sprintf(msg, "CAN ID: 0x%lX, DLC: %u, Data: ", (unsigned long)RxHeader.StdId, (unsigned int)RxHeader.DLC);
//
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
//
//    for (int i = 0; i < RxHeader.DLC; i++)
//    {
//      len = sprintf(msg, "0x%02X ", RxData[i]);
//      HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
//    }
//    len = sprintf(msg, "\r\n");
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
//  }
//}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CanMessage_t receivedMsg; // Local variable to store the message temporarily

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &receivedMsg.header, receivedMsg.data) == HAL_OK)
  {
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
	CAN_FilterTypeDef sFilterConfig;
	  CAN_FilterTypeDef sFilterConfig2; // ADDED: New filter config for DHT22 ID
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
  // Debugging: Print APB1 clock frequency
    SystemCoreClockUpdate();
    uint32_t apb1_freq = HAL_RCC_GetPCLK1Freq();
    char buf[50];
    int len_clock = snprintf(buf, sizeof(buf), "APB1 Clock: %lu Hz\r\n", apb1_freq);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len_clock, 100);


    // Configure CAN Filter 0 (for original ID 0x123)
      sFilterConfig.FilterBank = 0;
      sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
      sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
      sFilterConfig.FilterIdHigh = (0x123 << 5);
      sFilterConfig.FilterIdLow = 0x0000;
      sFilterConfig.FilterMaskIdHigh = (0x7FF << 5);
      sFilterConfig.FilterMaskIdLow = 0x0000;
      sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
      sFilterConfig.FilterActivation = ENABLE;
      sFilterConfig.SlaveStartFilterBank = 14;

      if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
      {
        Error_Handler();
      }

    // ADDED: Configure CAN Filter 1 (for DHT22 ID 0x124)
      sFilterConfig2.FilterBank = 1; // Use a different filter bank
      sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
      sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
      sFilterConfig2.FilterIdHigh = (0x124 << 5); // Filter for ID 0x124
      sFilterConfig2.FilterIdLow = 0x0000;
      sFilterConfig2.FilterMaskIdHigh = (0x7FF << 5); // Match all bits of ID 0x124
      sFilterConfig2.FilterMaskIdLow = 0x0000;
      sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to the same FIFO0
      sFilterConfig2.FilterActivation = ENABLE;

      if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig2) != HAL_OK) // Apply the second filter
      {
        Error_Handler();
      }
    // END ADDED

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
      // Process messages from the CAN queue
      if (!isCanQueueEmpty()) {
          CanMessage_t msgToProcess = dequeueCanMessage();
          char uart_buffer[100];
          int len_uart;

          // --- Handle different CAN IDs ---
          if (msgToProcess.header.StdId == 0x123) { // Original data from Arduino
              len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "CAN ID: 0x%lX, DLC: %u, Data: ",
                                  (unsigned long)msgToProcess.header.StdId, (unsigned int)msgToProcess.header.DLC);
              if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, 100);
              }

              for (int i = 0; i < msgToProcess.header.DLC; i++)
              {
                len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "0x%02X ", msgToProcess.data[i]);
                if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, 100);
                }
              }
              len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "\r\n");
              if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, 100);
              }
          }
          else if (msgToProcess.header.StdId == 0x124) { // DHT22 data
              if (msgToProcess.header.DLC >= 4) { // Ensure enough data bytes
                  // Unpack temperature and humidity
                  // Reconstruct 16-bit integers from 8-bit bytes
                  int16_t temperature_raw = (msgToProcess.data[0] << 8) | msgToProcess.data[1];
                  int16_t humidity_raw = (msgToProcess.data[2] << 8) | msgToProcess.data[3];

                  // Convert back to float by dividing by 100.0
                  float temperature = (float)temperature_raw / 100.0;
                  float humidity = (float)humidity_raw / 100.0;

                  len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "DHT22 ID: 0x%lX, Temp: %.2f C, Hum: %.2f %%\r\n",
                                      (unsigned long)msgToProcess.header.StdId, temperature, humidity);
                  if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, 100);
                  }
              } else {
                   len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "DHT22 ID: 0x%lX, Error: Not enough data for DHT22\r\n",
                                       (unsigned long)msgToProcess.header.StdId);
                   if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                       HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, 100);
                   }
              }
          }
          else { // Handle unknown IDs
              len_uart = snprintf(uart_buffer, sizeof(uart_buffer), "Unknown CAN ID: 0x%lX, DLC: %u\r\n",
                                  (unsigned long)msgToProcess.header.StdId, (unsigned int)msgToProcess.header.DLC);
              if (len_uart > 0 && len_uart < sizeof(uart_buffer)) {
                  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len_uart, 100);
              }
          }
          // --- End of CAN ID handling ---
      }

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
