/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct mouseHID_t {
	uint8_t buttons;
	int8_t x;
	int8_t y;
	int8_t wheel;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEFT_BTN 0x01
#define RIGHT_BTN 0x02
#define MIDDLE_BTN 0x04
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t mouseTaskHandle;
const osThreadAttr_t mouseTask_attributes = {
  .name = "mouseTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t customHIDTaskHandle;
const osThreadAttr_t customHIDTask_attributes = {
  .name = "customHIDTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void mouseTask(void *argument);
void customHIDTask(void *argument);
void LEDTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;
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

  /* Сброс USB */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_Delay(100);
  HAL_Init();
  SystemClock_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */


  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  mouseTaskHandle = osThreadNew(mouseTask, NULL, &mouseTask_attributes);

  //LEDTaskHandle = osThreadNew(LEDTask, NULL, &LEDTask_attributes);


  customHIDTaskHandle = osThreadNew(customHIDTask, NULL, &customHIDTask_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN4_Pin RETURN_TO_BOOTLOADER_Pin RIGHT_Pin UP_Pin
	                           LEFT_Pin DOWN_Pin */
	GPIO_InitStruct.Pin = BTN4_Pin|RETURN_TO_BOOTLOADER_Pin|RIGHT_Pin|UP_Pin
			|LEFT_Pin|DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN5_Pin BTN3_Pin BTN2_Pin BTN1_Pin */
	GPIO_InitStruct.Pin = BTN5_Pin|BTN3_Pin|BTN2_Pin|BTN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void LEDTask(void *argument)
{
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  osDelay(250);
  }
}

void customHIDTask(void *argument)
{
	uint8_t ggg[5];

	ggg[0] = 2;
	ggg[1] = 1;
	ggg[2] = 2;
	ggg[3] = 3;
	ggg[4] = 4;


  for(;;)
  {
	  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, ggg, sizeof(ggg));
	  osDelay(1000);
  }
}

void mouseTask(void *argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN 5 */

  struct mouseHID_t mouseHID;
  GPIO_PinState oldSCROLL_UP = HAL_GPIO_ReadPin(SCROLL_UP_GPIO_Port, SCROLL_UP_Pin);
  GPIO_PinState oldSCROLL_DOWN = HAL_GPIO_ReadPin(SCROLL_DOWN_GPIO_Port, SCROLL_DOWN_Pin);
  uint16_t stickTime = 0;
  uint16_t speed = 0;
  uint8_t oldDir = 0;
  /* Infinite loop */
  for(;;)
  {
	  mouseHID.buttons = 0;
	  mouseHID.x = 0;
	  mouseHID.y = 0;
	  mouseHID.wheel = 0;

	  uint8_t up = (HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == GPIO_PIN_RESET)?1:0;
	  uint8_t down = (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == GPIO_PIN_RESET)?1:0;
	  uint8_t left = (HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin) == GPIO_PIN_RESET)?1:0;
	  uint8_t right = (HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin) == GPIO_PIN_RESET)?1:0;
	  uint8_t dir = (up << 0) | (down << 1) | (left << 2) | (right << 3);

	  // Сброс скорости при смене направления
	  if (dir != oldDir){
		  oldDir = dir;
		  if (speed > 1){
			  speed = 0;
			  stickTime = 0;
		  }
	  }

	  if (dir){
		  if (stickTime == 0){
			  if (speed < 30) speed++;
			  stickTime = 200;
		  }
	  } else {
		  speed = 0;
		  stickTime = 0;
	  }

	  if (left || right){
		  mouseHID.x = ((right)?1:-1) * speed;
	  }
	  if (up || down){
		  mouseHID.y = ((down)?1:-1) * speed;
	  }


	  if (HAL_GPIO_ReadPin(SCROLL_UP_GPIO_Port, SCROLL_UP_Pin) != oldSCROLL_UP){
		  oldSCROLL_UP = HAL_GPIO_ReadPin(SCROLL_UP_GPIO_Port, SCROLL_UP_Pin);
		  if (oldSCROLL_UP == GPIO_PIN_RESET){
			  HAL_Delay(10);
			  mouseHID.wheel = 3;
		  }
	  }

	  if (HAL_GPIO_ReadPin(SCROLL_DOWN_GPIO_Port, SCROLL_DOWN_Pin) != oldSCROLL_DOWN){
		  oldSCROLL_DOWN = HAL_GPIO_ReadPin(SCROLL_DOWN_GPIO_Port, SCROLL_DOWN_Pin);
		  if (oldSCROLL_DOWN == GPIO_PIN_RESET){
			  HAL_Delay(10);
			  mouseHID.wheel = -3;
		  }
	  }


	  if (HAL_GPIO_ReadPin(L_KEY_GPIO_Port, L_KEY_Pin) == GPIO_PIN_RESET) mouseHID.buttons |= LEFT_BTN;
	  if (HAL_GPIO_ReadPin(R_KEY_GPIO_Port, R_KEY_Pin) == GPIO_PIN_RESET) mouseHID.buttons |= RIGHT_BTN;
	  if (HAL_GPIO_ReadPin(M_KEY_GPIO_Port, M_KEY_Pin) == GPIO_PIN_RESET) mouseHID.buttons |= MIDDLE_BTN;

	  //if (mouseHID.buttons || mouseHID.x || mouseHID.y || mouseHID.wheel){
	  USBD_MOUSE_SendReport(&hUsbDeviceFS, &mouseHID, sizeof(struct mouseHID_t));
	  osDelay(1);
	  //}

	  if (stickTime) stickTime--;
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
