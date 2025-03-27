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
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"
#include "stdlib.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	IDLE = 0,
	SPINNING = 1,
	RESET_TO_IDLE = 2
} State_t;

typedef enum {
	EVT_ANY = 0,
	EVT_BUTTON_PRESS = 1,
	EVT_ANIM_COMPLETE = 2
} EventType_t;

typedef struct {
	EventType_t type;
	void* args;
} SystemEvent_t;

typedef struct {
	// maybe add another component here later, e.g duration
	void* args;
	void (*animation)(void*);
} Animation_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PORT GPIOD
#define RED_LED_PIN GPIO_PIN_14
#define ORANGE_LED_PIN GPIO_PIN_13
#define GREEN_LED_PIN GPIO_PIN_12
#define BLUE_LED_PIN GPIO_PIN_15

#define BUTTON_PORT      GPIOA
#define BUTTON_PIN       GPIO_PIN_0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static QueueHandle_t xEventQueue = NULL;
static QueueHandle_t xAnimationQueue = NULL;
static const uint16_t LEDS[4] = {GREEN_LED_PIN, ORANGE_LED_PIN, RED_LED_PIN, BLUE_LED_PIN };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void AnimateTask(void* args);

static void PollButtonTask(void* args);

static void StateMachineTask(void* args);

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
#ifdef DEBUG
	srand((unsigned int) xTaskGetTickCount());
#else
  int32_t seed;
  if (HAL_RNG_GenerateRandomNumber(&hrng, &seed) == HAL_OK) {
	  srand(seed);
  } else {
	  Error_Handler();
  }
#endif
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

  xEventQueue = xQueueCreate(10, sizeof(SystemEvent_t));
  xAnimationQueue = xQueueCreate(5, sizeof(Animation_t));
  if(xEventQueue == NULL || xAnimationQueue == NULL) {
	  Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(AnimateTask, "Animator", configMINIMAL_STACK_SIZE, 128, tskIDLE_PRIORITY +1, NULL);
  xTaskCreate(PollButtonTask, "PollButton", configMINIMAL_STACK_SIZE, 128, tskIDLE_PRIORITY +1, NULL);
  xTaskCreate(StateMachineTask, "HandleStateLogic", configMINIMAL_STACK_SIZE, 128, tskIDLE_PRIORITY +2, NULL);
  vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */

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
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// green, orange, red, blue

static void writeAllLeds(GPIO_PinState state) {
	for(int i = 0; i < 4; ++i) {
		HAL_GPIO_WritePin(LED_PORT, LEDS[i], state);
	}
}

static void toggleLeds() {
	static int toggle = 0;
	writeAllLeds(toggle ? GPIO_PIN_SET : GPIO_PIN_RESET);
	toggle = !toggle;
}

static void wheelAnimation(void* arg) {
	int* finalIndex = (int*)arg;
	const int spins = 5;
	const int totalSpins = (spins*4) + *finalIndex + 1;

	const TickType_t delayMs_inc = 20;
	TickType_t delayMs = 50;
	int current_i = 0;
	for(int i = 0; i < totalSpins; ++i) {
		writeAllLeds(GPIO_PIN_RESET);

		HAL_GPIO_WritePin(LED_PORT, LEDS[current_i], GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(delayMs));
		delayMs += delayMs_inc;
		current_i = (current_i + 1) % 4;
	}
	writeAllLeds(GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LEDS[*finalIndex], GPIO_PIN_SET);
}


static void collectedAnimation(void* args) {
	int* collectedMask = (int*)args;
	for(int j = 0; j < 4; ++j) {
		for(int i = 0; i < 4; ++i) {
			if(*collectedMask & (1 << i)) {
				HAL_GPIO_WritePin(LED_PORT, LEDS[i], GPIO_PIN_SET);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(200));
		writeAllLeds(GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

static void loseAnimation(void* args) {
	for (int i = 0; i < 3; i++) {
		HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(250));
		HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(250));
	}

	writeAllLeds(GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(200));
	writeAllLeds(GPIO_PIN_RESET);
}

static void winningAnimation(void* args) {
    for (int cycle = 0; cycle < 3; cycle++) {
        for (int i = 0; i < 4; i++) {
            writeAllLeds(GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_PORT, LEDS[i], GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
    }

    writeAllLeds(GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(500));
    writeAllLeds(GPIO_PIN_RESET);
}

void AnimateTask(void *args) {
	Animation_t next;
	SystemEvent_t evt;
	for(;;) {
		if(xQueueReceive(xAnimationQueue, &next, portMAX_DELAY) == pdTRUE) {
			next.animation(next.args);

			evt.type = EVT_ANIM_COMPLETE;
			evt.args = NULL;
			xQueueSend(xEventQueue, &evt, 0);
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void PollButtonTask(void *args) {
	SystemEvent_t evt;
	for(;;) {
		if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) {
			evt.type = EVT_BUTTON_PRESS;
			evt.args = NULL;

			xQueueSend(xEventQueue, &evt, 0);
			vTaskDelay(pdMS_TO_TICKS(200)); // Debounce
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void spin(Animation_t* anim, int* colorBit, int* winningNum) {
	*winningNum = rand() % 4;
	*colorBit = (1 << *winningNum);
	anim->animation = wheelAnimation;
	anim->args = winningNum;
}

void setNextAnimation(Animation_t* nextAnim, State_t* state, int* collectedMask, int* colorBit)
{
    switch (*state) {
        case SPINNING:
        	*state = RESET_TO_IDLE;
            if ((*collectedMask) & *colorBit) {
            	(*collectedMask) = 0;
            	nextAnim->animation = loseAnimation;
            } else {
                (*collectedMask) |= *colorBit;
                if (((*collectedMask) & 0xF) == 0xF) {
                    (*collectedMask) = 0;
                    nextAnim->animation = winningAnimation;
                } else {
                    nextAnim->animation = collectedAnimation;
                    nextAnim->args = collectedMask;
                }
            }
            break;
        case RESET_TO_IDLE:
        	*state = IDLE;
            break;
    }
}

void StateMachineTask(void *args) {
	SystemEvent_t evt;
	EventType_t next = EVT_ANY;
	State_t state = IDLE;
	Animation_t nextAnim;

	int next_number = 0;
	nextAnim.animation = wheelAnimation;
	nextAnim.args = &next_number;
	int collectedMask = 0;
	int colorBit = 0;

	for(;;) {
		if(xQueueReceive(xEventQueue, &evt, portMAX_DELAY) == pdTRUE) {
			if(next != EVT_ANY && next != evt.type) {
				continue; // Do not process
			}

			switch (evt.type) {
			case EVT_BUTTON_PRESS:
				state = SPINNING;
				next = EVT_ANIM_COMPLETE;
				spin(&nextAnim, &colorBit, &next_number);
				xQueueSend(xAnimationQueue, &nextAnim, portMAX_DELAY);
				break;
			case EVT_ANIM_COMPLETE:
				setNextAnimation(&nextAnim, &state, &collectedMask, &colorBit);
				if (state == RESET_TO_IDLE) {
					xQueueSend(xAnimationQueue, &nextAnim, portMAX_DELAY);
				}
				else if (state == IDLE) {
					next = EVT_ANY;
				}
			default:
				break;
			}
		}
	}
}

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
	  HAL_GPIO_TogglePin(LED_PORT, RED_LED_PIN);
	  HAL_GPIO_WritePin(LED_PORT, BLUE_LED_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
   	  HAL_GPIO_WritePin(LED_PORT, ORANGE_LED_PIN, GPIO_PIN_RESET);
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
