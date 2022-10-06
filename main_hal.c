/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
#define SPI_PIN_NSS_PB6	6	//PB6
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t spi_read_address(uint8_t slave_address)
{
	uint8_t address = slave_address + (0 << 7);
	uint8_t rx_data = 0;

	GPIOB->ODR &= ~(1 << SPI_PIN_NSS_PB6);

	  HAL_SPI_Transmit(&hspi1, &address, sizeof(uint8_t), 0);

	  HAL_SPI_Receive(&hspi1, &rx_data, sizeof(uint8_t), 0);

	 GPIOB->ODR |= (1 << SPI_PIN_NSS_PB6);

	  return rx_data;
}

void spi_write_to_address(uint8_t slave_address, uint8_t new_reg_value)
{
	uint8_t address = slave_address | 0x80;
	uint8_t rx_data = 0;

	GPIOB->ODR &= ~(1 << SPI_PIN_NSS_PB6);

	  HAL_SPI_Transmit(&hspi1, &address, sizeof(uint8_t), 0);

	  //HAL_SPI_Receive(&hspi1, &rx_data, sizeof(uint8_t), 0);

	  HAL_SPI_Transmit(&hspi1, &new_reg_value, sizeof(uint8_t), 0);

	  //uint8_t read_value = 0;

	  //HAL_SPI_Receive(&hspi1, &read_value, sizeof(uint8_t), 0);

	 GPIOB->ODR |= (1 << SPI_PIN_NSS_PB6);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

#define PART_TWO

#ifdef PART_TWO
  uint8_t tx = 0;
  uint8_t rx = 0;
#endif

#ifdef PART_TWO
  // Setting NSS

  GPIOB->MODER &= ~(3u << (2 * SPI_PIN_NSS_PB6));       //Clear PB6
  GPIOB->MODER |= (1u << (2 * SPI_PIN_NSS_PB6));        //Set PB as output


	// Slave select init at high level
	// Disable Slave Select
	GPIOB->ODR |= (1 << SPI_PIN_NSS_PB6);

	uint8_t transmit_data = 0;
	uint8_t received_data = 0;

	uint8_t dummy_data = 0;
	uint8_t register_read = 0;
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

#ifdef PART_ONE
	  if(tx == 255) {
		  tx = 0;
	  } else {
		  tx++;
	  }


	  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, sizeof(uint8_t), 0);
#endif

#ifdef PART_TWO

	  /*
	   * //WORKING!
	  //Reading 0x01 address and we want to read
	  uint8_t address = 0x05 | (0 << 8);
	  // Enabling slave select
	  GPIOB->ODR &= ~(1 << SPI_PIN_NSS_PB6);

	  HAL_SPI_Transmit(&hspi1, &address, sizeof(uint8_t), 0);

	  HAL_SPI_Receive(&hspi1, &received_data, sizeof(uint8_t), 0);
	  // Disabling slave select
	  GPIOB->ODR |= (1 << SPI_PIN_NSS_PB6);
	  */

	  uint8_t RegOpMode = 0x01;

	  received_data = spi_read_address(RegOpMode);

	  spi_write_to_address(RegOpMode, 0x00); // switching to sleep mode to configure LongRangeMode

	  spi_write_to_address(RegOpMode, 0x80); // switching to LoRa Mode

#endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
