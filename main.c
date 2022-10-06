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
#define SPI_PIN_SCK_PA5	5	// PA5
#define SPI_PIN_MOSI_PA6 6  //PA6
#define SPI_PIN_MISO_PA7 7  //PA7
#define SPI_PIN_NSS_PA4	4	//PA4
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void spi1_set_clock(void)
{
	RCC->APB2ENR |= (1u << 12);    // Enable SPI1 Clock
}

void spi1_set_pins(void)
{
    //RESET PIN MODES
    GPIOA->MODER &= ~((3u << (2 * SPI_PIN_SCK_PA5))       //Clear PA5
                     |(3u << (2 * SPI_PIN_MOSI_PA6))      //Clear PA6
                     |(3u << (2 * SPI_PIN_MISO_PA7))      //Clear PA7
                     );

    GPIOB->MODER &= ~(3u << (2 * SPI_PIN_NSS_PA4));       //Clear PA4


    //CONFIGURE PIN MODES
    GPIOA->MODER |= ((2u << (2 * SPI_PIN_SCK_PA5))        //Set PA5 TO AF
                    |(2u << (2 * SPI_PIN_MOSI_PA6))       //Set PA6 TO AF
                    |(2u << (2 * SPI_PIN_MISO_PA7))       //Set PA7 TO AF
                    );

    GPIOB->MODER |= (1u << (2 * SPI_PIN_NSS_PA4));        //Set PA4 as output

	// AFR[0] controls pin 0 to 7
    // Reset pin alternate functions
    GPIOA->AFR[0] &= ~(
                       (15u << (4 * SPI_PIN_SCK_PA5))     //Clear PA5 AF
					   | (15u << (4 * SPI_PIN_MOSI_PA6))	//Clear PA6 AF
					   | (15u << (4 * SPI_PIN_MISO_PA7))	//Clear PA7 AF
                      );

    // Set pin alternate functions
    GPIOA->AFR[0] &= ~(
                       (5u << (4 * SPI_PIN_SCK_PA5))     //Set PA5 AF
					   | (5u << (4 * SPI_PIN_MOSI_PA6))	//Set PA6 AF
					   | (5u << (4 * SPI_PIN_MISO_PA7))	//Set PA7 AF
                      );
}

void spi1_config_parameters(void)
{
	// SPI1_CR1 Register configuration
	// Clearing bits
	SPI1->CR1 &= ~((1u << 15)           // Full Duplex mode
				  | (1u << 13)           // Disabling CRC
				  | (1u << 10)           // No RX Only => We want RX & TX
				  | (1u << 7)            // MSB first
				  | (7u << 3)            // We manage SPI Frequency divider when setting bits
				  // Slave is an SX1272 LoRa Transceiver, requiring CPOL = 0 & CPHA = 0
				  | (1u << 1)             // CPOL = 0
				  | (1u << 0)             // CPHA = 0
				  );



	// Set bits
	SPI1->CR1 |= ((1u << 9)             // Software slave management
				 | (1u << 8)             // Internal slave select
				 | (5u << 3)             // SPI Frequency divider 64 ==> SPI_CLK = 1 MHz
				 | (1u << 2)             // Master mode
				 );


	// SPI1_CR2 Register configuration
	// Clearing bits
	SPI1->CR2 &= ~((1u << 7)	// Disabling TXEIE Interrupt
					|(1u << 6)	// Disabling RXNEIE Interrupt
					|(1u << 5)	// Disabling ERRIE Interrupt
					|(1u << 4)  // Motorola SPI frame format
					|(1u << 1)  // Disabling TX Buffer DMA
					|(1u << 0)  // Disabling RX Buffer DMA
				  );

	// Setting bits
	SPI1->CR2 |= (1u << 2); // Disabling Slave Select Output in Master mode
}

void spi1_init(void)
{
	spi1_set_clock();
	spi1_set_pins();

	// Slave select init at high level
	// Disable Slave Select
	GPIOA->ODR |= (1 << SPI_PIN_NSS_PA4);

	spi1_config_parameters();

	// Enable SPI1 peripheral
	SPI1->CR1 |= (1u << 6);
}

void spi1_hello_world(void)
{
	// data to send
	uint8_t transmit_data = 0x5D;

	// No slave select because Master's MOSI and MISO are linked together

	uint32_t sr_txe = !!(SPI1->SR & SPI_SR_TXE);
	uint32_t sr_rxne = !!(SPI1->SR & SPI_SR_RXNE);

	// Writing data + dummy data
	SPI1->DR = transmit_data;

	// Waiting for TX Shift Register to be empty
	//while((SPI1->SR & SPI_SR_TXE) == 0);

	//XXX TEST
	//while((SPI1->SR & SPI_SR_RXNE) == 0);

	HAL_Delay(100);
	sr_txe = !!(SPI1->SR & SPI_SR_TXE);
	sr_rxne = !!(SPI1->SR & SPI_SR_RXNE);

	uint8_t received_data = SPI1->DR;

	//TEST
	sr_txe = !!(SPI1->SR & SPI_SR_TXE);
	sr_rxne = !!(SPI1->SR & SPI_SR_RXNE);
	HAL_Delay(100);

}

void spi1_1_byte_transmit(void)
{
	// data to send
	uint8_t transmit_data = 0x5D;

	// Enabling slave select
	GPIOA->ODR &= ~(1 << SPI_PIN_NSS_PA4);

	// Setting data + dummy data
	uint16_t data_and_dummy_data = transmit_data << 8;

	// Writing data + dummy data
	SPI1->DR = data_and_dummy_data;

	// Waiting for TX Shift Register to be empty
	while((SPI1->SR & SPI_SR_TXE) == 0);

	uint8_t received_data = SPI1->DR;

	// Disabling slave select
	GPIOA->ODR |= (1 << SPI_PIN_NSS_PA4);
}

uint8_t spi1_hal_transmit_receive(uint8_t tx_data)
{
	uint8_t rx_data = 0;
	HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, sizeof(uint8_t), 0);

    HAL_Delay(100);
    return rx_data;
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
  //spi1_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t transmit_data = 0xFF;
  uint8_t receive_data = 0;

  /*
	uint8_t dummy_data = 0x00;
	HAL_SPI_Receive(&hspi1, &dummy_data, 1, 0);
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Register
	  /spi1_hello_world();
	  */

	  // HAL
	  if(transmit_data == 255) {
		  transmit_data = 0;
	  } else {
		  transmit_data++;
	  }

	  //receive_data = spi1_hal_transmit_receive(transmit_data);
	  HAL_SPI_TransmitReceive(&hspi1, &transmit_data, &receive_data, sizeof(uint8_t), 0);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
