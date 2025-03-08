/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "mcp23008_driver_basic.h"

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
 I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print(char *const pBuffer, size_t u8Length);
uint8_t i2c_read(uint8_t addr, uint8_t *buf, uint16_t len);
uint8_t i2c_write(uint8_t addr, uint8_t *buf, uint16_t len);

typedef enum{                   /**< driver test state chine */

	SET_GPIO_DIR,
	GPIO_WRITE,
	GPIO_READ,
	GPIO_TOGGLE,
	GPIO_EXT_INT,
	PORT_WRITE,

}driver_example_t;

typedef struct test_s{
	driver_example_t state;
}test_t;

test_t test;

uint8_t int_flag;
int btn_press_status;

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
     HAL_GPIO_WritePin(mcp23008_reset_GPIO_Port, mcp23008_reset_Pin, 1); /**< make the reset pin on the slave device is constantly high during communication */

     mcp23008_basic_initialize(MCP23008_I2C_ADDRESS_PIN_A110);		     /**< initialise chip and set i2c pin level (A2, A1, A0) */
 	 mcp23008_info(&mcp23008_handle);

 	 mcp23008_interface_debug_print("Chip name :\t%s\n\r", mcp23008_handle.info.chip_name);
 	 mcp23008_interface_debug_print("Manufacturer: \t%s\n\r",  mcp23008_handle.info.manufacturer_name);

 	 mcp23008_interface_debug_print("Interface: \t%s\n\r",  mcp23008_handle.info.interface);
 	 mcp23008_interface_debug_print("Supply voltage max : \t%0.2fV\n\r",  mcp23008_handle.info.supply_voltage_max_v);
 	 mcp23008_interface_debug_print("Supply voltage min: \t%0.2fV\n\r",  mcp23008_handle.info.supply_voltage_min_v);
 	 mcp23008_interface_debug_print("Maximum current: \t%0.1fmA\n\r",  mcp23008_handle.info.max_current_ma);
 	 mcp23008_interface_debug_print("Temperature Max: \t%.1fC\n\r",  mcp23008_handle.info.temperature_max);
 	 mcp23008_interface_debug_print("Temperature Min: \t%.1fC\n\r",  mcp23008_handle.info.temperature_min);
 	 mcp23008_interface_debug_print("Driver version: \tV%.1f.%.2d\n\r", ( mcp23008_handle.info.driver_version / 1000), (uint8_t)( mcp23008_handle.info.driver_version - (uint8_t)( mcp23008_handle.info.driver_version / 100)*100));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	 HAL_GPIO_TogglePin(user_led_GPIO_Port, user_led_Pin);
	  switch((int)test.state)
	  		{

	  			case SET_GPIO_DIR:
	  			{
	  				/**< set GPIO0 and 5, 6 and 7 as output */
	  				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_7, MCP23008_OUTPUT);
	  				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_6, MCP23008_OUTPUT);
	  				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_5, MCP23008_OUTPUT);

	  				/**< set GPIO0 and 0 and 1 as input */
	  				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_0, MCP23008_INPUT);
	  				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_1, MCP23008_INPUT_PULLUP);

	  				break;
	  			}

	  			case GPIO_WRITE:
	  			{
	  				/**< Write gpio logic level */
	  				mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_7, MCP23008_GPIO_HIGH);
	  				mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_6, MCP23008_GPIO_LOW);

	  				break;
	  			}

	  			case GPIO_READ:
	  			{
	  				/**< read gpio pin !!CONCIDER DEBOUNCING */
	  				if(mcp23008_basic_gpio_read(MCP23008_GPIO_PIN_1))
	  				{
	  					mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_5, MCP23008_GPIO_HIGH);
	  				}else{
	  				  	mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_5, MCP23008_GPIO_LOW);
	  				}

	  				btn_press_status = mcp23008_basic_gpio_read(MCP23008_GPIO_PIN_0);
	  				if(btn_press_status == MCP23008_GPIO_LOW){
	  					mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_6, MCP23008_GPIO_HIGH);
	  				}else{
	  					mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_6, MCP23008_GPIO_LOW);
	  				}
	  				break;
	  			}

	  			case GPIO_TOGGLE:
	  			{
	  				/**< gpio toggle pin */
	  				mcp23008_basic_gpio_toggle(MCP23008_GPIO_PIN_5);
	  				mcp23008_interface_delay_ms(500);
	  				break;
	  			}

	  			case GPIO_EXT_INT:
	  			{
	  				/**< enable interrupt on GPIO 0 as falling edge, disable interrupt on gpio 1*/
	  				mcp23008_basic_interrupt_enable(MCP23008_GPIO_PIN_0, MCP23008_interrupt_FALLING_EDGE);
	  				mcp23008_basic_interrupt_disable(MCP23008_GPIO_PIN_1);

	  				/**< read interrupt flag status*/
	  				mcp23008_basic_get_interrupt_flag(MCP23008_GPIO_PIN_0, &int_flag);
	  				if(int_flag){
	  					mcp23008_basic_clr_interrupt_flag();
	  				}

	  				//mcp23008_basic_gpio_irq_callBack(mcp23008_irq_cb);				/**< interrupt callback function (to be called in the external interrupt callback function) */
	  				break;
	  			}

	  			case PORT_WRITE:
	  			{
	  				mcp23008_basic_pin_write_all(MCP23008_GPIO_HIGH);
	  				mcp23008_interface_delay_ms(500);
	  				mcp23008_basic_pin_write_all(MCP23008_GPIO_LOW);
	  				mcp23008_interface_delay_ms(500);
	  				break;
	  			}

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00909BEB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(mcp23008_reset_GPIO_Port, mcp23008_reset_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(user_led_GPIO_Port, user_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : mcp23008_reset_Pin user_led_Pin */
  GPIO_InitStruct.Pin = mcp23008_reset_Pin|user_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief transmit serial data via usb com port
 * @param[in] pTxBuffer point to data to sent
 * @param[in] length is the data size
 * @return
 * @note		none
 * */
void serial_print(const char *pString, uint8_t u8Length)
{
	HAL_UART_Transmit(&huart2, (const char *) pString, u8Length, HAL_MAX_DELAY);
}

/**
 * @brief i2c data transmit
 * @param[in] addr is the slave address
 * @param[in] reg the register to write
 * @param[in] buf the data to be writen
 * @param[in] len size of data
 * @return  status code
 * 			- 0 success
 * 			- 1 failed to write
 * @note	none
 * */
uint8_t i2c_write(uint8_t addr, uint8_t *buf, uint16_t len)
{
	err = HAL_I2C_Master_Transmit(&hi2c1, (addr<<1), (uint8_t *)buf, len,1000);
	if(err !=HAL_OK)
	{
		return 1;				/**< failed */
	}
	return 0;
}

/**
 * @brief i2c data Read
 * @param[in] addr is the slave address
 * @param[in] reg the register to read
 * @param[out] buf point to data to read
 * @param[in] len size of data
 * @return  status code
 * 			- 0 success
 * 			- 1 failed to read
 * @note	none
 * */

uint8_t i2c_read(uint8_t addr, uint8_t *buf, uint16_t len)
{
	int err;
	err = HAL_I2C_Master_Receive(&hi2c1, (addr<<1), (uint8_t *)buf, len, 1000);
	if(err !=  HAL_OK)
	{
		return 1;				/**< failed */
	}
	return 0;                   /**< success */                  /**< success */
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
