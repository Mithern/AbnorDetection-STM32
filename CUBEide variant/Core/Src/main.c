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
  *Additional Credit:
  *https://github.com/AC-Tan/BENR3523_Assignment/blob/main/Detection/main.c				- Guidance on the general structure for LIS3DH part!
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "knowledge.h"
#include "NanoEdgeAI.h"

float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER];

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEARNING_ITERATIONS 80 //120 is too much, DEFAULT=70
//Using same declaration, referencing to the original data yield that I can do so.
#define LIS3DH_V_CHIP_ADDR (0x19<<1)	//SA0(=SD0 pin) = Vdd
#define LIS3DH_CTRL_REG1 0x20
#define LIS3DH_CTRL_REG4 0x23
#define LIS3DH_OUT_X_L 0x28
#define BUFFER_SIZE     DATA_INPUT_USER * AXIS_NUMBER
#define NB_AXES         3
#define CLASS_NUMBER	3

#define I2C1_SCL_PIN 	GPIO_PIN_8
#define I2C1_SDA_PIN 	GPIO_PIN_7
#define I2C1_SCL_PORT	GPIOB
#define I2C1_SDA_PORT	GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t buf[32];
uint8_t buf2[64];
float x = 0;
float y = 0;
float z = 0;

uint8_t rxbuf[60];
uint8_t msg1[] = "Confidence: ";
uint8_t msg2[] = "Status:";
uint8_t msg3[] = "ESP32Connect";

uint8_t conf[3];	//Confidence value
//uint8_t stat[1];	//shows A or B, A for normal B for ERROR

//float acc_buffer[BUFFER_SIZE * NB_AXES]; //BUFFER_SIZE * NB_AXES	//Superceded by input_buffer from NEAI AD example
//uint16_t id_class = 0; // Point to id class, unused, AD classification is active
float output_class_buffer[CLASS_NUMBER]; // Buffer of class probabilities
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//Source https://electronics.stackexchange.com/questions/351972/hal-i2c-hangs-cannot-be-solved-with-standard-routine-use-to-unlock-i2c
void I2C1_ClearBusyFlagErratum(I2C_HandleTypeDef *instance)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    int timeout =100;
    int timeout_cnt=0;

    // 1. Clear PE bit.
    instance->Instance->CR1 &= ~(0x0001);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    GPIO_InitStruct.Mode         = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Alternate    = GPIO_AF4_I2C1;
    GPIO_InitStruct.Pull         = GPIO_PULLUP;
    GPIO_InitStruct.Speed        = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin          = I2C1_SCL_PIN;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin          = I2C1_SDA_PIN;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);


    // 3. Check SCL and SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        //Move clock to release I2C
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
        asm("nop");
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_RESET);

    //  5. Check SDA Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);

    //  7. Check SCL Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

    GPIO_InitStruct.Pin = I2C1_SCL_PIN;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C1_SDA_PIN;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 |= 0x8000;

    asm("nop");

    // 14. Clear SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 &= ~0x8000;

    asm("nop");

    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    instance->Instance->CR1 |= 0x0001;

    // Call initialization function.
    HAL_I2C_Init(instance);
}

void fill_buffer(float input_buffer[])
{
	buf[0] = LIS3DH_OUT_X_L | 0x80;	//I Would look into this further, but I have no idea how.
		  for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
			  if (HAL_I2C_IsDeviceReady(&hi2c1, LIS3DH_V_CHIP_ADDR, 3, 100) == 0) { // New data is available
				  buf[0] = LIS3DH_OUT_X_L | 0x80;
				  HAL_I2C_Master_Transmit(&hi2c1, LIS3DH_V_CHIP_ADDR, buf, 1, HAL_MAX_DELAY);
		    	  HAL_I2C_Master_Receive(&hi2c1, LIS3DH_V_CHIP_ADDR, buf, 6, HAL_MAX_DELAY);
		    	  x = buf[1] << 8 | buf[0];
		    	  y = buf[3] << 8 | buf[2];HAL_UART_Transmit(&huart2, buf, strlen(( char*)buf), HAL_MAX_DELAY);
		    	  z = buf[5] << 8 | buf[4];
		          input_buffer[NB_AXES * i] = x;
		          input_buffer[(NB_AXES * i) + 1] = y;
		          input_buffer[(NB_AXES * i) + 2] = z;
			  } else {
		    	  i--; // New data not ready, wait, for loop can be controlled like this ? Yoink!
		      }
		  }
}

void transmit(int input1, int input2) {
	uint8_t stat1[] = "A";
	uint8_t stat2[] = "B";

	char tx1[3];
	sprintf(tx1, "%d", input1),


	//Might need to change this based on how arduino behaves.
	HAL_UART_Transmit(&huart2, msg1, sizeof(msg1), HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart1, , sizeof(msg1), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t *)tx1, strlen(tx1), HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart1, msg1, sizeof(msg1), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, msg2, sizeof(msg2), HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart1, msg1, sizeof(msg1), HAL_MAX_DELAY);
	if(input1==1) {
	HAL_UART_Transmit(&huart2, stat1, sizeof(stat1), HAL_MAX_DELAY);
	}
	else {
	HAL_UART_Transmit(&huart2, stat2, sizeof(stat2), HAL_MAX_DELAY);
	}
	//HAL_UART_Transmit(&huart1, msg1, sizeof(msg1), HAL_MAX_DELAY);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	enum neai_state error_code = neai_anomalydetection_init();
	uint8_t similarity = 0;

	if(error_code != NEAI_OK) {
		//This shows somethting bad happened (wrong board or wrong lib)
		sprintf((char*)buf, "ErrorTx NanoEdge AI\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	}

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_StatusTypeDef ret;

  	HAL_Delay(1999); //10 seconds delay to allow user to connect things.

  	I2C1_ClearBusyFlagErratum(&hi2c1);
/*
  	HAL_UART_Transmit(&huart1, msg3, sizeof(msg3), 1500);		//Send it to the USB serial.

	while(!HAL_UART_Receive(&huart1, rx_buff, 10, 1000)==HAL_OK) //Nothing happening ? STOP THE WHOLE THING LMAO
	{}

	if(HAL_UART_Receive(&huart1, rx_buff, 10, 1000)==HAL_OK) //if transfer is successful
	{
		//__NOP(); //You need to toggle a breakpoint on this line!
		if(rx_buff == (uint8_t) "SUCCESS");{
			HAL_UART_Transmit(&huart2, rx_buff, 10, 1000);		//Send it to the USB serial.
		}
	}
*/

    buf[0] = LIS3DH_CTRL_REG1;
    buf[1] = 0x97;
    ret = HAL_I2C_Master_Transmit(&hi2c1, LIS3DH_V_CHIP_ADDR, buf, 2, HAL_MAX_DELAY);
    if ( ret != HAL_OK ) {
  	  sprintf((char*)buf,"ErrorTx CTRL_REG1\n");	//TODO: CONFIRM THIS IS NEEDED
  	  HAL_UART_Transmit(&huart2, buf, strlen(( char*)buf), HAL_MAX_DELAY);
    }

    buf[0] = LIS3DH_CTRL_REG4;
    buf[1] = 0x08;
    ret = HAL_I2C_Master_Transmit(&hi2c1, LIS3DH_V_CHIP_ADDR, buf, 2, HAL_MAX_DELAY);
    if ( ret != HAL_OK ) {
  	  sprintf((char*)buf,"ErrorTx CTRL_REG4\n");	//TODO: CONFIRM THIS IS NEEDED
  	  HAL_UART_Transmit(&huart2, buf, strlen(( char*)buf), HAL_MAX_DELAY);
    }

    for (uint16_t iteration = 0; iteration < LEARNING_ITERATIONS ; iteration++) {
    	fill_buffer(input_user_buffer);
    	neai_anomalydetection_learn(input_user_buffer);
    }

    int statc = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Getting fresh accelerometer data, code migrated into fill_buffer */
	  /*buf[0] = LIS3DH_OUT_X_L | 0x80;	//I Would look into this further, but I have no idea how.
	  for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		  if (HAL_I2C_IsDeviceReady(&hi2c1, LIS3DH_V_CHIP_ADDR, 3, 100) == 0) { // New data is available
			  buf[0] = LIS3DH_OUT_X_L | 0x80;
			  HAL_I2C_Master_Transmit(&hi2c1, LIS3DH_V_CHIP_ADDR, buf, 1, HAL_MAX_DELAY);
			  HAL_I2C_Master_Receive(&hi2c1, LIS3DH_V_CHIP_ADDR, buf, 6, HAL_MAX_DELAY);
			  x = buf[1] << 8 | buf[0];
			  y = buf[3] << 8 | buf[2];
			  z = buf[5] << 8 | buf[4];
			  acc_buffer[NB_AXES * i] = x;
			  acc_buffer[(NB_AXES * i) + 1] = y;
			  acc_buffer[(NB_AXES * i) + 2] = z;
		  } else {
			  i--; // New data not ready, wait, for loop can be controlled like this ? Yoink!
		  }
	   }
	   */

	  fill_buffer(input_user_buffer);
	  neai_anomalydetection_detect(input_user_buffer, &similarity);	//See how similar the data get is to the


	  if (similarity > 70) {
		  //Normal State Operation
		  statc = 1;
	  }
	  else {
		  //Abnormal State Operation
		  statc = 2;
	  }

	  transmit(similarity,statc);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00201D2B;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
