/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t FXOS8700CQ_SLAVE_ADDR= 0x1f<<1;
uint8_t FXAS21002= 0x21<<1;

// FXOS8700CQ internal register addresses
uint8_t FXOS8700CQ_WHOAMI = 0x0D;
uint8_t FXAS21002_WHOAMI= 0x0C;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t rec_data[1]={12};
uint8_t snum[100];
uint8_t ret=7;
uint8_t accelconfig[2]={0x2A,0x00};
uint8_t accelconfig2[2]={0x2A,0x0D};
uint8_t magconfig[2]={0x5B,0x1f};
uint8_t magconfig2[2]={0x5C,0x20};
uint8_t XYZ_DATA_CFG[2]= {0x0E,0x01};
uint8_t Buffer[13];
uint8_t gyrobuf[7]={0};


int x,y,z, mx,my,mz, gx,gy,gz;
float gxx,gyy,gzz;

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
void accelmagconfig(void);
void gyroconfig(void);
void readgyro(void);

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
/*put code that ensures both sensors and all sensor registers are reset
  and/or put in standby/ready mode before set up (where appropraite)*/

  HAL_Delay(500);

 accelmagconfig();

  gyroconfig();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// HAL_I2C_Mem_Read(&hi2c1, FXAS21002, FXAS21002_WHOAMI, 1, rec_data, 1, 1000); //readonh whoami regsiters
	  HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR, 0x00, 1, Buffer, 13, 1000);

	  readgyro();


	  x = (int16_t)(((Buffer[1] << 8) | Buffer[2]))>> 2;
	  y = (int16_t)(((Buffer[3] << 8) | Buffer[4]))>> 2;
	  z = (int16_t)(((Buffer[5] << 8) | Buffer[6]))>> 2;
	  // copy the magnetometer byte data into 16 bit words
	  mx = (Buffer[7] << 8) | Buffer[8];
	  my = (Buffer[9] << 8) | Buffer[10];
	  mz = (Buffer[11] << 8) | Buffer[12];


	 //sprintf((char*)snum, "%.i,%.i,%.i,%.i,%.i,%.i,%.2f,%.2f,%.2f\r \n ", mx,my,mz,x,y,z,gxx,gyy,gzz);
	  //sprintf((char*)snum, "%.i,%.i,%.i\r \n ",);
	// sprintf((char*)snum, "%.2f,%.2f,%.2f\r \n ", );
	  //sprintf((char*)snum, "%.i\r \n ", gyrobuf[0]);


	//sprintf((char*)snum, "%.i\r \n ", rec_data[0]);




	 // sprintf((char*)snum, "%.i\r \n ", ret);

	   	HAL_UART_Transmit(&huart2, snum, strlen((char*)snum), HAL_MAX_DELAY);
	   //	HAL_Delay(25);

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
  hi2c1.Init.ClockSpeed = 400000;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void accelmagconfig(void) {
ret = HAL_I2C_Master_Transmit(&hi2c1,FXOS8700CQ_SLAVE_ADDR, accelconfig, 2, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {
	   sprintf((char*)snum, "%.i\r \n ", ret);
        }

   ret = HAL_I2C_Master_Transmit(&hi2c1,FXOS8700CQ_SLAVE_ADDR, magconfig, 2, HAL_MAX_DELAY);
     if ( ret != HAL_OK ) {
  	   sprintf((char*)snum, "%.i\r \n ", ret);
           }

       ret = HAL_I2C_Master_Transmit(&hi2c1,FXOS8700CQ_SLAVE_ADDR, magconfig2, 2, HAL_MAX_DELAY);
          if ( ret != HAL_OK ) {
       	   sprintf((char*)snum, "%.i\r \n ", ret);
                }

           ret = HAL_I2C_Master_Transmit(&hi2c1,FXOS8700CQ_SLAVE_ADDR, XYZ_DATA_CFG, 2, HAL_MAX_DELAY);
                     if ( ret != HAL_OK ) {
                  	   sprintf((char*)snum, "%.i\r \n ", ret);
                           }

                  ret = HAL_I2C_Master_Transmit(&hi2c1,FXOS8700CQ_SLAVE_ADDR, accelconfig2, 2, HAL_MAX_DELAY);
                                           if ( ret != HAL_OK ) {
                                        	   sprintf((char*)snum, "%.i\r \n ", ret);
                                                 }
                                           }

void gyroconfig(void){
	uint8_t CTRL_REG1[2]={0x13,0x00};
	  uint8_t CTRL_REG12[2]={0x13,0x40};
	  uint8_t CTRL_REG0[2]={ 0x0D,0x03};
	  uint8_t CTRL_REG13[2]={ 0x13,0x0E};

	 ret = HAL_I2C_Master_Transmit(&hi2c1,FXAS21002, CTRL_REG1, 2, 5000);
	    if ( ret != HAL_OK ) {
	  	   sprintf((char*)snum, "%.i\r \n ", ret);
	          }
	 ret = HAL_I2C_Master_Transmit(&hi2c1,FXAS21002, CTRL_REG12, 2, 5000);
	        if ( ret != HAL_OK ) {
	      	   sprintf((char*)snum, "%.i\r \n ", ret);
	              }
	 ret = HAL_I2C_Master_Transmit(&hi2c1,FXAS21002, CTRL_REG0, 2, 5000);
	               if ( ret != HAL_OK ) {
	             	   sprintf((char*)snum, "%.i\r \n ", ret);
	                     }
     ret = HAL_I2C_Master_Transmit(&hi2c1,FXAS21002, CTRL_REG13, 2, 5000);
	                      if ( ret != HAL_OK ) {
	                    	   sprintf((char*)snum, "%.i\r \n ", ret);
	                            }
	                      HAL_Delay(100);
}

void readgyro(void){
	HAL_I2C_Mem_Read(&hi2c1, FXAS21002, 0x00, 1, gyrobuf, 7, 1000);
		  if(gyrobuf[0]==15){
	      gx = (int16_t)((gyrobuf[1] << 8) | gyrobuf[2]);
	  	  gy = (int16_t)((gyrobuf[3] << 8) | gyrobuf[4]);
	  	  gz = (int16_t)((gyrobuf[5] << 8 )| gyrobuf[6]);
	  	      gxx = (float)gx*0.0078125*0.017453293;
	  	  	  gyy = (float)gy*0.0078125*0.017453293;
	  	  	  gzz = (float)gz*0.0078125*0.017453293;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
