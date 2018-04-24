
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ADDR_AT24C02_Write 0xA0
#define ADDR_AT24C02_Read 0xA1
uint8_t I2C_Buffer_Write[256];
uint8_t I2C_Buffer_Read[256];
uint16_t i;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  /* USER CODE BEGIN 2 */

	  printf("\r\n \r\n*********STM32CubeMX I2C AT24C02 Example*********\r\n");
	  printf("\r\n I2C Write Buffer:\r\n");
	  for(i=0; i<256; i++)
	  {
		  I2C_Buffer_Write[i]=i;    /* WriteBuffer Initialization */
		  printf("%02X ", I2C_Buffer_Write[i]);

	  }

	  /* write data to AT24C02 */
	  for(i=0; i<256; i=i+8)
//	  for(i=0; i<256; i++)
	  {
		  /*
		   *  HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		   * I2C_HandleTypeDef *hi2c IIC的句柄

    		uint16_t DevAddress IIC设备的外部地址

    		uint16_t MemAddress IIC设备的内部地址

    		uint16_t MemAddSize 读写内部地址的方式，一般使用I2C_MEMADD_SIZE_8BIT方式,
    		注意只有两种选择：I2C_MEMADD_SIZE_8BIT 或者 I2C_MEMADD_SIZE_16BIT

    		uint8_t *pData 写入的数组或者指针

    		uint16_t Size 写入的字节数！
		   */
			/*以下三个语句都可以正确写入
			 *I2C_Buffer_Write数组名实际上就是指针，指针+i其实就是下一组8个数组元素起始地址
			 */
//		  if (HAL_I2C_Mem_Write(&hi2c2, ADDR_AT24C02_Write, i, I2C_MEMADD_SIZE_8BIT,I2C_Buffer_Write+i,8, 10000) == HAL_OK)
//		  if (HAL_I2C_Mem_Write(&hi2c2, ADDR_AT24C02_Write, i, I2C_MEMADD_SIZE_8BIT,(uint8_t*)(&(I2C_Buffer_Write[i])),8, 10000) == HAL_OK)
		  if (HAL_I2C_Mem_Write(&hi2c2, ADDR_AT24C02_Write, i, I2C_MEMADD_SIZE_8BIT,&(I2C_Buffer_Write[i]),8, 1000) == HAL_OK)
		  {
			  printf("\r\n Byte %02d to Byte %02d Write OK",i,i+7);
			  HAL_Delay(5);//不能少
		  }

		  else
			  printf("\r\n Byte %02d to Byte %02d Write Failed",i,i+7);
	  }


	  /* read data from EEPROM */
	  printf("\r\n Reading from AT24C02:\r\n");
	  HAL_I2C_Mem_Read(&hi2c2, ADDR_AT24C02_Read, 0, I2C_MEMADD_SIZE_8BIT,I2C_Buffer_Read,256, 1000);

	  for(i=0; i<256; i++)
	  {
		  printf("0x%02X  ",I2C_Buffer_Read[i]);
	  }
	  HAL_Delay(2000);
//

	  if(memcmp(I2C_Buffer_Read,I2C_Buffer_Write,256) == 0 ) /* check data */
	      printf("\r\n AT24C02 Read Test OK\r\n");
	  else
	      printf("\r\n AT24C02 Read Test Failed\r\n");

	  /* USER CODE END 2 */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
