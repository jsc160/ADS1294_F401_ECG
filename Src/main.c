/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define delayUs(x) { unsigned int _dcnt; \
      _dcnt=(x*16); \
      while(_dcnt-- > 0) \
      { continue; }\
     }

		 
#define CMD_BYTE  0x41 //0x41 //0x69 //0x69-24daolian  0xf9-18daolian
#define PKT_LENGTH 22//22 //78
#define LED_B		GPIO_PIN_3
#define LED_G		GPIO_PIN_4		 
#define LED_R		GPIO_PIN_5	 
		  
//1294 CONFIG
#define ADS1294_CS_L    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define ADS1294_CS_H    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define ADS1294_START_L    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define ADS1294_START_H    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define ADS1294_RESET_L    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define ADS1294_RESET_H    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)

#define RESET 	0x06
#define START   0x08
#define RDATAC	0x10
#define SDATAC	0x11
#define RDATA		0x12
#define RREG 0x20
#define WREG 0x40
#define ID_ADDR				0x00
#define CONFIG1_ADDR	0x01
#define CONFIG2_ADDR	0x02
#define CONFIG3_ADDR	0x03
#define LOFF					0x04

#define CH1SET_ADDR		0x05
#define RLD_SENSP     0x0D
#define RLD_SENSN			0x0E
#define LOFF_SENSP 		0x0F
#define LOFF_SENSN 		0x10
#define LOFF_FLIP 		0x11
#define	GPIO_ADDR			0x14
#define MISC1_ADDR		0x15
#define MISC2_ADDR		0x16
#define CONFIG4_ADDR	0x17
#define ADC1_SRB1			0x20
#define ADC2_SRB1			0x20




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t g_time_seq;
uint16_t g_seq;
uint8_t g_pkt_buf[PKT_LENGTH];
uint8_t adc1_data_ready=0;
uint8_t g_adc1_buf[64];
uint8_t  aRxBuffer[100] ;
uint8_t send_flag=0;
uint8_t Res=0;
uint8_t g_ContiueSend=0;
uint32_t  g_mstimer=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendDataTo_device(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len)
{
		HAL_UART_Transmit_DMA(huart, buf, len);
}

void __printf(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len)
{
	for(uint16_t i=0;i<len;i++)
	{
		while((huart->Instance->SR & UART_FLAG_TC)==0);//循环发送,直到发送完毕 
		huart->Instance->DR = buf[i];
	}
}

void ADS1299_RESET()
{
	ADS1294_RESET_L;
	HAL_Delay(1);
	ADS1294_RESET_H;
}

uint8_t ADS1294_WRITE(uint8_t *tmp_TXData, uint8_t len)
{
		uint8_t ret=0;
	
		uint8_t tmp_RXData[20];
		ADS1294_CS_L;
		ret=HAL_SPI_TransmitReceive(&hspi1, tmp_TXData, tmp_RXData, len, 1000);
		ADS1294_CS_H;
		delayUs(50);
	
	return ret;
}

void ADS1294_Init()
{
		uint8_t tmp_TXData[20];

		//soft reset
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = RESET;
		ADS1294_WRITE(tmp_TXData, 1);

		HAL_Delay(10);
	
		//sdatac command
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = SDATAC;
		ADS1294_WRITE(tmp_TXData, 1);

#if 1	
		//write config 1 
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|CONFIG1_ADDR;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0x64;//0x64; //0x64-1k , 0x63-2k, 0x62-4k, 0x61-8k
		ADS1294_WRITE(tmp_TXData, 3);

		
		//write config 2 
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|CONFIG2_ADDR;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0x10;
		ADS1294_WRITE(tmp_TXData, 3);

		//write config 3 
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|CONFIG3_ADDR;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0xD6;//0x40; //0x60  0xc0
		ADS1294_WRITE(tmp_TXData, 3);

		//write LOFF 
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|LOFF;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0x13;//	0x1F		??DC????
		//tmp_TXData[2] = 0x0F;//			??DC????  DC??
		ADS1294_WRITE(tmp_TXData, 3);


		//write LOFF_SENSP 
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|LOFF_SENSP;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0x01;//			??DC????
		ADS1294_WRITE(tmp_TXData, 3);
		
		//write LOFF_SENSN
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|LOFF_SENSN;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0x01;//			??DC????
		ADS1294_WRITE(tmp_TXData, 3);

		//write LOFF_FLIP 
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|LOFF_FLIP;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0x00;//			??DC????
		ADS1294_WRITE(tmp_TXData, 3);



		//write config 4 
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|CONFIG4_ADDR;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0x02; //SINGLE SHOT
		ADS1294_WRITE(tmp_TXData, 3);


		//write CHnSET 
		memset(tmp_TXData, 0, sizeof(tmp_TXData));//0X00= normal input  0X01=SHORTED 0X02= RLD 0X0  0X03=MVDD
		tmp_TXData[0] = WREG|CH1SET_ADDR;
		tmp_TXData[1] = 0x63;
		
		tmp_TXData[2] = 0x00;    //1??????????
		tmp_TXData[3] = 0x01;		 //2????????
		tmp_TXData[4] = 0x02;		 //3????RLD??
		tmp_TXData[5] = 0x07;	   //4?? ??VREF-RLD
		ADS1294_WRITE(tmp_TXData, 6);
		
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|RLD_SENSP;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0x01; //SINGLE SHOT
		ADS1294_WRITE(tmp_TXData, 3);
		
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = WREG|RLD_SENSN;
		tmp_TXData[1] = 0x00;
		tmp_TXData[2] = 0x01; //SINGLE SHOT
		ADS1294_WRITE(tmp_TXData, 3);
		
//		tmp_TXData[1] = 0x07;
//		tmp_TXData[2] = 0x00;
//		if(adc_num == 1)
//			tmp_TXData[6] = 0x04;
//			tmp_TXData[9] = 0x04; //stonemm   0x01  0x04??
//		ADC_WRITE(adc_num, tmp_TXData, 10);
			
		HAL_Delay(1);
		
	#endif
}

void ADS1294_PowerOnInit(void)
{
	//if(__DEBUG) printf_stone("read ADS1299 ID\r\n");
	uint8_t ret=0;
	uint8_t tmp_TXData[20],tmp_RXData[20];
	
	ADS1294_START_L;
	
	//hard reset
	ADS1299_RESET();

	HAL_Delay(100);
			
	//read adc ID
	ADS1294_Init();	
	do{
		memset(tmp_TXData, 0, sizeof(tmp_TXData));
		tmp_TXData[0] = RREG|ID_ADDR;
		tmp_TXData[1] = 0x01; 
		ADS1294_CS_L;
		ret=HAL_SPI_TransmitReceive(&hspi1, tmp_TXData, tmp_RXData, 4, 1000);
		ADS1294_CS_H;
		printf("ADS1294 ID = %02x %02x %02x %02x\r\n", tmp_RXData[0], tmp_RXData[1], tmp_RXData[2], tmp_RXData[3]);
		//printf("ret %d \r\n",ret);
		//HAL_Delay(100);
	}while(tmp_RXData[2]!=0x90);
	HAL_Delay(100);
	
		
	ADS1294_START_H; //bypass adc1

	//RDATAC
	memset(tmp_TXData, 0, sizeof(tmp_TXData));
	tmp_TXData[0] = RDATAC;
	ADS1294_WRITE(tmp_TXData, 1);
//	ADS1294_WRITE( tmp_TXData, 1);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart1, &Res, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
	g_time_seq = 0;
	g_seq = 0;
	adc1_data_ready = 0;

	memset(g_adc1_buf, 0, sizeof(g_adc1_buf));
	HAL_Delay(1);
	ADS1294_PowerOnInit();
	HAL_Delay(1);
	 HAL_Delay(1500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if 0
		if(adc1_data_ready == 1)
		{
			memset(g_pkt_buf, 0, sizeof(g_pkt_buf));
			g_pkt_buf[0] = 0xFD;
			g_pkt_buf[1] = 0xA6;
			g_pkt_buf[2] = PKT_LENGTH; //pkt length, 8 daolian=30, 16 daolian=54, 24 daolian=78, 32 daolian=102
			g_pkt_buf[3] = 0x11; //ECG和gsensor的区别
			g_pkt_buf[4] = CMD_BYTE; //cmd bit7-4 0000 1ch 0001 2ch, bit3 0 16bit 1 24bit, bit2-0 000-500hz 001-1khz 010-2khz
			
			g_pkt_buf[PKT_LENGTH-7] = 0x55; //DEV_ID
			g_time_seq++;
			g_pkt_buf[PKT_LENGTH-6] = g_time_seq>>24;
			g_pkt_buf[PKT_LENGTH-5] = g_time_seq>>16;
			g_pkt_buf[PKT_LENGTH-4] = g_time_seq>>8;
			g_pkt_buf[PKT_LENGTH-3] = g_time_seq;
			g_seq++;
			g_pkt_buf[PKT_LENGTH-2] = g_seq>>8;
			g_pkt_buf[PKT_LENGTH-1] = g_seq;
			//sendDataTo_device(&huart1, g_pkt_buf, PKT_LENGTH);
			__printf(&huart1, g_pkt_buf, PKT_LENGTH);

			adc1_data_ready = 0;
			//HAL_Delay(1);
		}
#else
		if(adc1_data_ready == 1)
		{
			adc1_data_ready = 0;
			memset(g_pkt_buf, 0, sizeof(g_pkt_buf));
			g_pkt_buf[0] = 0xFD;
			g_pkt_buf[1] = 0xA6;
			g_pkt_buf[2] = PKT_LENGTH; //pkt length, 8 daolian=30, 16 daolian=54, 24 daolian=78, 32 daolian=102
			g_pkt_buf[3] = 0x11; //ECG和gsensor的区别
			g_pkt_buf[4] = CMD_BYTE; //cmd bit7-4 0000 1ch 0001 2ch, bit3 0 16bit 1 24bit, bit2-0 000-500hz 001-1khz 010-2khz
			
			memcpy(&g_pkt_buf[5], g_adc1_buf, 15);
			for(int i=0; i<2; i++)
			{
				g_pkt_buf[5+i*3] ^= 0x80;
			}	
	
			g_pkt_buf[5]=g_pkt_buf[5+1*3];
			g_pkt_buf[6]=g_pkt_buf[5+1*3+1];		
			g_pkt_buf[7]=g_pkt_buf[5+1*3+2];

			g_pkt_buf[9]=0;
			g_pkt_buf[10]=((g_adc1_buf[0]<<4)|(g_adc1_buf[1]>>4))&0x01;
			
			g_pkt_buf[11]=0;
			g_pkt_buf[12]=((g_adc1_buf[1]<<4)|(g_adc1_buf[2]>>4))&0x01;
			
			if(g_pkt_buf[10]==0 &&g_pkt_buf[12]==0)
			{

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, LED_G, GPIO_PIN_RESET);
			}				
			else 
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, LED_R, GPIO_PIN_RESET);
			}
				
			
//			g_pkt_buf[5]=0;
//			g_pkt_buf[5+1]=((g_adc1_buf[0]<<4)|(g_adc1_buf[1]>>4))&0x01;
//			g_pkt_buf[5+2]=0;
//			
//			g_pkt_buf[5+2*3]=0;
//			g_pkt_buf[5+2*3+1]=((g_adc1_buf[1]<<4)|(g_adc1_buf[2]>>4))&0x01;
//			g_pkt_buf[5+2*3+2]=0;
//			
//			g_pkt_buf[5+12*3+2]=g_pkt_buf[5+1*3+2];
//			g_pkt_buf[5+12*3+1]=g_pkt_buf[5+1*3+1];
//			g_pkt_buf[5+12*3+0]=g_pkt_buf[5+1*3];
//			
//			g_pkt_buf[5+12*3+2] |= 0x01;
//		
//			g_pkt_buf[PKT_LENGTH-7] = 0x55; //DEV_ID
//			g_time_seq++;
//			g_pkt_buf[PKT_LENGTH-6] = g_time_seq>>24;
//			g_pkt_buf[PKT_LENGTH-5] = g_time_seq>>16;
//			g_pkt_buf[PKT_LENGTH-4] = g_time_seq>>8;
//			g_pkt_buf[PKT_LENGTH-3] = g_time_seq;

			g_seq++;
			g_pkt_buf[PKT_LENGTH-2] = g_seq>>8;
			g_pkt_buf[PKT_LENGTH-1] = g_seq;
			
//			sendDataTo_device(&huart1, g_pkt_buf, PKT_LENGTH);
//			__printf(&huart1, g_pkt_buf, PKT_LENGTH);
			g_mstimer++;
			if(g_mstimer>2000)
				g_ContiueSend=1;
			if(g_ContiueSend==1)
				sendDataTo_device(&huart1, g_pkt_buf, PKT_LENGTH);
			
			
		}
		
		if(send_flag)
		{
			send_flag=0;
//			__printf(&huart1, g_pkt_buf, PKT_LENGTH);
			sendDataTo_device(&huart1, g_pkt_buf, PKT_LENGTH);
		}

#endif
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t tmp_TXData_1[64],tmp_RXData_1[64];
	
	memset(tmp_TXData_1, 0, sizeof(tmp_TXData_1));
	
		//AD2 READY
	if(GPIO_Pin == GPIO_PIN_0)
  {
		if(adc1_data_ready == 0)
		{
			memset(g_adc1_buf, 0, sizeof(g_adc1_buf));
			memset(tmp_RXData_1, 0, sizeof(tmp_RXData_1));
			ADS1294_CS_L;
			HAL_SPI_TransmitReceive(&hspi1, tmp_TXData_1, tmp_RXData_1, 15, 1000);	
			ADS1294_CS_H;
			//memcpy(&g_pkt_buf[29], &tmp_RXData[3], 24);
			memcpy(g_adc1_buf, tmp_RXData_1, 15);
			adc1_data_ready = 1;
		}
	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int ret=0 ;
	if(huart == &huart1)
	{
		
//		do
//		{
//			ret = HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);  
//		}while(ret != HAL_OK);
		
		if(Res==0xaa)
		{
		    send_flag=1;
		}		
		g_mstimer=0;
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
