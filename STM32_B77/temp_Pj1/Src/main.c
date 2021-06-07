/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;

TIM_HandleTypeDef htim4;

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};

/* USER CODE BEGIN PV */
#define LED_INTERVAL 500  		 // �������� ���������� ��������� �����������
#define SYS_INTERVAL 500

u64 STM32_VERSION = 0x070620211017;//����� ������ �������� 12-02 ����� � 24-05-2021 ����
u32 IP_my=0;
u16 PORT_my=0;

u32 TIMER_CONTROL_SYS;   		//���������� ������� �������� ��������� ������� ����� �������

volatile  uint16_t adcBuffer[5]; // Buffer for store the results of the ADC conversion
float adc_ch[12];

uint8_t transmitBuffer[32];
uint8_t receiveBuffer[32];

uint32_t TIMER1;
u32 TIMER_BP_PWM;
volatile u32  SysTickDelay; 

u32 FLAG_T1;
u32 FLAG_T2;
u8  FLAG_FAPCH_ON;

uint8_t RX_uBUF[1];
uint8_t RX2_uBUF[1];

//----------------UART1---------------------------------
unsigned int timer_DMA2;
u8 flag_pachka_TXT; //
uint16_t  text_lengh;
uint8_t text_buffer[Bufer_size];

volatile char          rx_buffer1[RX_BUFFER_SIZE1];
volatile unsigned int rx_wr_index1,rx_rd_index1,rx_counter1;
volatile u8  rx_buffer_overflow1;
//----------------UART2---------------------------------
unsigned int timer_DMA1_Stream6;
u8 flag_pachka_TXT2; //
uint16_t  text_lengh2;
uint8_t text_buffer2[Bufer_size2];

volatile char          rx_buffer2[RX_BUFFER_SIZE2];
volatile unsigned int rx_wr_index2,rx_rd_index2,rx_counter2;
volatile u8  rx_buffer_overflow2;

char sr[BUFFER_SR+1];
unsigned char lsr;
unsigned char lk;
unsigned int led_tick;

char  strng[buf_IO];
char      InOut[BUFFER_SR+1];
char      Word [buf_Word];    //
char DATA_Word [buf_DATA_Word];    //
char DATA_Word2[buf_DATA_Word];    //
 
char Master_flag; // 
char lsym;
char  sym;
char flag;
char    NB;
char Adress;  //
char packet_sum;
char crc,comanda;
      
char In_data[BUF_STR];
char ink1; //
char data_in;

u16 lenght;
u16 SCH_LENGHT_PACKET;
	
unsigned     int index1;
unsigned     char crc_ok;
unsigned     char packet_ok;
unsigned     char packet_flag;
unsigned     int indexZ; 
unsigned     int index_word;
unsigned     int index_data_word;
unsigned     int index_data_word2;
unsigned     int lenght_data;//
unsigned     char data_flag;
unsigned     char data_flag2;
unsigned     char FLAG_lenght;//
unsigned     int sch_lenght_data;
unsigned     char FLAG_DATA;
unsigned char FLAG_CW;
float time_uart; //
unsigned char flag_pcf;
char lsym1;
char pack_ok1;
char pack_sum1;
char sym1;


u32 sch_rx_byte;
  
u8  Adress_ATT1;
u8  Adress_ATT2;
u8  Adress_ATT3;
u8  Adress_ATT4;
  
u8  Adress_SWITCH;
u8  Adress_KONTROL_WIRE;
u8  Adress_KONTROL_1;
u8  Adress_KONTROL_2;

u8 PWRDN_DAC1;
u8 PWRDN_DAC2;
u8 PWRDN_ADC1;
u8 PWRDN_ADC2;

u8 RESET_DAC1;
u8 RESET_DAC2;
u8 RESET_ADC1;
u8 RESET_ADC2;

u8 EVENT_INT0=0;
u8 EVENT_INT1=0;
u8 EVENT_INT2=0;
u8 EVENT_INT3=0;
u8 EVENT_INT4=0;
u8 EVENT_INT5=0;
u8 EVENT_INT6=0;
u8 EVENT_INT7=0;
u8 EVENT_INT12;
u8 FLAG_DMA_ADC=0;

u32 TEST_LED     =0;
u64 TIME_SYS     =0;//���������� ������� ��������� ����� � ������������
u32 TIME_TEST    =0;
u32 ID_CMD       =0;//���������� ������� ������� ID ����� ��������� 
u32 FLAG_T1HZ_MK =0;//��� ���� ���������� ��� � ������� ���� ������ ���� ���� ��������� �����
u32 TIMER_T1HZ_MK=0;//��� ����������� ������ ��� �������� ������� ��������� �����
u8  PWR_CHANNEL=255;

//����� �����������
u8  LED_ISPRAV_AC=0;
u8  LED_PROGR    =0;
u8  LED_OFCH     =0;
u8  LED_SINHR    =0;
u8  LED_LS       =0;
u8  LED_ISPR_J330=0;
u8  LED_OTKL_AC  =0;
u8  LED_TEMP	 =0;
u32 TIMER_LED    =0;

u8 D_TEMP[4];
int TMP_v=0;

u8 FLAG_TIME_OF_WORK_WRITE=0; //���� ��������� ������ �� ���� ��� �������� �����
u32 TIME_OF_SECOND=0;         //c������ ������ � ������ ������
u32 TIME_OF_WORK=0;           //����� ��������� ����� � �������� �����
SYS_STATE_BOARD B077;         //��������� ���������� ��������� ����� �330
//------------------------------------------------------
u8  FLAG_ASQ_TEST_485 =0;    //���� ������ �� ������ ����� �� 485 ����
u8  FLAG_ASQ_TEST_JTAG=0;    //���� ������ �� ������ ����� �� SPI ����
u8  FLAG_ASQ_TEST_SPI =0;    //���� ������ �� ������ ����� �� JTAG ����

POINTER * PNT[PNT_BUF];  //������ ���������� �� ���������� ������

POINTER  POINTER_TEST_485_REQ;  // 0  ���� ����������� �������� ������ �� ������ �� ���� 485 
POINTER  POINTER_TEST_485;      // 1  ���� ����������� ���� �������� ���� 485
POINTER  POINTER_TEST_SPI_REQ;  // 2  ���� ����������� �������� ������ �� ������ �� ���� SPI 
POINTER  POINTER_TEST_SPI;      // 2  ���� ����������� ���� �������� ���� SPI
POINTER  POINTER_TEST_JTAG_REQ;  // 3  ���� ����������� �������� ������ �� ������ �� ���� JTAG 
POINTER  POINTER_TEST_JTAG;     // 3  ���� ����������� ���� �������� ���� JTAG
POINTER  POINTER_ADR_COLLECT;   // 4  ���� ����������� ���� ������� � ����������
POINTER  POINTER_MASTER_IP0_SETUP;     // 5  ���� ����������� ������� IP0 ������� �������� 072 �� ���������
POINTER  POINTER_SLAVE_IP0_SETUP;
POINTER  POINTER_MASTER_IP1_SETUP;     // 6  ���� ����������� ������� IP1 ������� �������� 072 �� ���������
POINTER  POINTER_SLAVE_IP1_SETUP;
POINTER  POINTER_DEST_MASTER_IP0_SETUP;// 7  ���� ����������� ������� DEST_IP0 ������� �������� 072 �� ���������
POINTER  POINTER_DEST_SLAVE_IP0_SETUP;
POINTER  POINTER_DEST_MASTER_IP1_SETUP;// 8  ���� ����������� ������� DEST_IP1 ������� �������� 072 �� ���������
POINTER  POINTER_DEST_SLAVE_IP1_SETUP;
POINTER  POINTER_ADR_REQ;       // 9  ���� ������� ������ �� ������ �� ���������, ����������� ����� ������������� 
POINTER  POINTER_ETHERNET_RERUN;// 10 ���� �� �������� ���������� ������� ��� �������������������� ����� ����� 072

u8  ADR_SLAVE [8];          //��� ������ ������ ������ �� ����� ��
u8  NUMBER_OF_B072;         //����� ������ �072 �� ���������
u32 TIMER_TIMEOUT=0;        //������ �������� �������� �������

u8 SCH_TST1=0;//������� ����� �������
u8 SCH_TST2=0;
u8 SCH_TST3=0;
//-----------------------------------------------------------------------------
//                           �������� �������� ���������� � ���������
/* USER CODE END PV */

 Frame INVOICE[quantity_SENDER];//��������� ��������� � ��������� ���, �� ����� ������������� ���������
 Frame FRM1[quantity_SENDER];	//�������� �����, �� ����� ������������� ���������
 SERVER SERV1;					//��������� "���������"
 ID_SERVER ID_SERV1;			//��������� ��������� ��� "���������"
 CMD_RUN CMD1;
 ADR_SENDER ADDR_SNDR;			//��������� ������ ������ ������� ������������ 

u64 ADRES_SENDER_CMD=0; //��� ������ ����� ���������� ����������, ��� ����������� � ��� ���������
u8 FLAG_ADRES_SENDER_CMD=0;//���� ���� ��� ���� ���� ���������� ���������
//-----------------------------------------------------------------------------
//                       JTAG
#include "jtagtap.h"
#include "jtag_scan.h"
#include "jtag_devs.h"

extern jtag_dev_t jtag_devs[JTAG_MAX_DEVS+1];
extern int jtag_dev_count;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

void Initialization_wiznet(void);

/* USER CODE BEGIN PFP */
u8 START_BP=0;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  
  /*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;// ��� ��� ������ �� ����������� ��������� ������
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;//
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  */
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;// ��� ��� ������ �� ������ 16 ���
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  
  /*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;// ��� ��� ������ �� ������ 8 ���
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
/*
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
 

  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

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

static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

   /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_SLAVE;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  huart1.Init.BaudRate = 256000;//115200
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
    /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
//  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins :*/
  GPIO_InitStruct.Pin   = GPIO_PIN_9|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins :*/
  GPIO_InitStruct.Pin   = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB9 */
  GPIO_InitStruct.Pin   = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 PD12 PD14 
                           PD15 PD0 PD1 PD4 
                           PD7 */
  GPIO_InitStruct.Pin   = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
    /*Configure GPIO pins :*/
  GPIO_InitStruct.Pin  = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins :*/
  GPIO_InitStruct.Pin  = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin :*/
  GPIO_InitStruct.Pin  = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
    /*Configure GPIO pin :  */
  GPIO_InitStruct.Pin  = GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
    /*Configure GPIO pins :*/
  GPIO_InitStruct.Pin   = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
      /*Configure GPIO pin :  */
  GPIO_InitStruct.Pin  = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
        /*Configure GPIO pin :  */
  GPIO_InitStruct.Pin  = GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin :  */
  GPIO_InitStruct.Pin  = GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin  = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
 
   /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin  = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
    /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin  = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;//GPIO_MODE_IT_FALLING
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim4.Init.AutoReloadPreload = TIM_MASTERSLAVEMODE_DISABLE;//TIM_AUTORELOAD_PRELOAD_ENABLE
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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


void spi4send32 (u32 d) //32 ����
{
	u8 a1;
	u8 a2;
	u8 a3;
	u8 a4;
	
	u8 b;

  a1 = (d >> 24)&0xff;
  a2 = (d >> 16)&0xff;
  a3 = (d >>  8)&0xff;
  a4 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi4, &a1, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi4, &a2, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi4, &a3, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi4, &a4, &b,1, 5000);  
}

u8 spi4send8 (u8 d) //8 ���
{
	u8 a1;
	u8  b;

  a1 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi4, &a1, &b,1, 5000); 
  return b; 
}

u8 spi5send8 (u8 d) //8 ���
{
	u8 a1;
	u8  b;

  a1 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi5, &a1, &b,1, 5000); 
  return b; 
}

void Delay( unsigned int Val)  
{  
   SysTickDelay = Val;  
   while (SysTickDelay != 0) {};  
}
//----------------------------------------------
volatile void delay_us( uint32_t time_delay)
{	
	time_delay=time_delay*10;
    while(time_delay--)	;
} 
//-------------------------------------------
unsigned int leng ( char *s)
{
  unsigned  char i=0;
  while ((s[i]!='\0')&&(i<120)) { i++;}
  return i;
}

void Transf(const char* s)  // ��������� �������� ������ �������� � ����
{
  u32 l=0;
  u32 i=0;
         
  if ((flag_pachka_TXT==0) )
  {
    l=strlen(s);
    if ((text_lengh+l)>Bufer_size-5) text_lengh=0u;
    for (i=text_lengh;i<(text_lengh+l);i++) text_buffer[i]=s[i-text_lengh];
    text_lengh=text_lengh+l;
  } 
}

void Transf2(const char* s)  // ��������� �������� ������ �������� � ����
{
  u32 l=0;
  u32 i=0;
         
  if ((flag_pachka_TXT2==0) )
  {
    l=strlen(s);
    if ((text_lengh2+l)>Bufer_size2-5) text_lengh2=0u;
    for (i=text_lengh2;i<(text_lengh2+l);i++) text_buffer2[i]=s[i-text_lengh2];
    text_lengh2=text_lengh2+l;
  } 
}

void itoa(int val,  char *bufstr, int base) //
{
    u8 buf[32] = {0};
    int i = 30;
    int j;
    for(; val && i ; --i, val /= base)
        buf[i] = "0123456789abcdef"[val % base];
    i++; j=0;
    while (buf[i]!=0){ bufstr[j]=buf[i]; i++; j++;}
}

void f_out (char s[],float a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%.2f",a);
   Transf(strng);
   Transf ("\r\n");
}

void d_out (char s[],int a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%d",a);
   Transf(strng);
   Transf ("\r");
}

void u_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
   Transf ("\r");
}

void un_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
}

void nu_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
   //Transf ("\r\n");
}

void x32_out (char s[],u32 a)
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);
   Transf ("\r\n");
}

void x_out (char s[],u32 a)//���� u64 
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);
   Transf ("\r\n");
}

void xn_out (char s[],u32 a)//���� u64 
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);   
}

void in_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%d",c);
   Transf (" ");
   Transf(strng);   
}

void i_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%d",c);
   Transf (" ");
   Transf(strng);
   Transf ("\r\n");   
}

void hn_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%X",c);
   Transf (" ");
   if (c<0x10) 
   {
   Transf ("0");
   } 
   Transf(strng);   
}

void h_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%X",c);
   Transf (" ");
   if (c<0x10) 
   {
   Transf ("0");
   } 
   Transf(strng);  
   Transf ("\r\n");   
}

void u64_out (char s[],u64 a)
{
   u64 z=0;	
   Transf (s);
   z=a>>32;
   a=a&0xffffffff;
   sprintf (strng,"%u",z);
   Transf(strng);
   sprintf (strng,"%u",a);
   Transf(strng);
   Transf ("\r");
}

void un64_out (char s[],u64 a)
{
   u64 z=0; 
   Transf (s);
   z=a>>32;
   a=a&0xffffffff;
   sprintf (strng,"%u",z);
   Transf(strng);
   sprintf (strng,"%u",a);
   Transf(strng);
}

//������� ���������� ����� �������� ��� IP ������
void xun_out (char s[],u32 a)//���� u64 
{
   u8 tmp0=0;
   Transf (s);
   tmp0=(a>>24)&0xff;
   sprintf (strng,"%u",tmp0);
   Transf(strng);   
   Transf(".");
   tmp0=(a>>16)&0xff;
   sprintf (strng,"%u",tmp0);
   Transf(strng);   
   Transf(".");
   tmp0=(a>> 8)&0xff;
   sprintf (strng,"%u",tmp0);
   Transf(strng);   
   Transf(".");
   tmp0=(a>> 0)&0xff;
   sprintf (strng,"%u",tmp0);
   Transf(strng);   
}

void ARRAY_DATA (u32 a)  //��������� 32 ������ ���������� �� ����� � ��� � ������������ ������
{
	D_TEMP[0]=a>>24;
	D_TEMP[1]=a>>16;
	D_TEMP[2]=a>> 8;
	D_TEMP[3]=a;
}

void UART_IT_TX (void)
{
 uint16_t k;
/*
if ((flag_pachka_TXT==0)&&(text_lengh>1u))
{ 
    k = text_lengh;
  	HAL_UART_Transmit_IT(&huart1,text_buffer,k);
    text_lengh=0u;  //��������� �������� ������ 
    flag_pachka_TXT=1; //������������� ���� ��������
  }
  */
}
 
void UART_DMA_TX (void)
{
 uint16_t k;

if (HAL_UART_GetState(&huart1)!=HAL_UART_STATE_BUSY_TX )
	{
		if ((flag_pachka_TXT==0)&&(text_lengh>1u)&&(timer_DMA2>250))
		 {
			k = text_lengh;
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)text_buffer,k);
			text_lengh=0u;  //��������� �������� ������ 
			flag_pachka_TXT=1; //������������� ���� ��������
			timer_DMA2=0;
		  }
	}	
} 

void UART_DMA_TX2 (void)
{
 uint16_t k;

if (HAL_UART_GetState(&huart2)!=HAL_UART_STATE_BUSY_TX )
	{
		if ((flag_pachka_TXT2==0)&&(text_lengh2>1u)&&(timer_DMA1_Stream6>250))
		 {
			NRE_RS485(1);//��������� ����� 485 �� ���� 
			DE_RS485(1);//�������� ����� 485 �� ��������

			k = text_lengh2;
			HAL_UART_Transmit_DMA(&huart2,(uint8_t *)text_buffer2,k);
			text_lengh2=0u;  //��������� �������� ������ 
			flag_pachka_TXT2=1; //������������� ���� ��������
			timer_DMA1_Stream6=0;
		  }
	}	
} 

void spisend32 (u32 d) //32 ����
{
	u8 a1;
	u8 a2;
	u8 a3;
	u8 a4;
	
	u8 b;
  //HAL_SPI_TransmitReceive(&hspi3, &address, &data, sizeof(data), 5000);

  a1 = (d >> 24)&0xff;
  a2 = (d >> 16)&0xff;
  a3 = (d >>  8)&0xff;
  a4 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi3, &a1, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi3, &a2, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi3, &a3, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi3, &a4, &b,1, 5000);  
}

u8 spisend8 (u8 d) //8 ���
{
	u8 a1;
	u8  b;
  //HAL_SPI_TransmitReceive(&hspi3, &address, &data, sizeof(data), 5000);

  a1 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi3, &a1, &b,1, 5000); 
  return b; 
}

u64 FPGA_rSPI (u8 size,u8 adr)
{
   u64 d[8];
   u8 i,k;
   u8 adr_a=0;
   u64 value;
   
   k=size/8;
   adr_a=adr;
  
//   delay_us(1);
 
   spi5send8 (adr_a); //
   for (i=0;i<(size/8);i++) d[i] = spi5send8 (0x00);  //��������� ������
//   delay_us(1);  
   
   if (k==1) value =   d[0];
   if (k==2) value =  (d[0]<< 8)+ d[1];
   if (k==3) value =  (d[0]<<16)+(d[1]<< 8)+ d[2];
   if (k==4) value =  (d[0]<<24)+(d[1]<<16)+(d[2]<< 8)+ d[3];
   if (k==5) value =  (d[0]<<32)+(d[1]<<24)+(d[2]<<16)+(d[3]<< 8)+ d[4];
   if (k==6) value =  (d[0]<<40)+(d[1]<<32)+(d[2]<<24)+(d[3]<<16)+(d[4]<< 8)+ d[5];
   if (k==7) value =  (d[0]<<48)+(d[1]<<40)+(d[2]<<32)+(d[3]<<24)+(d[4]<<16)+(d[5]<< 8)+ d[6];
   if (k==8) value =  (d[0]<<56)+(d[1]<<48)+(d[2]<<40)+(d[3]<<32)+(d[4]<<24)+(d[5]<<16)+(d[6]<<8)+d[7];

   return value;
}

u32 FPGA_wSPI (u8 size,u8 adr,u64 data)
{
   u8 d[8];
   u8 i,k;
   
   k=size/8;
    
   if (k==1)  d[3]=data;
   
   if (k==2) {d[2]=data;
        d[3]=data>>8;
        }
        
   if (k==3) {d[1]=data;
        d[2]=data>>8;
        d[3]=data>>16;
        }
		
   if (k==4) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
        }
		
   if (k==5) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
        }
		
	if (k==6) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
        }
	if (k==7) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
		d[6]=data>>48;
        }
	if (k==8) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
		d[6]=data>>48;
		d[7]=data>>56;
        }
  
  // CS_5_MK(0);
   spi5send8 (adr|0x80); //������� ������
   k--;
   for (i=0;i<(size/8);i++) spi5send8 (d[k-i]); //����� ������
   //CS_5_MK(1);

   return 0;
}  
//-----------------------------------------------------------------
//             �������� ����� "���������"
void PRINT_SERV (void)
{
	u32 i=0;
	u32 j=0;
	Transf("\r\n");
	
	for (i=0;i<SIZE_SERVER;i++)
	{
		if (j<4) 
		{
			if (SERV1.MeM[i]<10) xn_out("  ",SERV1.MeM[i]); 
			else				 xn_out(" " ,SERV1.MeM[i]);
		    if (j==3) {Transf("\r\n");j=0;}	else j++;		
		}
	}	
	Transf("\r\n");	
}


void PRINT_SERV_ID (void)
{
	u32 i=0;
	
	Transf("\r\n");
	x_out("N_sch    :",ID_SERV1.N_sch);
	
	for (i=0;i<SIZE_ID;i++)
	{
		if (ID_SERV1.INDEX[i]!=0xffffffff) 
		{		
			Transf("\r\n");	
			u_out("INDEX    :",ID_SERV1.INDEX      [i]);
			x_out("CMD_TYPE :0x",ID_SERV1.CMD_TYPE [i]);
			x_out("SENDER_ID:0x",ID_SERV1.SENDER_ID[i]);
			u_out("TIME     :",ID_SERV1.TIME       [i]);			
		}
	}		
}


//----------------------------------------------------------
void info ()
{

}

void BUS_485_TEST (u8 a)
{
   u8 Str[17];
   int n=0;

   Str[n++]=' ';   //0
   Str[n++]='~';   //1
   Str[n++]=0x30+a;//2
   Str[n++]=' ';   //3
   Str[n++]='r';   //4
   Str[n++]='s';   //5
   Str[n++]='4';   //6
   Str[n++]='8';   //7
   Str[n++]='5';   //8
   Str[n++]='_';   //9
   Str[n++]='t';   //10
   Str[n++]='e';   //11
   Str[n++]='s';   //12
   Str[n++]='t';   //13
   Str[n++]=';';   //14
   Str[n++]=';';   //15
   Str[n++]=0x00;  //16

   Transf2(Str);
   Transf("������ ���: ");
   Transf(Str);
   Transf("\r\n");
}

u32 crc_input=0u; 
u32 crc_comp=0u;
u8 Str[10];

u32 IO ( char* str)      // ������� ��������� ��������� ������
{

 unsigned int i=0;
	uint8_t z1=0;
  i = lenght;//������ �������� �����
  if (lenght==0) i = leng(str);
  lenght = 0;
 
  indexZ = 0;
  
  if ((time_uart>50u)||(SCH_LENGHT_PACKET>MAX_PL))
  {
	  //-------------------
		packet_flag=0; 
		//-------------------
		time_uart=0u;  //��������� �������� ���� ����
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //���� ������������ �����, ��������� ����� ����� ����������
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;
  }
  
  while (i>0u)   //���������� ��������� ������ � ������ ���������
  
  {  

	if ((str[indexZ]==0x7e)&&(packet_flag==0))// ���������� ������ ������
	  {  
		//-------------------
		packet_flag=1; 
		//-------------------
		time_uart=0u;  //��������� �������� ���� ����
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //���� ������������ �����, ��������� ����� ����� ����������
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;		
	  }

	 InOut[index1]=str[indexZ];
	 SCH_LENGHT_PACKET++;//������������ ������ ������
		 
	if (( InOut[index1]==';')&&(FLAG_DATA==0u)&&(packet_flag==1))  {packet_flag=0;packet_ok=1u;FLAG_CW=1u;}
    
	if (((InOut[index1]=='=')||(InOut[index1]==':'))&&(data_flag==0)) {data_flag=1u;FLAG_CW=1u;}

	if (( InOut[index1]=='.')&&(data_flag2==0)&&(FLAG_DATA==0))   {data_flag2=1u; FLAG_CW=1u;}
	
	if (( InOut[index1]=='$')&&(FLAG_lenght==0u)) {FLAG_lenght=2u;FLAG_CW=1u;}
    
	if ((index1>2u)&&(InOut[2]==' ')&&(FLAG_CW==0u)&&(FLAG_lenght<2u))  
            {
                             if   (data_flag!=1u) {Word[index_word]=InOut[index1];} // ��������� ��������� �����
                      
                             if  ((data_flag==1u)&&(data_flag2==0u))     DATA_Word[index_data_word]=InOut[index1];// ���������  ����� ������1
                             if  ((data_flag==1u)&&(data_flag2==1u))     DATA_Word2[index_data_word2]=InOut[index1]; // ���������  ����� ������2
                    
                             if  (data_flag!=1u)
                                     {if (index_word<buf_Word) index_word++;} 
                                   else 
                                            {
                                             if ((data_flag==1u)&&(data_flag2==0u)) if (index_data_word<buf_DATA_Word)  {index_data_word++;sch_lenght_data++;}
                                            
                                             if ((data_flag==1u)&&(data_flag2==1u)) if (index_data_word2<buf_DATA_Word) index_data_word2++;
                                            }
			}
	
		if ((FLAG_lenght==2u)&&(FLAG_CW==0u)) {lenght_data = (u8)(InOut[index1]);FLAG_lenght=1u;} //���������� ������ ������ ������ ����� ":"
	
		if ((sch_lenght_data<lenght_data)&&(FLAG_lenght==1u)) FLAG_DATA = 1u; else {FLAG_DATA = 0u;}
	 
		if (index1<BUFFER_SR)  index1++;
		if (indexZ <BUFFER_SR)  indexZ ++;
		i--;
		FLAG_CW=0u;	
  } 

if (packet_ok==1u) 
  {    
      if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   // �������� ������� ������� ������ - ������ ������
      if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   // �������� ������� ������� ������ - ������� ����������
 
if (crc_ok==0x3)  //��������� ������ ��������� ������� �������� ������ 
{
  if (strcmp(Word,"versiya")==0) //
   {
    Transf ("������ versiya\r\n"); 
    REQ_VERSIYA();       
   } else
if (strcmp(Word,"rs485_test_OK")==0) // ��������� ���� 485! ������ ������ �����.
   {
	 u_out("������ rs485_test_OK:",crc_comp);
	 Transf("���� 485 ���� ������� �������!\r\n");
	 FLAG_ASQ_TEST_485=1;
   } else	
if (strcmp(Word,"rs485_test")==0) // ��������� ���� 485! ������ ������ �����.
   {
	 u_out("������ rs485_test:",crc_comp);	 
     BUS_485_TEST (crc_comp);
   } else
 if (strcmp(Word,"rs485_help")==0) // ~0 spi_write:adr.code;
   {
	 u_out("������ rs485_help:",crc_comp);
	 Transf2("~0 help;");
   } else		
 if (strcmp(Word,"spi_write")==0) // ~0 spi_write:adr.code;
   {
	 crc_comp =atoi(DATA_Word);	//������ ����� - ������ ������ �� ���������
 	 crc_input=atoi(DATA_Word2);//������������ 32-������ ��� 
	 crc_comp=(crc_comp<<4)|0x2;
	 FPGA_wSPI (32,crc_comp,crc_input);
	 Transf("������ FPGA spi_write\r\n");
   } else
if (strcmp(Word,"JTAG_TST")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("������ JTAG_TST\r"    );
     Transf("\r"); 
	 TST ();
   } else	
if (strcmp(Word,"JTAG2_SCAN")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("������ JTAG_SCAN\r"    );
     Transf("\r"); 
	 //--------------------
	 /*
		 u8 list[2]; //������ ���� IR ��������� - �� ����� ������ ���� �� ����� ��������� �� ���� 
			list[0]=10;
			list[1]=10;
			list[2]=10;
			list[3]=10;
			list[4]=10;
			list[5]=10;
			list[6]=10;
			list[7]=10;
			list[1]=0;*/
//	 crc_comp=jtag_scan(list,crc_comp); 
	 crc_comp=jtag_scan(NULL,crc_comp); 
   } else
if (strcmp(Word,"JTAG1_SCAN")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("������ JTAG_SCAN\r"    );
     Transf("\r"); 
     JTAG_SCAN();
   } else
if (strcmp(Word,"JTAG_ID")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("������ JTAG_ID\r"    );
     Transf("\r");  
     ID_recive (crc_comp);
   } else		 
if (strcmp(Word,"BatPG")==0) //
   {
      u_out ("������ BatPG:",0); 
	  crc_comp=DS4520_read();
	  u_out("POWER_GOOD:",crc_comp);
   } else		
if (strcmp(Word,"time")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
//    u_out ("������ time:",crc_comp); 
//	  u_out("TIME_SYS:",TIME_SYS);
    un64_out("[",TIME_SYS);Transf("]\r\n");
//	  u_out("TIMER1  :",TIMER1);
//	  u_out("TIME_TEST:",TIME_TEST);
	  
   } else	
	
if (strcmp(Word,"mem")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ mem:",crc_comp); 
	     PRINT_SERV();
   } else
if (strcmp(Word,"MSG")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ MSG:",crc_comp); 
	  MSG_SHOW ();
   } else
 if (strcmp(Word,"ID_SERV")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ ID_SERV:",crc_comp); 
	  STATUS_ID (&ID_SERV1);
   } else
	 if (strcmp(Word,"nss")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ nss:",crc_comp); 
	  NSS_4(crc_comp);
   } else
	 if (strcmp(Word,"eth_init")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ eth_init:",crc_comp); 
	  Set_network();
   } else 
if (strcmp(Word,"sys_info")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ sys_info:",crc_comp); 
      SYS_INFO(crc_comp);
   } else	   
 if (strcmp(Word,"i2c_adr")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ i2c_adr:",crc_comp); 
      u_out("i2c:",HAL_I2C_IsDeviceReady(&hi2c1, crc_comp,1, 1000));
   } else	
 if (strcmp(Word,"start")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ start:",crc_comp); 
      START_BP=crc_comp;
   } else
 if (strcmp(Word,"pwrdn")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ pwrdn:",crc_comp); 
      PWDN_4(crc_comp);
   } else
 if (strcmp(Word,"ADC_test")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ ADC_test:",crc_comp); 
      ADC_test ();
   } else
if (strcmp(Word,"help")==0)                     
   {
     Transf ("������ help\r\n"    );
     Menu1(0);
   } else
//-------------------i2c----------------	   
if (strcmp(Word,"i2c_status")==0)                     
   {
     Transf ("������ i2c_status\r\n"    );
	 crc_comp =atoi(DATA_Word);	
     if (HAL_I2C_IsDeviceReady(&hi2c1,crc_comp,1,10) ==HAL_OK) Transf("i2c_OK\r\n");else Transf("i2c_error\r\n");	 
   } else
if (strcmp(Word,"i2c_transmit")==0)                     
   {
     Transf ("������ i2c_transmit\r\n"    );
	 crc_comp =atoi(DATA_Word);	
 	 crc_input=atoi(DATA_Word2);
	 z1=crc_input;
     HAL_I2C_Master_Transmit(&hi2c1,crc_comp,&z1,1,10);	 
   } else
if (strcmp(Word,"i2c_recive")==0)                     
   {
     Transf ("������ i2c_recive\r\n"    );
	   crc_comp =atoi(DATA_Word);	
     HAL_I2C_Master_Receive(&hi2c1,crc_comp,&z1,1,10);
	 x_out("data=",z1);	 
   } else 
if (strcmp(Word,"DS4306_read")==0)                     
   {
     Transf ("������ DS4306_read\r\n");
     crc_comp =atoi(DATA_Word);	//������ ����� - ������ �-�
 	 crc_input=atoi(DATA_Word2);//������ ����� ������ ��������	 
     z1=DS4306_read (crc_comp,crc_input);  //
	 x_out("code:",z1);
   } else    
if (strcmp(Word,"tca")==0)                     
   {
     Transf ("������ tca_read:\r\n");
     crc_comp =atoi(DATA_Word);	//������ ����� - ������ �-�
 	 crc_input=atoi(DATA_Word2);//������ ����� ������ ��������	 
     z1=tca9534_read (crc_comp,crc_input);  //
	 x_out("code:",z1);
   } else 
if (strcmp(Word,"tca6424a_read")==0)                     
   {
     Transf ("������ tca_read:\r\n");
     crc_comp =atoi(DATA_Word);	//������ ����� - ����� ���������� �� �����
     tca6424a_from_port (crc_comp,Str);
	 x_out("code[0]:",Str[0]);
	 x_out("code[1]:",Str[1]);
	 x_out("code[2]:",Str[2]);
   } else 
if (strcmp(Word,"LED_DD44")==0)                     
   {
	   crc_comp =atoi(DATA_Word);	
      Transf ("������ LED_DD44\r\n"); 
      LED_DD44 (crc_comp);
   } else     
if (strcmp(Word,"info")==0)                     
   {
     Transf ("������ info\r\n"    ); 
     info();
   } else  
if (strcmp(Word,"RSC")==0)                     
   {
     Transf ("������ RSC\r\n"    );
     REF_SFP_CONTROL (1);
   } else
if (strcmp(Word,"menu")==0)                     
   {
     Transf ("������ menu\r\n"    );
     Menu1(0);
   } else
 if (strcmp(Word,"Show_worktime")==0) //
   {
    u_out ("������ Show_worktime:",TIME_OF_WORK); 
   } else
  if (strcmp(Word,"MSG_ADR")==0) //������ ��������� � �������� �� ���������, �����-�� ������ ��������������� � ������ ���������������
   {
    crc_comp =atoi(DATA_Word); 
    u_out ("������ MSG_ADR:",crc_comp); 
    FUNC_FLAG_UP (&POINTER_ADR_COLLECT,100);//������ ���������� ������ ��� ������ ������ �� ���������
   }
 } 
	  for (i=0u;i<buf_Word;i++)               Word[i]     =0x0;
      for (i=0u;i<buf_DATA_Word;  i++)   DATA_Word[i]     =0x0;
      for (i=0u;i<buf_DATA_Word;  i++)  DATA_Word2[i]     =0x0;  
      for (i=0u;i<BUFFER_SR;i++)  
      {
        InOut[i]     =0x0;
      }  
      
	  time_uart=0;  //��������� �������� ���� ����
      packet_flag=0; 
      index1=0u; 
      crc_ok=0; 
      i=0;
      packet_ok=0; 
      index_word=0u; 
      index_data_word=0u;
      data_flag=0;
      index_data_word2=0u;
      data_flag2=0;
      indexZ =0u;
      FLAG_lenght=0u;
      lenght_data=0u;
      sch_lenght_data=0u;
      FLAG_CW = 0u; //���� ������������ �����, ��������� ����� ����� ����������
      FLAG_DATA = 0u;	  
      	  
      DATA_Word [0]=' ';
      DATA_Word2[0]=' ';
	  SCH_LENGHT_PACKET=0;
  }

  if ((packet_ok==1)&&(crc_ok==0x1))     //��������� ������ ��������� ������� �������� �����

  {
    
    if (Master_flag==0)

      {            
         
      }
  }  
  return  crc_input;         
} 

void Menu1(char a) 
 {
//***************************************************************************
    int i;  
 
  for (i=0;i<20;i++) Transf("\r");    // ������� ���������
  for (i=0; i<20; i++) Transf ("-");  // ����� �����������
  Transf("\r");
  Transf("..........Terminal 330 �������.........\r\n");
  Transf("\r");
  Transf("MENU :\r");
  Transf("-------\r");
  Transf("����������� ��������� �������:\r");
  Transf("~ - ��������� ����\r");
  un_out("",Adress-0x30);
  Transf(" - ����� ��������\r");
  Transf(";- ����� ����� \r");
  Transf(".............. \r");
  Transf("---------------------------------------------\r\n");
  Transf("PORT:");
  un_out("",PORT_my);
  Transf("       - ����� ����� �����\r");
  Transf("IP  :");
  xun_out("",IP_my);
  Transf(" - IP �����    �����\r"); 
  Transf("~0 help; - ������� ����\r");
  Transf("~0 info; - ���������� \r");
  Transf("~0 rs485_test; - \r");
  Transf("~0 JTAG_TST;   - ������������ JTAG\r");
  Transf("~0 JTAG2_SCAN; - ������������ ���� JTAG\r");
  Transf("~0 JTAG1_SCAN; - ������������ ���� JTAG\r");
  Transf("~0 JTAG_ID; \r");
  Transf("~0 time;       - ����� ���������� �������\r");
  Transf("~0 sys_info; \r");
  Transf("~0 start:1; \r");
  Transf("~0 lmk_sync; - sync �� LMK\r");
  Transf("~0 init_lmk; - init �� LMK\r");
  Transf("-------------------------------------------\r");
  Transf("\r");
  Transf("\r");
  Transf("++++++++++++++++++++\r");
  Transf("\r");
  Transf("\r");
  //for (i=0; i<64; i++) zputs ("*",1);  // ����� �����������
  //for (i=0;i<10;i++) puts("\r",1);  // ������� ���������
  Transf("\r");
  REQ_VERSIYA(); //������� ������ ��������
  //*******************************************************************************
}


char getchar1(void)
{
   uint8_t data;
   while (rx_counter1 == 0);
   data = rx_buffer1[ rx_rd_index1++ ];
   if (rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1 = 0;
    --rx_counter1;
    return data;
}

char getchar2(void)
{
   uint8_t data;
   while (rx_counter2 == 0);
   data = rx_buffer2[ rx_rd_index2++ ];
   if (rx_rd_index2 == RX_BUFFER_SIZE2) rx_rd_index2 = 0;
    --rx_counter2;
    return data;
}
      
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

		//-------------------------  
   		rx_buffer1[rx_wr_index1++]= (uint8_t) (RX_uBUF[0]& 0xFF); //��������� ������ � �����, ������������� ����� ������
   		if ( rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0; //���� �� �����
   	 
	  	if (++rx_counter1 == RX_BUFFER_SIZE1) //������������ ������
      	{
        	rx_counter1=0; //�������� ������� (������� ��� ������)
        	rx_buffer_overflow1=1;  //�������� � ������������
      	}	  
 		//--------------------------  
   		HAL_UART_Receive_IT(huart,RX_uBUF,1);
}


void UART_conrol (void)
{
 u16 i=0;
 u16 j=0;

  if (rx_counter1!=0u)
    {   
      if (rx_counter1<BUFFER_SR) j = rx_counter1; else j=BUFFER_SR;

      for (i=0u;i<j; i++) 
         {
           sr[i]=getchar1();
           lenght=i+1;  
           if (sr[i]==';') {break;}
          }
            sr[lenght]=0x00;
            IO (sr);
        };
}


void LED (void)
{	
static u64 p=0xffffffffff;
u8 a[3]={0x00,0x00,0x00};
u8 b[3]={0xff,0xff,0xff};

	if ((TIMER1<100)&&(FLAG_T1==0)) 
	{		
		VD3(1);
		VD4(0);
		VD5(0);
		if (p!=0) p=p>>1; else p=0xffffffffff;
		
		DD8_write  (p);
		DD9_write  (p>>8);
		DD25_write (p>>16);
		DD27_write (p>>24);
		DD37_write (p>>32);
		
		tca6424_to_port_write (31,a);
		
		LED_DD44 (1);
		LED_DD43 (0);
		LED_DD33 (1);
		LED_DD38 (0);
		
		LED_TH_R_MK(1);
		LED_TH_MK(0);
		
		FLAG_T1=1;
		TEST_LED=TEST_LED<<1;
	}
	
	if ((TIMER1>200)&&(FLAG_T2==0)) 
	{
		VD3(0);
		VD4(1);
		VD5(0);
		
        LED_DD44 (0);
		LED_DD43 (1);
		LED_DD33 (0);
		LED_DD38 (1);
		
		RAB_NORMA_MK(1);
		PIT_NORMA_MK(0);
		
		tca6424_to_port_write (31,b);
		
		FLAG_T2=1;
		//Transf2("~0 help;\r\n");
	}
	
	if ((TIMER1>400))
	{
		VD3(0);
		VD4(0);
		VD5(1);
		
		LED_DD44 (1);
		LED_DD43 (0);
		LED_DD33 (0);
		LED_DD38 (1);
		
		LED_TH_R_MK(0);
		LED_TH_MK(1);
		
	    RAB_NORMA_MK(0);
		PIT_NORMA_MK(1);
		
	}

	if (TIMER1>500)	
	{ 
		TIMER1=0;
		FLAG_T1=0;
		FLAG_T2=0;
	}	
}



volatile  u32 adc_cntl=0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)

{
	adc_cntl++;
//	HAL_ADC_Start_IT(hadc1);
}



double ADC_Temp(double t)
{
double TCelsius;	
TCelsius = ((t - 0.760) / 0.0025) + 25.0 ;
return TCelsius;
} 

void ADC_test (void)
{
//	u32 i=0;

	adc_cntl=0;	
	
	adc_ch[ 0] = ((float)adcBuffer[0])*3.3/4096*2;
	adc_ch[ 1] = ((float)adcBuffer[1])*3.3/4096*2;
	adc_ch[ 2] = ((float)adcBuffer[2])*3.3*5.71428/4096;//12 ����� �������� 47�-10�
	adc_ch[ 3] = ((float)adcBuffer[3])*3.3/4096*2;
	adc_ch[ 4] = ((float)adcBuffer[4])*3.3/4096;	
	
	Transf("\r\n---------\r\n");
	f_out("adc0_5.0V     :",adc_ch[0 ]);
	f_out("adc1_3.3V     :",adc_ch[1 ]);
	f_out("adc2_12V      :",adc_ch[2 ]);
	f_out("adc3_3.3V     :",adc_ch[3 ]);
	f_out("temp_sens(�)  :",ADC_Temp(adc_ch[4]));
	
	Transf("\r\r\r");
	u_out("adc0_5.0V     :",adcBuffer[0 ]);
	u_out("adc1_3.3V     :",adcBuffer[1 ]);
	u_out("adc2_12V      :",adcBuffer[2 ]);
	u_out("adc3_3.3V     :",adcBuffer[3 ]);
	u_out("temp_sens(�)  :",adcBuffer[4 ]);	
}

u8 FLAG_WDG=0;

void DMA_ADC (void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA  (&hadc1,(uint32_t*)&adcBuffer,5); // Start ADC in DMA 	
}

void WATCH_DOG (void)
{	
	if (FLAG_WDG==0) {FLAG_WDG=1;} else FLAG_WDG=0;	
	WDI_MK(FLAG_WDG);	
}

  
void PWM (u16 a)
{
  sConfigOC.Pulse      = a;
  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4,TIM_CHANNEL_1);
}  


u8 TCA_test (void)
{
 return 0;
}

u8 TCA_WR (u32 z)
{
	uint16_t DevAddress=0x22;//����� 
	 u8  a[4];
	 uint8_t state=0;
	 
	 HAL_I2C_Init(&hi2c1);
	
	a[0] =   0x8c;  //Configuration Port 0
	a[1] =   0x00;
	a[2] =   0x00;
	a[3] =   0x00;
	
	state=HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,4,1000);//������������� ����� �� ��� ������
	
	a[0] =   0x84;  //Output Port 0
	
	a[1] = (z>> 0)&0xff;
	a[2] = (z>> 8)&0xff;
	a[3] = (z>>16)&0xff;
	
	state=HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,4,1000);
	
	HAL_I2C_DeInit(&hi2c1);

	return state;
}


u8 DS4520_read (void)  //��������� ��������� ����� ����������� - 15 ���� ����������� DD10 (8����)  - ������ POWERGOOD �� �������������
{
	uint16_t DevAddress=0x53;//����� DD10 ds4520
	 u8  c[1]; 
	 u8  a[2];
	 uint8_t state=0;
	 uint8_t v=0;
	 u32 error=0;
	
	HAL_I2C_Init(&hi2c1);	

	c[0] =   0xf9;  //�������-������ I/O Status 1 
	
	state  = HAL_I2C_IsDeviceReady  (&hi2c1,(DevAddress<<1),1,  1000);
if (!state)  
	{
		HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,1,1);
	}
	
	HAL_I2C_DeInit(&hi2c1);	
	v=a[0]&0x01;	
	return v;
}

u8 DS4306_read (u8 adr_m,u8 adr_r)  //
{
	uint16_t DevAddress=adr_m;//����� DD10 ds4520
	 u8  c[1]; 
	 u8  a[2];
	 uint8_t state=0;
	 uint8_t v=0;
	 u32 error=0;
	
	HAL_I2C_Init(&hi2c1);	

	c[0] =adr_r;  //�������-������ I/O Status 1 

	HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<0),c,1,a,1,1);
	
	HAL_I2C_DeInit(&hi2c1);	
	v=a[0]&0x01;	
	return v;
}

u8 DS4306_write (u8 adr_m,u8 adr_r,u8 data)
{
	uint16_t DevAddress=adr_m;//����� 
	 u8  a[4];
	 uint8_t state=0;
	 
	 HAL_I2C_Init(&hi2c1);
	
	a[0] =   adr_r;  //Configuration Port 0
	a[1] =   data;
	
	state=HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<0),a,2,1000);//������������� ����� �� ��� ������
	
	HAL_I2C_DeInit(&hi2c1);

	return state;
}

void LED_DD44 (u8 a)
{
	u8 dat=(a&1)<<5;
	DS4306_write (0xaa,1,dat);
}

void LED_DD43 (u8 a)
{
	u8 dat=(a&1)<<5;
	DS4306_write (0xa8,1,dat);
}

void LED_DD38 (u8 a)
{
	u8 dat=(a&1)<<5;
	DS4306_write (0xae,1,dat);
}

void LED_DD33 (u8 a)
{
	u8 dat=(a&1)<<5;
	DS4306_write (0xb0,1,dat);
}

u8 tca9534_read (u8 adr_m,u8 adr_r)  //
{
	uint16_t DevAddress=adr_m;//����� DD10 ds4520
	 u8  c[1]; 
	 u8  a[2];
	 uint8_t state=0;
	 uint8_t v=0;
	 u32 error=0;
	
	HAL_I2C_Init(&hi2c1);	

	c[0] =adr_r;  //�������-������ I/O Status 1 

	HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,1,1);
	
	HAL_I2C_DeInit(&hi2c1);	
	v=a[0]&0x01;	
	return v;
}

u8 tca9534_write (u8 adr_m,u8 adr_r,u8 data)
{
	uint16_t DevAddress=adr_m;//����� 
	 u8  a[4];
	 uint8_t state=0;
	 
	 HAL_I2C_Init(&hi2c1);
	
	a[0] =   adr_r;  //Configuration Port 0
	a[1] =   data;
	
	state=HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,2,1000);//������������� ����� �� ��� ������
	
	HAL_I2C_DeInit(&hi2c1);

	return state;
}

void DD8_write (u8 a)
{
	tca9534_write (32,3,0); //���������� � ���������������� ������� - ��� ��� ����� ��� ������
	tca9534_write (32,1,a); //���������� ������ � �����
}

void DD9_write (u8 a)
{
	tca9534_write (33,3,0); //���������� � ���������������� ������� - ��� ��� ����� ��� ������
	tca9534_write (33,1,a); //���������� ������ � �����
}

void DD25_write (u8 a)
{
	tca9534_write (37,3,0); //���������� � ���������������� ������� - ��� ��� ����� ��� ������
	tca9534_write (37,1,a); //���������� ������ � �����
}

void DD27_write (u8 a)
{
	tca9534_write (36,3,0); //���������� � ���������������� ������� - ��� ��� ����� ��� ������
	tca9534_write (36,1,a); //���������� ������ � �����
}

void DD37_write (u8 a)
{
	tca9534_write (39,3,0); //���������� � ���������������� ������� - ��� ��� ����� ��� ������
	tca9534_write (39,1,a); //���������� ������ � �����
}

void tca6424a_read (u8 adr_m,u8 adr_r,u8 *dat) //������ � ������ �� ���� ����
{
uint16_t DevAddress=adr_m;//����� 
	 u8  c[1]; 
	 u8  a[3];
	 uint8_t state=0;
	 uint8_t v=0;
	 u32 error=0;
	
	HAL_I2C_Init(&hi2c1);	

	c[0] =adr_r;  //�������-������ I/O Status 1 

	HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,3,1);
	
	HAL_I2C_DeInit(&hi2c1);	
	dat[0]=a[0];
	dat[1]=a[1];
	dat[2]=a[2];
}

void tca6424a_from_port (u8 number,u8 *a)
{
   tca64_adr (number);//����� ��� ����������� ����������� �� ����� 31 ��������� �������� ������ �����
   tca6424a_read (34,0x80,a);
}

u8 tca6424a_write (u8 adr_m,u8 adr_r,u8 *dat)
{
	uint16_t DevAddress=adr_m;//����� 
	 u8  a[4];
	 uint8_t state=0;
	 
	 HAL_I2C_Init(&hi2c1);
	
	a[0] =   adr_r;  //Configuration Port 0
	a[1] =   dat[0];
	a[2] =   dat[1];
	a[3] =   dat[2];
	
	state=HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,4,1000);//������������� ����� �� ��� ������
	
	HAL_I2C_DeInit(&hi2c1);

	return state;
}

void tca6424_to_port_write (u8 number,u8 *a)
{
	u8 b[3]={0x00,0x00,0x00};
	tca64_adr (number);//����� ��� ����������� ����������� �� ����� 31 ��������� �������� ������ �����
	tca6424a_write (34,0x8c,b);//������������� ����� �� ������
	tca6424a_write (34,0x84,a);
}


void tca64_adr (u8 a)
{
	if (a==31) 
	{
		ADDR_DD36   (0);
		ADDR_DD37   (1);
		ADDR_FAULT  (1);
		ADDR_DISABLE(1);
	} else
	if (a==35) 
	{
		ADDR_DD36   (1);
		ADDR_DD37   (1);
		ADDR_FAULT  (1);
		ADDR_DISABLE(0);
	}else 
	if (a==32) 
	{
		ADDR_DD36   (1);
		ADDR_DD37   (1);
		ADDR_FAULT  (0);
		ADDR_DISABLE(1);
	}else 
	if (a==34) 
	{
		ADDR_DD36   (1);
		ADDR_DD37   (0);
		ADDR_FAULT  (1);
		ADDR_DISABLE(1);
	} 
		
}

void SYS_INFO (u8 a)
{
	
}

float okrug(float chislo, long znaki)
{
    float res;
    return round(chislo * pow(10, znaki)) / pow(10, znaki);
}

u32 idx_srv(
u32 a,//������� ������ ������� � ������� ���������
u32 k //�������� ������ � ������ �� �� ���������� ���������
) //������� ����������� ������ ������ � ������� "���������" 
{
 u32 idx=0;
 idx=a+24+k;//
 if (idx>(SIZE_SERVER-1)) idx=idx-SIZE_SERVER;
 return idx;
}


//������� �������� ...
void MSG_SEND_UDP (ID_SERVER *id,SERVER *srv,u32 msg_type)
{
  u32 i=0;
  u32 idx0=0;
  u32 idx1=0;
  u32 idx2=0;
  u32 idx3=0;
  u32 idx4=0;
  u32 idx5=0;
  u32 idx6=0;
  u32 idx7=0;
  
  u64 ADR=ADRES_SENDER_CMD;
  u32 data=0;
  u8 D[4];
        
 if (msg_type==MSG_REQ_TEST_485)
 {
    ARRAY_DATA(FLAG_ASQ_TEST_485);
 }else
 if (msg_type==MSG_REQ_TEST_SPI)
 {
    ARRAY_DATA(FLAG_ASQ_TEST_SPI);
 }else
 if (msg_type==MSG_REQ_TEST_JTAG)
 {
    ARRAY_DATA(FLAG_ASQ_TEST_JTAG);
 }
       

  SYS_CMD_MSG(
        id,//������
        &INVOICE[ADR],  //��������� ��������� 
        i,        //������ � �������
        msg_type, //��� ���������
        4,        //����� ������ ��������� � ������
        D_TEMP,   //������ ��������� - ������ ������
        TIME_SYS  //����� ����������� ���������
        );
}


#define COL 37
u8 DATA_TR [COL];

void ARR_Z ()
{u8 n=0;
  
  DATA_TR[n++]=B077.INIT;			//0
  DATA_TR[n++]=B077.TEMP_MAX>>8;	//1	
  DATA_TR[n++]=B077.TEMP_MAX&0xff;  //2	 
  DATA_TR[n++]=B077.FLAG_1HZ;		//19
  DATA_TR[n++]=B077.B077_NUMBER>>24;//20
  DATA_TR[n++]=B077.B077_NUMBER>>16;//21
  DATA_TR[n++]=B077.B077_NUMBER>> 8;//22
  DATA_TR[n++]=B077.B077_NUMBER;	//23
  DATA_TR[n++]=B077.PRG_VERSIYA>>56;//24
  DATA_TR[n++]=B077.PRG_VERSIYA>>48;//25
  DATA_TR[n++]=B077.PRG_VERSIYA>>40;//26
  DATA_TR[n++]=B077.PRG_VERSIYA>>32;//27
  DATA_TR[n++]=B077.PRG_VERSIYA>>24;//28
  DATA_TR[n++]=B077.PRG_VERSIYA>>16;//29
  DATA_TR[n++]=B077.PRG_VERSIYA>> 8;//30
  DATA_TR[n++]=B077.PRG_VERSIYA>> 0;//31
  DATA_TR[n++]=B077.WORK_TIME>>24;	//32
  DATA_TR[n++]=B077.WORK_TIME>>16;	//33
  DATA_TR[n++]=B077.WORK_TIME>> 8;	//34
  DATA_TR[n++]=B077.WORK_TIME>> 0;  //35
  DATA_TR[n++]=B077.STATUS_OK; 		//36

}

void SYS_INFO_SEND_UDP (ID_SERVER *id,SERVER *srv)
{
	u32 i=0;
	u32 idx0=0;
	u32 idx1=0;
	u32 idx2=0;
	u32 idx3=0;
	u32 idx4=0;
	u32 idx5=0;
	u32 idx6=0;
	u32 idx7=0;
	
	u64 ADR=ADRES_SENDER_CMD;
	u32 data=0;
	u8 D[4];
		
//----------------------------------
//��������� �� ����� ��������� ������� �330
	ARR_Z ();//��������� ������������ ������

			SYS_CMD_MSG(
			id,//������
			&INVOICE[ADR], 	//��������� ���������	
			i,	 			//������ � �������
			MSG_STATUS_OK,  //��� ���������
			COL,		    //����� ������ ��������� � ������
			DATA_TR,  		//������ ��������� - ������ ������
			TIME_SYS	  	//����� ����������� ���������
			);

}


void CMD_search (ID_SERVER *id,SERVER *srv)
{
	u32 i=0;
	u32 idx0=0;
	u32 idx1=0;
	u32 idx2=0;
	u32 idx3=0;
	u32 idx4=0;
	u32 idx5=0;
	u32 idx6=0;
	u32 idx7=0;
	
	u64 ADR=0;
	u32 data=0;
	u8 D[4];

	for (i=0;i<SIZE_ID;i++)
	{		
		if (id->CMD_TYPE[i]==CMD_TIME_SETUP) 
		{
			Transf("\r\n------\r\n");
			Transf("�������:��������� �������!\r\n");
			
			idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
			idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
			idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
			idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
			idx4=idx_srv(id->INDEX[i],4);//������ ������������ ������ � "���������"
			idx5=idx_srv(id->INDEX[i],5);//������ ������������ ������ � "���������"
			idx6=idx_srv(id->INDEX[i],6);//������ ������������ ������ � "���������"
			idx7=idx_srv(id->INDEX[i],7);//������ ������������ ������ � "���������"
			
			TIME_SYS=((u64)srv->MeM[idx0]<<56)|((u64)srv->MeM[idx1]<<48)|
					 ((u64)srv->MeM[idx2]<<40)|((u64)srv->MeM[idx3]<<32)|
					 ((u64)srv->MeM[idx4]<<24)|((u64)srv->MeM[idx5]<<16)|
					 ((u64)srv->MeM[idx6]<< 8)|((u64)srv->MeM[idx7]<< 0);
			IO("~0 time;");
			Transf("\r\n------\r\n");		
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//���� ���������� ����� ����������� � ��������� ������������, ���� ��� ��� ���  - �� ������� ����
			u_out("����� �����������:",ADR);

			ERROR_CMD_MSG ( //��������� ��������� � ���������� �������
			id,			    //��������� �� ������
			&INVOICE[ADR],  //��������� �� ��������� ���������
			i, 			    //������ ������� � �������
			MSG_CMD_OK,	    //��������� ���������
			0,				//������ ���������
			TIME_SYS	    //������� ��������� ����� 
			);	
			SERV_ID_DEL (id,i);//������� ������� �� �������
		} else
		
		if (id->CMD_TYPE[i]==CMD_STATUS)//������� ������� ��������
		{
			//��� ������ � �������
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//���� ���������� ����� ����������� � ��������� ������������, ���� ��� ��� ���  - �� ������� ����
			ADRES_SENDER_CMD=ADR;
//			FLAG_ADRES_SENDER_CMD=1;//��������� ���� ���� ��� � ��� ���� ���� ���������� ���������
			ERROR_CMD_MSG ( //��������� ��������� � ���������� �������
			id,			    //��������� �� ������
			&INVOICE[ADR],  //��������� �� ��������� ���������
			i, 			    //������ ������� � �������
			MSG_CMD_OK,	    //��������� ���������
			0,				//������ ���������
			TIME_SYS	    //������� ��������� ����� 
			);	
			
			//------------------------------------
			SYS_INFO_SEND_UDP (id,srv);
			//-----------------------------------
			SERV_ID_DEL (id,i);//������� ������� �� �������
			
//			Transf("������ ���������!\r\n");	
//			un_out("[",TIME_SYS);Transf("]\r\n");
		}	else
		
		
		if (id->CMD_TYPE[i]==CMD_12V)//������� ��������� ��������� +12V
		{
			idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
			idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
			idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
			idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			START_BP=data;//��������� �������
			
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//���� ���������� ����� ����������� � ��������� ������������, ���� ��� ��� ���  - �� ������� ����
			FLAG_ADRES_SENDER_CMD=1;//��������� ���� ���� ��� � ��� ���� ���� ���������� ���������

			ERROR_CMD_MSG ( //��������� ��������� � ���������� �������
			id,			    //��������� �� ������
			&INVOICE[ADR],  //��������� �� ��������� ���������
			i, 			    //������ ������� � �������
			MSG_CMD_OK,	    //��������� ���������
			0,				//������ ���������
			TIME_SYS	    //������� ��������� ����� 
			);	
			SERV_ID_DEL (id,i);//������� ������� �� �������
			
			Transf("���������� �������� +12V!\r\n");
			u_out("START_BP:",START_BP);
		} else		
     if (id->CMD_TYPE[i]==CMD_TEST_485)//������� ������ ����� �������� ���� 485
    {
      Transf("�������� ���� �������� ���� 485!\r\n");
      FLAG_ASQ_TEST_485=0;//���������� ���� ������ �� ���� 485
      FUNC_FLAG_UP (&POINTER_TEST_485,10);//������ ���������� ������ ��� ������ ������ �� ���������, ������ ������� 0 � ��������!!!
      ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//���� ���������� ����� ����������� � ��������� ������������, ���� ��� ��� ���  - �� ������� ����

      ERROR_CMD_MSG ( //��������� ��������� � ���������� �������
      id,             //��������� �� ������
      &INVOICE[ADR],  //��������� �� ��������� ���������
      i,              //������ ������� � �������
      MSG_CMD_OK,     //��������� ���������
      0,              //������ ���������
      TIME_SYS        //������� ��������� ����� 
      );  

      SERV_ID_DEL (id,i);//������� ������� �� �������
    }else
       if (id->CMD_TYPE[i]==CMD_TEST_SPI)//������� ������ ����� �������� ���� SPI
    {
      Transf("�������� ���� �������� ���� SPI!\r\n");
      FLAG_ASQ_TEST_SPI=0;//���������� ���� ������ �� ���� 485
      FUNC_FLAG_UP (&POINTER_TEST_SPI,10);//������ ���������� ������ ��� ������ ������ �� ���������, ������ ������� 0 � ��������!!!
      ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//���� ���������� ����� ����������� � ��������� ������������, ���� ��� ��� ���  - �� ������� ����

      ERROR_CMD_MSG ( //��������� ��������� � ���������� �������
      id,             //��������� �� ������
      &INVOICE[ADR],  //��������� �� ��������� ���������
      i,              //������ ������� � �������
      MSG_CMD_OK,     //��������� ���������
      0,              //������ ���������
      TIME_SYS        //������� ��������� ����� 
      );  

      SERV_ID_DEL (id,i);//������� ������� �� �������
    }else
       if (id->CMD_TYPE[i]==CMD_TEST_JTAG)//������� ������ ����� �������� ���� JTAG
    {
      Transf("�������� ���� �������� ���� JTAG!\r\n");
      FLAG_ASQ_TEST_JTAG=0;//���������� ���� ������ �� ���� 485
      FUNC_FLAG_UP (&POINTER_TEST_JTAG,10);//������ ���������� ������ ��� ������ ������ �� ���������, ������ ������� 0 � ��������!!!
      ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//���� ���������� ����� ����������� � ��������� ������������, ���� ��� ��� ���  - �� ������� ����

      ERROR_CMD_MSG ( //��������� ��������� � ���������� �������
      id,             //��������� �� ������
      &INVOICE[ADR],  //��������� �� ��������� ���������
      i,              //������ ������� � �������
      MSG_CMD_OK,     //��������� ���������
      0,              //������ ���������
      TIME_SYS        //������� ��������� ����� 
      );  

      SERV_ID_DEL (id,i);//������� ������� �� �������
    }
		
	//	if (id->TIME<TIME_SYS)
	}
}

void CONTROL_T1HZ_MK (void)
{
	u8 val=0;
	if ((TIMER_T1HZ_MK>1500)&&(FLAG_T1HZ_MK==1)) 
	{
		FLAG_T1HZ_MK=0;
		Transf("������ ������ T1HZ_MK!!!\r\n");
	}
	B077.FLAG_1HZ=FLAG_T1HZ_MK;
}
  
void LED_CONTROL (void)
{
       u32 z =0;
static u32 z0; 

}

void CONTROL_POK (void)
{

}

void CONTROL_SYS (void)
{
	static u8 flag=0;
	u32 tmp0=0; 
}

void ALARM_SYS_TEMP (void)  
{
	u8 var=0;
}

void UART_CNTR (UART_HandleTypeDef *huart)
{
	if (huart->gState != HAL_UART_STATE_BUSY_TX)
	{
		NRE_RS485(0);//�������� ����� 485 �� ���� 
		DE_RS485 (0);//��������� ����� 485 �� ��������
	}
}  

void REQ_VERSIYA (void)
{
  u32 tmp0,tmp1;
  tmp0=STM32_VERSION>>16;   //��� ����
  tmp1=STM32_VERSION&0xffff;//��� �����
  Transf("----------------------\r\n");
  Transf("������ �������� STM32:\r\n");
  xn_out("����:",tmp0);Transf("  ");x_out("�����:",tmp1); 
}

//��������� ���������� ������
void DISPATCHER (u32 timer)
{
	u32 tmp0=0;
	int i=0;

	  if (FLAG_DWN(&POINTER_TEST_485))
      {
		    FUNC_FLAG_UP (&POINTER_TEST_485_REQ,500);//������ ���������� ������ ��� �������� ������� ������ �� ���� 485
        IO("~0 time;");
        Transf("�������� ��� �� ���� 485!\r\n");
        tmp0=ADR_SLAVE[1];//
        BUS_485_TEST (tmp0);
        return;
      } else		
	    if (FLAG_DWN(&POINTER_TEST_485_REQ))
      {
        IO("~0 time;");
        Transf("��������� ��������� ����� ���� 485!\r\n");
        if (FLAG_ASQ_TEST_485==1) Transf("���� �������!\r\n");
		    else
        {
          if (SCH_TST1<3)
          {
            SCH_TST1++;
            FUNC_FLAG_UP (&POINTER_TEST_485,10);//������ �����
          } else Transf("���� �� �������!\r\n");
        }    

        MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_485);//������� ��������� ������� �� ����������� �����
        return;
      } else
       if (FLAG_DWN(&POINTER_TEST_JTAG))
      {
		FUNC_FLAG_UP (&POINTER_TEST_JTAG_REQ,500);//������ ���������� ������ ��� �������� ������� ������ �� ���� 485
        IO("~0 time;");
        Transf("���������� ���� JTAG!\r\n");
        FLAG_ASQ_TEST_JTAG=0;
        tmp0=jtag_scan(NULL,0);
        u_out("tmp0:",tmp0);
        return;
      }else		
	    if (FLAG_DWN(&POINTER_TEST_JTAG_REQ))
      {
        IO("~0 time;");
        Transf("��������� ��������� ����� ���� JTAG!\r\n");
        u_out("���������� ���������:",jtag_dev_count);
        for(i=0;i<jtag_dev_count;i++)
        {
          un_out("",i);Transf("] ");xn_out("",jtag_devs[i].idcode);Transf(" - ");Transf(jtag_devs[i].descr);Transf("\r\n");
        }        

        if (jtag_dev_count==2)
        {
        	if (jtag_devs[1].idcode==0x2a020dd)
        	{     Transf("���� �������!\r\n"   );FLAG_ASQ_TEST_JTAG=1;}
		    else  Transf("���� �� �������!\r\n");
        } 

        MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_JTAG);//������� ��������� ������� �� ����������� �����
        return;
      } 	 

  }

//��� ��������� ��������� ���������� �� ���� � ������������� ������� �� ������� �����
void FUNC_FLAG_UP (POINTER *p,u32 time)
{
  int i=0;
  p->FLAG=0;
  p->timer=time;
  while (PNT[i]!=0) {i++;};//���� ��������� ����� � ������� ����������
  PNT[i]=p;

  Transf("��������� ���� ����������� �������!\r\n");
  IO("~0 time;");
  u_out("timeout:",time);
}

//��� ��������� ������� ���������� �� ����
u8 FLAG_DWN (POINTER *p)
{
  if (p->FLAG==1)
  {
  	  p->FLAG=0;
  	  return 1;
  } else return 0;
}

void FTIME_OF_WORK ()
{
	if (TIME_OF_SECOND>600) 
	{
		TIME_OF_SECOND=0;
		TIME_OF_WORK++;
		FLAG_TIME_OF_WORK_WRITE=1;
	}
}

void REF_SFP_CONTROL (u8 a)
{
	if (SFP2_LOS    ==0) B077.SFP_REF2=0;
	if (SFP1_LOS    ==0) B077.SFP_REF1=0;
	if (i2c_PRESENT1==0) B077.I2cPRESENT1=0;
	if (i2c_PRESENT2==0) B077.I2cPRESENT2=0;
	if (i2c_PRESENT3==0) B077.I2cPRESENT3=0;
	if (i2c_PRESENT4==0) B077.I2cPRESENT4=0;
	if (i2c_PRESENT5==0) B077.I2cPRESENT5=0;
	if (i2c_PRESENT6==0) B077.I2cPRESENT6=0;
	
	if (a==1)
	{
		u_out("B077.SFP_REF2   :",B077.SFP_REF2);
		u_out("B077.SFP_REF1   :",B077.SFP_REF1);
		u_out("B077.i2cPRESENT1:",B077.I2cPRESENT1);
		u_out("B077.i2cPRESENT2:",B077.I2cPRESENT2);
		u_out("B077.i2cPRESENT3:",B077.I2cPRESENT3);
		u_out("B077.i2cPRESENT4:",B077.I2cPRESENT4);
		u_out("B077.i2cPRESENT5:",B077.I2cPRESENT5);
		u_out("B077.i2cPRESENT6:",B077.I2cPRESENT6);		
	}

}


int main(void)
{
	int i=0;
  /* USER CODE BEGIN 1 */
  Adress=0x30; //������ ������� 

  for (i=0;i<PNT_BUF;i++) PNT[i]=NULL;//�������� ����� ����������
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
  MX_I2C1_Init();
  MX_GPIO_Init();

  MX_DMA_Init ();
  MX_ADC1_Init();
//MX_TIM4_Init();  �� �����, � ����� ��� ������ ����!!!
//MX_SPI2_Init();
//MX_SPI3_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
//MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  
  Transf("-------------\r\n");
  Transf("    �077\r\n");
  Transf("-------------\r\n");
  DE_RS485(1);
  
  VD3(0);
  VD4(0);
  VD5(0);
  
 WDI_MK(0); 
 
HAL_UART_Receive_IT(&huart1,RX_uBUF,1);
HAL_UART_Receive_IT(&huart2,RX_uBUF,1);
HAL_ADC_Start_DMA  (&hadc1,(uint32_t*)&adcBuffer,5); // Start ADC in DMA 

//--------init wiz820------------------
 PWDN_4(0);//������� ��������� � wiz820
 NSS_4(1);
 RES_4(0);
 Delay(2);
 RES_4(1);
 INIT_SERV_ARCHIV (&SERV1,&ID_SERV1,&ADDR_SNDR);//�������������� ��������� � ������
 
 while (i<200)
 {
	 i++;
	 Delay(1);
	 WATCH_DOG ();
 }
 
 Set_network();
 RECEIVE_udp(0, 3001,1);
 
 
 for (i=0;i<8;i++) ADR_SLAVE[i]=0xff;//������� ������ � �������� ������� (��� ������ ������ ��� ������ �� 485 ����)

  B077.PRG_VERSIYA=STM32_VERSION;
  B077.WORK_TIME=TIME_OF_WORK; //����� ��������� ����� � �������� �����;


   ADDR_DD36(0); 
   ADDR_DD37(0); 
     EN_SINH(0); 
       RES_4(0); 
       RES_1(0); 
      WDI_MK(0); 
       NSS_4(0); 
      PWDN_4(0); 
      PWDN_1(0); 
         VD3(0); 
         VD4(0); 
         VD5(0); 
     CS_EPCS(0); 
   RESET_TCA(0); 
    DE_RS485(0); 
   NRE_RS485(0); 
 IN_SEL_DD29(0); 
  ADDR_FAULT(0); 
ADDR_DISABLE(0); 
   LED_TH_MK(0); 
 LED_TH_R_MK(0); 
RAB_NORMA_MK(0); 
PIT_NORMA_MK(0); 


  while (1)
  {
    /* USER CODE END WHILE */

	WATCH_DOG ();
	LED();
	UART_conrol();
	
	if (EVENT_INT1==1)
	{
		EVENT_INT1=0;
	//	Transf("event 1!\r");
		RECEIVE_udp (0, 3001,1);		
	}; 
	
	if (EVENT_INT3==1)//�������� ��������� ����� T1HZ_MK
	{
		//Transf("����   ������ T1HZ_MK!\r");
		if (FLAG_T1HZ_MK==0) Transf("����   ������ T1HZ_MK!\r");
		if (FLAG_ADRES_SENDER_CMD==1) SYS_INFO_SEND_UDP(&ID_SERV1,&SERV1);//�������� ��������� � ����� ���������
		EVENT_INT3=0;
		FLAG_T1HZ_MK=1;
		TIMER_T1HZ_MK=0;
		TIME_OF_SECOND++;//������������ ����� ������ � ������� ���������			
	}; 

    DISPATCHER        (TIMER_TIMEOUT);//��������� ���������� ������
	ALARM_SYS_TEMP    ();//���������� ���������� ����������� � ��������� ���������  
    CONTROL_SYS       ();//��������� ��������� �������: ����������� , ��� ����������� � �.�.
	CONTROL_POK 	  ();
	REF_SFP_CONTROL   (0);
	LED_CONTROL 	  ();
	CONTROL_T1HZ_MK   ();
	CMD_search        (&ID_SERV1,&SERV1);
	SEND_UDP_MSG 	  ();
    UART_DMA_TX  	  ();
	UART_DMA_TX2  	  ();
	UART_CNTR         (&huart2);//��� ��������� ��������� 485
	FTIME_OF_WORK     ();       //��� ������ �� �������� ���������

  }

}
