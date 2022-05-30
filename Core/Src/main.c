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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
//#include "FONT_X.h"
#include "F1.h"
#include "F2.h"
#include "F3.h"
//#include "monocond12.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//==================================== WEO128128A ======================================
#define SET_DISPLAY_COL_ADD 0x15
#define SET_DISPLAY_ROW_ADD 0x75
#define SET_DISPLAY_CONTRAST 0x81
#define SET_DISPLAY_RE_MAP 0xA0
#define SET_DISPLAY_START_LINE 0xA1
#define SET_DISPLAY_OFFSET 0xA2
#define SET_DISPLAY_MODE 0xA4
#define SET_DISPLAY_MUX 0xA8
#define SET_DISPLAY_REG 0xAB
#define SET_DISPLAY_OFF 0xAE
#define SET_DISPLAY_ON 0xAF
#define SET_DISPLAY_PHASE 0xB1
#define SET_DISPLAY_CLOCK 0xB3
#define SET_DISPLAY_GPIO 0xB5
#define SET_DISPLAY_PERIOD 0xB6
#define SET_DISPLAY_R_SCROLL 0x26
#define SET_DISPLAY_L_SCROLL 0x27
#define SET_DISPLAY_DEACT_SCROLL 0x2E
#define SET_DISPLAY_ACT_SCROLL 0x2F
#define BACKGROUND_COLOR 0x00
#define MAX_CONTRAST 0xFF
#define OLED_DIM_WIDTH  0x7F
#define OLED_DIM_HEIGHT 0x7F
#define		MAX_COL		64
#define		MAX_ROW		128
//=====================================	LIS3DH ===========================================
#define accelDataRate_PWRDWN 0x00
#define accelDataRate_1_Hz 0x10
#define accelDataRate_10_Hz 0x20
#define accelDataRate_25_Hz 0x30
#define accelDataRate_50_Hz 0x40
#define accelDataRate_100_Hz 0x50
#define accelDataRate_200_Hz 0x60
#define accelDataRate_400_Hz 0x70
#define accelDataRate_1600_Hz 0x80
#define accelDataRate_5000_Hz 0x90
#define FIFO_BYPASS 0b00000000
#define FIFO_FIFO_MODE 0b01000000
#define FIFO_STREAM_MODE 0b10000000
#define FIFO_REIGGER_MODE 0b11000000
#define FIFO_TRIGGER_INT1 0b00000000
#define FIFO_TRIGGER_INT2 0x00100000
#define Xen 0b00000001
#define Yen 0b00000010
#define Zen 0b00000100
#define BDU 0b10000000
#define BLE 0b01000000
#define FS1 0b00100000
#define FS0 0b00010000
#define FULL_SCALE_2G 0x00
#define FULL_SCALE_4G 0x10
#define FULL_SCALE_8G 0x20
#define FULL_SCALE_16G 0x30
#define HR 0b00000100
#define ST1 0b00000100
#define ST0 0b00000010
#define SIM 0b00000001
#define FIFO_EN 0b01000000
#define OUT1L 0x08
#define OUT1H 0x09
#define OUT2L 0x0A
#define OUT2H 0x0B
#define OUT3L 0x0C
#define OUT3H 0x0D
#define INT_COUNTER 0x0E
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define CTRL_REG6 0x25
#define REF_DATACAPTURE 0x26
#define STATUS_REG 0x27
#define OUTXL 0x28
#define OUTXH 0x29
#define OUTYL 0x2A
#define OUTYH 0x2B
#define OUTZL 0x2C
#define OUTZH 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT_1_CFG 0x30
#define INT_1_SRC 0x30

//#define X_increment 8
//#define startAddressForImageInfo 0x00000000
#define startAddressForSoundInfo 0x400000
//#define startAddressForSoundInfo 0				//4 test ONLY!!!!!!!!!!!!!!!!!!!!!!!

#define USART_ISR_RXNE                      ((uint32_t)0x00000020U)
#define USART_ISR_TXE                      ((uint32_t)0x00000080U)

#define USART_CR2_MSBFIRST   (1 << 19)
#define imReg USART3->RDR


#define bufLen (uint32_t)1024
#define PI 3.14159265358979323846
#define TAU (2.0 * PI)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 COMP_HandleTypeDef hcomp1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_tx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

USART_HandleTypeDef husart3;

/* USER CODE BEGIN PV */
uint8_t isReceiverDisabled;
uint8_t dataReceived = 0; // признак данное получено
uint8_t dataTransmitted = 1; // признак данное передано
uint8_t PCB_type[17] = { 0b00000010, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
		0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }; //cmd 00h (contains 17 bytes)
uint8_t PCB_rev[0x08] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77 }; // cmd 01h (contains 8 bytes)
uint8_t currentConsumption = 0x80; //cmd 02h (contains 1 byte in mA)
uint8_t EmitterSN[16] = { 0x1, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
		0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }; //cmd 03h  (contains 16 bytes)
uint8_t picNum, soundNum, volume, contrast, currentUARTspeed,strLen;
		ASCII_Y,imX,imY, dat2screen, CommandID, uartSpeed;
uint8_t  X_increment=0x07;
uint8_t  Y_increment=0x0E;
uint8_t  ASCII_height=0x0E;
uint8_t dataASCII[18], I2Cbuf[5];
uint16_t uartData[10];
uint8_t xVal, yVal, zVal = 0x00;
uint16_t xFull, yFull, zFull=0;
uint32_t MEM_ID = 0;
uint16_t PAGE_SIZE = 4096;
extern uint32_t addr;
extern uint32_t DATA_COUNT = 0x2000;
extern uint8_t MEM_Buffer[8192];
extern uint8_t imInfo[6];
extern uint8_t keyboard, cmdLen;
extern uint8_t currentVolume = 0x09;
uint8_t cmd[25];
uint8_t accelBuff[6];
uint16_t ans[10];
uint8_t bf=0x7F;
uint8_t tmp_str[10];
uint8_t ByteReceived=0;
uint8_t firstByteReceived=0;
uint16_t dt1;
uint16_t ind = 0;
extern uint16_t currentPic;
uint32_t tmp;
uint32_t startAddressForImageInfo=0x00000000;
uint8_t uartFlag=0;
uint8_t cmd2Execute,strLen;
uint8_t my_test1[], Image[], coala[],h1[],ASCIIcode[],h2[],image_data_Font_0x30[],image_data_Font_0x31[],
image_data_Font_0x32[],image_data_Font_0x33[],image_data_Font_0x34[],image_data_Font_0x35[],image_data_Font_0x36[],
image_data_Font_0x37[],image_data_Font_0x38[],image_data_Font_0x39[];
uint8_t aim[],frame[],test[];
uint8_t bf4me;
uint8_t dimmer=0;
uint8_t inputCS=0;
uint8_t dma_spi_fl=0;
uint16_t dma_spi_cnt=1;
uint32_t len;
uint8_t MEM_Buffer[8192];
uint32_t address;
uint32_t curAddr;
uint8_t decY;
uint8_t fontInfo;
uint8_t color;

// uint16_t signal[2048];
uint16_t nsamples = 2048;
uint32_t curBuf;
uint32_t bufCount;
 uint16_t SOUND1[bufLen];
 uint16_t SOUND2[bufLen];

// uint8_t soundReady=1;
	uint8_t bufAccel[] = { 0x33, 0x0F };
	uint32_t soundLen;
	uint32_t addrSound;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_COMP1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_Init(void);
static void MX_I2S1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t weoShowFullScreen(uint8_t picNum);
uint8_t weoShowFullScreenDMA(uint8_t picNum);
uint8_t soundLoad(uint8_t soundNum, uint8_t soundReady, uint8_t isSoundOver);
uint8_t soundPlay(uint8_t soundNum);
uint8_t weoShowSmallImage(uint8_t picNum, uint8_t imX, uint8_t imY);
uint8_t weoShowSmallImageDMA(uint8_t picNum, uint8_t imX, uint8_t imY);
uint8_t* weoShowFullScreenSoundnfo(uint32_t addr);
uint8_t answer2CPU(uint8_t cmd[]);
uint16_t Scount(void);
//uint8_t cmd2Execute;
uint8_t cmdExecute(uint8_t cmd2Execute);
uint8_t printASCIIarray(uint8_t imX,uint8_t imY,uint8_t strLen,uint8_t fontInfo,uint8_t dataASCII[]);
uint32_t MEM_GetID(void);
void squeak_single(uint16_t* signal);
void squeak_long(uint16_t* signal);
void squeak_double(uint16_t* signal);
void squeak_triple(uint16_t* signal);
void squeak_generate(void);
void MEM_Write(uint32_t addr);
void LIS3DHsetup(void);
uint8_t LIS3DHreadReg(uint8_t reg);
void LIS3DHreadData(void);
uint16_t Accel_ReadAcc(void);
void Demo(void);
void weoDrawRectangleFilled(unsigned char start_x, unsigned char start_y,
		unsigned char end_x, unsigned char end_y, unsigned char contrast,
		uint8_t *MEM_Buffer);
void weoDrawRectangleInit(unsigned char start_x, unsigned char start_y,
		unsigned char end_x, unsigned char end_y);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void GPIO_2_3state(void);

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
	uint16_t testINPUT = 0;
	uint16_t testAns = 0;
	uint16_t i = 0;
	uint16_t beep1 = 0x0000;
	uint16_t beep2 = 0xFFFF;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_COMP1_Init();
  MX_TIM1_Init();
  MX_USART3_Init();
  MX_I2S1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	HAL_COMP_Start(&hcomp1);
	HAL_Delay(100);
	uint8_t I2Cbuf[5];
	MEM_Reset();

	weoInit();
	HAL_Delay(1);
	weoClear();
	MEM_GetID();
	soundSetup();
	LIS3DHsetup();

	USART2->CR1 |= (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE|USART_CR1_M); // Enable USART, Receive and Transmit

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LL_USART_EnableIT_RXNE(USART2);
	LL_USART_EnableIT_ERROR(USART2);

	USART2->ICR|=USART_ICR_ORECF;

//	squeak_generate();
//
//	I2C_SOUND_ChangePage(0x01);
//	WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
//	WriteReg_I2C_SOUND(0x2E, 0x24);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000) FF- speaker muted, 0x00 - 0x74 - available
//    squeak_triple(signal);
//    soundLoad(3);
//    soundPlay(3);
//    squeak_long();

	uint8_t ASCII_X=0x02;
	uint8_t imY=0x04;
	uint8_t ASCII_height=0x26;
	uint8_t X_increment=0x10;
	uint8_t decY=1;
	uint16_t k,j;
	uint8_t fontInfo=0xF1;
	uint8_t fontCur;
	uint8_t curStr=1;
//	uint8_t curStr[4]={1,2,3,4};
	uint8_t strLen=4;
	uint16_t symLen;
//	GPIOA->ODR &= ~(1 << 6);	//reset cs
//					GPIOA->ODR &= ~(1 << 7);	// reset dc
//					USART_AS_SPI_sendCMD(0x81);	//Contrast Level
//					USART_AS_SPI_sendCMD(0xFF);
//					GPIOA->ODR |= 1 << 7;	//set dc
//					GPIOA->ODR |= 1 << 6;	//set cs

	uint8_t localWidth=0x07;
		uint8_t localHeight=0x0E;
		uint8_t x = 0;
		uint8_t y = 0;
				decY=0x01;
				if(y % 2 !=0){
					decY=0x02;
				}
	weoDrawRectangleFilled(x,y,(x+localWidth-1),y+(localHeight-decY),0xFF,aim);
//	if((fontInfo & 0x00)==0){
//		fontCur=0;
//	}
//	if((fontInfo & 0x01)==1){
//		fontCur=1;
//	}
//	if(fontCur==0){
//		symLen=304;
//		uint8_t weoBuffer[symLen];
//		X_increment=0x10;
//		ASCII_height=0x26;
//		for(k=0;k<strLen;k++){
//			for(j=0;j<symLen;j++){
//			weoBuffer[j]=F3[curStr[k]][j];
//			}
//		weoDrawRectangleFilled(ASCII_X, imY, ASCII_X+X_increment-1, imY + ASCII_height - decY, 0xFF, weoBuffer);
//		ASCII_X += X_increment+0;
//		}
//	}
//	if(fontCur==1){
//		symLen=99;
//		uint8_t weoBuffer[symLen];
//		X_increment=0x07;
//		ASCII_height=0x12;
//		for(i=0;i<strLen;i++){
//			for(j=0;j<symLen;j++){
//				weoBuffer[j]=F2[curStr[i]][j];
//			}
//		weoDrawRectangleFilled(ASCII_X, imY, ASCII_X+X_increment-1, imY + ASCII_height - decY, 0xFF, weoBuffer);
//		ASCII_X += X_increment+0;
//		}
//	}


//	uint8_t shiftX=0x02;
//	uint8_t shiftY=0x02;
//	x+=shiftX;
//	y+=shiftY;
//				localWidth=0x07;
//				localHeight=0x0E;
//				decY=0x01;
//				if(y % 2 !=0){
//					decY=0x02;
//				}
//	weoDrawRectangleFilled(x,y,(x+localWidth-1),(y+localHeight-decY),0xFF,aim);
//    GPIOB->PUPDR &= ~0x3F000;
	GPIOC->ODR |= 1 << 6;
	soundPlay(0);
	while (1) {
//		weoShowSmallImage(0x02,0x70,0x00);
//		LIS3DHreadData();
		cmdExecute(cmd2Execute);
//		WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
//		WriteReg_I2C_SOUND(0x2E, 0x24);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000) FF- speaker muted, 0x00 - 0x74 - available
//		squeak_single(signal);
//		soundPlay(0);
//		HAL_Delay(21000);
//		USART2->ICR|=USART_ICR_ORECF;
//		USART2->ICR|=USART_ICR_FECF;
//		USART2->ICR|=USART_ICR_NECF;
//		xVal=0;
//		HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x29,I2C_MEMADD_SIZE_8BIT,&xVal, 1, HAL_MAX_DELAY);	//33h - address for reading
//		xFull=xVal;
//		xFull<<=8;
//		xVal=0;
//		HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x28,I2C_MEMADD_SIZE_8BIT,&xVal, 1, HAL_MAX_DELAY);	//33h - address for reading
//		xFull|=xVal;
//				yVal=0;
//				HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2B,I2C_MEMADD_SIZE_8BIT,&yVal, 1, HAL_MAX_DELAY);	//33h - address for reading
//				yFull=yVal;
//				yFull<<=8;
//				yVal=0;
//				HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2A,I2C_MEMADD_SIZE_8BIT,&yVal, 1, HAL_MAX_DELAY);	//33h - address for reading
//				yFull|=yVal;
//						zVal=0;
//						HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2D,I2C_MEMADD_SIZE_8BIT,&zVal, 1, HAL_MAX_DELAY);	//33h - address for reading
//						zFull=zVal;
//						zFull<<=8;
//						zVal=0;
//						HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2C,I2C_MEMADD_SIZE_8BIT,&zVal, 1, HAL_MAX_DELAY);	//33h - address for reading
//						zFull|=zVal;
//		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) 0x33, bufAccel, 2, 1000);	//(uint8_t*)&
//		HAL_Delay(1);
//		Scount();
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_VREFINT;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.WindowOutput = COMP_WINDOWOUTPUT_EACH_COMP;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_TIM1_OC5;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_EVENT_RISING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

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
  hi2c1.Init.Timing = 0x1094102C;
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
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 639;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 10;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 57600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
//	USART3->CR2 &= ~(USART_CR1_UE);
	USART3->CR2|=USART_CR2_MSBFIRST;
//	USART3->CR2|=USART_CR1_UE;
//	husart3.Init.BaudRate = 8000000;
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  husart3.Instance = USART3;
  husart3.Init.BaudRate = 4000000;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_TX;
  husart3.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart3.Init.CLKPhase = USART_PHASE_2EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_ENABLE;
  husart3.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart3.SlaveMode = USART_SLAVEMODE_DISABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  husart3.Init.BaudRate = 8000000;
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_SetOutputPin(MEM_HOLD_GPIO_Port, MEM_HOLD_Pin);

  /**/
  LL_GPIO_SetOutputPin(MEM_WP_GPIO_Port, MEM_WP_Pin);

  /**/
  LL_GPIO_SetOutputPin(MEM_CS_GPIO_Port, MEM_CS_Pin);

  /**/
  LL_GPIO_SetOutputPin(DISP_CS_GPIO_Port, DISP_CS_Pin);

  /**/
  LL_GPIO_SetOutputPin(DISP_D_C_GPIO_Port, DISP_D_C_Pin);

  /**/
  LL_GPIO_SetOutputPin(BF_GPIO_Port, BF_Pin);

  /**/
  LL_GPIO_ResetOutputPin(TEST_1_GPIO_Port, TEST_1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(TEST_2_GPIO_Port, TEST_2_Pin);

  /**/
  GPIO_InitStruct.Pin = MEM_HOLD_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MEM_HOLD_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MEM_WP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MEM_WP_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MEM_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MEM_CS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = KEY_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(KEY_3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = KEY_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(KEY_2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = KEY_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(KEY_1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DISP_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DISP_CS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DISP_D_C_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DISP_D_C_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BF_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BF_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TEST_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(TEST_1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TEST_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(TEST_2_GPIO_Port, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_EnableFastModePlus(LL_SYSCFG_I2C_FASTMODEPLUS_PB9);

}

/* USER CODE BEGIN 4 */
//GPIO config after CUBE_MX
///**/
//	  GPIO_InitStruct.Pin = KEY_4_Pin;
//	  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//	  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
//	  LL_GPIO_Init(KEY_5_GPIO_Port, &GPIO_InitStruct);
//	  /**/
//	    GPIO_InitStruct.Pin = KEY_5_Pin;
//	    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//	    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
//	    LL_GPIO_Init(KEY_5_GPIO_Port, &GPIO_InitStruct);
void  USART2_RX_Callback(void)
{
  dt1 = LL_USART_ReceiveData9(USART2);// LL implementaion of 1 byte receive
//	dt1 = (uint16_t)(USART2->RDR & 0x01FF);//CMSIS implementaion of 1 byte receive
  ByteReceived=1;
  if(dt1 & 0x100){
	  cmd[0]=dt1;
	  ind = 0;
	  firstByteReceived=1;
//  while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
//  LL_USART_TransmitData9(USART2,(uint16_t*)dt1);
  }
//  cmdReceive();
  if(firstByteReceived==1){
	  cmdReceive(dt1);
  }
}
//====================================================================================================================
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi2)
{
//	HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)SOUND1, bufLen);
//  	GPIOA->ODR |= 1 << 11;	//set test 1
//  	GPIOA->ODR &= ~(1 << 11);	//reset test 1
//  	decY=0x01;
//  			if(imY % 2 !=0){
//  				decY=0x02;
//  			}
//  	if(cmd2Execute==0x11){
//  	GPIOA->ODR &= ~(1 << 6);	//reset cs of DISPLAY
//  		GPIOA->ODR |= 1 << 7;	//set   dc of DISPLAY
//  		HAL_USART_Transmit_DMA(&husart3, MEM_Buffer,len);
//  	}
//  	if(cmd2Execute==0x14){
//	I2C_SOUND_ChangePage(0x01);
//			WriteReg_I2C_SOUND(0x01, 0x00);
//			I2C_SOUND_ChangePage(0x00);
//			WriteReg_I2C_SOUND(0x41, 0x30);// 0x81 - 0x30 available
//		//	I2C_SOUND_ChangePage(0x00);
//			I2C_SOUND_ChangePage(0x01);
//			WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
//			WriteReg_I2C_SOUND(0x2E, 0x24);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000) FF- speaker muted, 0x00 - 0x74 - available
//  	  		HAL_I2S_Transmit_DMA(&hi2s1, MEM_Buffer,len*2);
//  	  	}

}
//==========================================================
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi2)
{
//	if(cmd2Execute==0x11){
//		GPIOB->ODR |= 1 << 9; // set cs
//		HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)SOUND1, bufLen);
//	}
//	if(cmd2Execute==0x14){
//return;
//	}
}
//==============================================================
void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart3)
{
	GPIOA->ODR &= ~(1 << 7);	//reset dc
	GPIOA->ODR |= 1 << 6;	//set cs
	cmd2Execute=0;
//	while(BFEN==0){};
	GPIOC->ODR |= 1 << 6;	//set BF
}
//==============================================================
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi1)
{
//	GPIOA->ODR &= ~(1 << 7);	//reset dc	????????????????????????????????????????????????
//	GPIOA->ODR |= 1 << 6;	//set cs		????????????????????????????????????????????????
	//	cmd2Execute=0;
//	GPIOC->ODR |= 1 << 6;	//set BF
}
//======================================================================================================================
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s1) {
	curBuf += 1;
//	soundReady = 0;
//	return;
//	if(curBuf < 512){
	soundLoad(soundNum, 0, 0);
//}
}
//======================================================================================================================
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s1) {

	soundLoad(soundNum, 1, 0);
}
//=======================================================================================================================
	void cmdReceive (uint16_t dt1)
	{
//	  uint8_t inputCS=0;
	  uint8_t i=0;
	  while (!ByteReceived) {}
	  ByteReceived=0;
	  cmd[ind] = dt1;
//	  if(dt1==0x110){
//		  BFEN=0;
//	  }
	  ind++;
	  if(ind>=1){
//		  LL_USART_TransmitData9(USART2,(uint16_t*)dt1);
		  if(ind>cmd[1]+1){
//			  LL_USART_TransmitData9(USART2,(uint16_t*)cmd[2]);
			 for(i=0;i<(cmd[1]+2);i++){
				 inputCS+=cmd[i];
			 }
			 if(!(inputCS==0x00)&&(ind==cmd[1]+2)){
//			 	LL_USART_TransmitData9(USART2,(uint16_t*)cmd[2]);
//				 firstByteReceived=0;
				 firstByteReceived=0;
				 				 for (i=0;i<cmd[1]+2;i++){
				 					 cmd[i]=0;
				 				 }
			 }
			 else{
				 answer2CPU(cmd);
			 }
//			 if(inputCS!=0){
//				 GPIOC->ODR |= 1 << 6;	//set BF
//			 }
		  }
	  }
//	  ind = 0;
	  USART2->ICR|=USART_ICR_ORECF;
	}
	void USART_AS_SPI_sendCMD(uint8_t byte) {
			HAL_USART_Transmit(&husart3, (uint8_t*)&byte, 1, 10);
		}
	void USART_AS_SPI_sendDAT(uint8_t byte) //(Without CS and D/C)
	{
		HAL_USART_Transmit(&husart3, (uint8_t*) &byte, 1, 10);
	}
	void weoInit(void) {

		USART3->CR1 &= ~(USART_CR1_UE);
		USART3->CR2 |= USART_CR2_MSBFIRST;
		USART3->CR1 |= USART_CR1_UE;

		HAL_Delay(1);
		HAL_Delay(1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_Delay(1);

		GPIOA->ODR &= ~(1 << 6);	//reset cs
		GPIOA->ODR &= ~(1 << 7);	// reset dc
		USART_AS_SPI_sendCMD(0xAF);
		USART_AS_SPI_sendCMD(0xB8);
		USART_AS_SPI_sendCMD(0);
		USART_AS_SPI_sendCMD(0);
		USART_AS_SPI_sendCMD(1);
		USART_AS_SPI_sendCMD(2);
		USART_AS_SPI_sendCMD(4);
		USART_AS_SPI_sendCMD(7);
		USART_AS_SPI_sendCMD(11);
		USART_AS_SPI_sendCMD(14);
		USART_AS_SPI_sendCMD(18);
		USART_AS_SPI_sendCMD(23);
		USART_AS_SPI_sendCMD(27);
		USART_AS_SPI_sendCMD(33);
		USART_AS_SPI_sendCMD(40);
		USART_AS_SPI_sendCMD(49);
		USART_AS_SPI_sendCMD(63);
		USART_AS_SPI_sendCMD(0xA0);	//Set Re-map
//		USART_AS_SPI_sendCMD(0x54);
//		USART_AS_SPI_sendCMD(0x51);
		USART_AS_SPI_sendCMD(0b01010010);// 0b01010010 = 0x52 is a proper remap 4 my bmp_2_bin converter, but pictures must b turned right @ 90 degrees. 0b01000001 = 0x is good if turn display on 180 degrees.
//		USART_AS_SPI_sendCMD(0x41); //	0x51 is a proper remap 4 lcd image converter // 0b01010010 is a proper remap 4 left-turned images
		USART_AS_SPI_sendCMD(0x81);	//Contrast Level
		USART_AS_SPI_sendCMD(0xFF);
		USART_AS_SPI_sendCMD(0xA1);	//Set Display Start Line
		USART_AS_SPI_sendCMD(0x00);
		USART_AS_SPI_sendCMD(0xA2);	//Set Display Offset
		USART_AS_SPI_sendCMD(0x00);
		USART_AS_SPI_sendCMD(0xA8);  // Select Multiplex Ratio
		USART_AS_SPI_sendCMD(0x7F); // Default => 0x3F (1/64 Duty)	0x1F(1/32 Duty)
		GPIOA->ODR |= 1 << 7;	//set dc
		GPIOA->ODR |= 1 << 6;	//set cs
		//=======================================================================================================
	}
	void weoClear(void) {
		uint16_t i;

					GPIOA->ODR &= ~(1 << 6);	//reset cs
					GPIOA->ODR &= ~(1 << 7);	// reset dc
						USART_AS_SPI_sendCMD(SET_DISPLAY_ROW_ADD);
						USART_AS_SPI_sendCMD(0x00);
						USART_AS_SPI_sendCMD(0x7F);
						USART_AS_SPI_sendCMD(SET_DISPLAY_COL_ADD);
						USART_AS_SPI_sendCMD(0x00);
						USART_AS_SPI_sendCMD(0x7F);
					GPIOA->ODR &= ~(1 << 6);	//reset cs
					GPIOA->ODR |= 1 << 7;	// set dc
					for (i = 0; i <= 8193;i++) {	//fullScreen + small reserve
						while(!(USART3->ISR & USART_ISR_TXE)){};
						USART3->TDR = (uint8_t) 0x00;
					}
					GPIOA->ODR &= ~(1 << 7);	//reset dc
					GPIOA->ODR |= 1 << 6;	//set cs
	}
//========================================================================================================================
	void weoDrawRectangleFilled(unsigned char start_x, unsigned char start_y,
				unsigned char end_x, unsigned char end_y, unsigned char contrast,
				uint8_t MEM_Buffer[]) {
			uint16_t i = 0;
			uint8_t start_x_New,start_y_New,end_x_New,end_y_New;
			if (start_x > OLED_DIM_WIDTH || start_y > OLED_DIM_HEIGHT
					|| end_x > OLED_DIM_WIDTH || end_y > OLED_DIM_HEIGHT) {
				return;
			}

			start_x_New=start_x;
			start_y_New=start_y;
			end_x_New=end_x;
			end_y_New=end_y;

			GPIOA->ODR &= ~(1 << 6);	//reset cs
			GPIOA->ODR &= ~(1 << 7);	// reset dc
					USART_AS_SPI_sendCMD(SET_DISPLAY_ROW_ADD);
					USART_AS_SPI_sendCMD(start_x_New/1);
					USART_AS_SPI_sendCMD(end_x_New/1);
					USART_AS_SPI_sendCMD(SET_DISPLAY_COL_ADD);
					USART_AS_SPI_sendCMD(start_y_New/2);
					USART_AS_SPI_sendCMD(end_y_New/2);
//					USART_AS_SPI_sendCMD(0x81);	//Contrast Level
//					USART_AS_SPI_sendCMD(contrast);
			GPIOA->ODR |= 1 << 7;	//set dc
			GPIOA->ODR |= 1 << 6;	//set cs
			GPIOA->ODR &= ~(1 << 6);	//reset cs
			GPIOA->ODR |= 1 << 7;	// set dc

			for (i = 0; i < ((end_x_New - start_x_New + 1) * (end_y_New/2 - start_y_New /2 + 1));i++) {
//			for (i = 0; i < len;i++) {
//				if (i%((end_y-start_y))==0){
//					MEM_Buffer[i] &=0xF0 ;
//				}
				while(!(USART3->ISR & USART_ISR_TXE)){};
				USART3->TDR =MEM_Buffer[i];
			}
			while(!(USART3->ISR & USART_ISR_TXE)){};
			HAL_Delay(1);
//			GPIOA->ODR &= ~(1 << 7);	// reset dc
//			USART_AS_SPI_sendCMD(0xBB);	// command for NOP
//			USART_AS_SPI_sendCMD(0x81);	//Contrast Level
//			USART_AS_SPI_sendCMD(0xFF);

			GPIOA->ODR &= ~(1 << 7);	//reset dc
			GPIOA->ODR |= 1 << 6;	//set cs
		}
//========================================================================================================================
	void weoDrawRectangleInit(unsigned char start_x, unsigned char start_y,
				unsigned char end_x, unsigned char end_y) {
			uint16_t i = 0;
			uint8_t start_x_New,start_y_New,end_x_New,end_y_New;
			if (start_x > OLED_DIM_WIDTH || start_y > OLED_DIM_HEIGHT
					|| end_x > OLED_DIM_WIDTH || end_y > OLED_DIM_HEIGHT) {
				return;
			}
			start_x_New=start_x;
			start_y_New=0x7F-end_y;
			end_x_New=end_x;
			end_y_New=0x7F-start_y;

			GPIOA->ODR &= ~(1 << 6);	//reset cs
			GPIOA->ODR &= ~(1 << 7);	// reset dc
					USART_AS_SPI_sendCMD(SET_DISPLAY_ROW_ADD);
					USART_AS_SPI_sendCMD(start_x_New/1);
					USART_AS_SPI_sendCMD(end_x_New/1);
					USART_AS_SPI_sendCMD(SET_DISPLAY_COL_ADD);
					USART_AS_SPI_sendCMD(start_y_New/2);
					USART_AS_SPI_sendCMD(end_y_New/2);
			GPIOA->ODR |= 1 << 7;	//set dc
			GPIOA->ODR |= 1 << 6;	//set cs
		}
//========================================================================================================================
	void weoReset(void) {
		HAL_Delay(1);
//		LL_GPIO_ResetOutputPin(GPIOC, DISP_RES_Pin);
		HAL_Delay(1);
//		LL_GPIO_SetOutputPin(GPIOC, DISP_RES_Pin);
		HAL_Delay(1);
	}
	void I2C_SOUND_ChangePage(uint8_t pageNum) {
		uint8_t buf[] = { 0x00, pageNum };
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) 0x30, buf, 2, 1000);
	}
	void WriteReg_I2C_SOUND(uint8_t reg, uint8_t data) {
		uint8_t buf[] = { reg, data };
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) 0x30, buf, 2, 1000);	//(uint8_t*)&
	}
	void soundSetup(void) {
		I2C_SOUND_ChangePage(0x00);
		WriteReg_I2C_SOUND(0x01, 0x01);	//Assert Software reset (P0, R1, D0=1)
		I2C_SOUND_ChangePage(0x01);
		WriteReg_I2C_SOUND(0x02, 0x00);	//LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
		HAL_Delay(15);
		I2C_SOUND_ChangePage(0x00);
	WriteReg_I2C_SOUND(0x04,0x03);//PLL_clkin = MCLK, codec_clkin = PLL_CLK, MCLK should be 11.2896MHz
	WriteReg_I2C_SOUND(0x05,0x91);//Power up PLL, set P=1, R=1
	WriteReg_I2C_SOUND(0x06,0x04);//Set J = 4
	WriteReg_I2C_SOUND(0x07,0x00);//SET D=0000
	WriteReg_I2C_SOUND(0x08,0x00);//SET D=0000
	HAL_Delay(15);
	WriteReg_I2C_SOUND(0x0B,0x84);//DAC NDAC Powered up, set NDAC = 4
	WriteReg_I2C_SOUND(0x0C,0x82);//DAC MDAC Powered up, set MDAC = 2
	WriteReg_I2C_SOUND(0x0D,0x00);//DAC OSR(9:0)-> DOSR=128
	WriteReg_I2C_SOUND(0x0E,0x80);//DAC OSR(9:0)
		WriteReg_I2C_SOUND(0x1B, 0x00);	//Codec Interface control Word length = 16bits, BCLK&WCLK inputs, I2S mode. (P0, R27, D7-D6=00, D5-D4=00, D3-D2=00)
	WriteReg_I2C_SOUND(0x1C,0x00);//Data slot offset
	WriteReg_I2C_SOUND(0x3c,0x02);//Dac Instruction programming PRB #2 for Mono routing. Type interpolation (x8) and 3 programmable	Biquads. (P0, R60, D4-D0=0010)
		I2C_SOUND_ChangePage(0x01);
		WriteReg_I2C_SOUND(0x01, 0x10);	//Master Reference Powered on (P1, R1, D4=1)
		WriteReg_I2C_SOUND(0x0A, 0x00);	//Output common mode for DAC set to 0.9V (default) (P1, R10)
		////////////////////////////////////////////////////////////////////
		WriteReg_I2C_SOUND(0x0C, 0x00);	// No analog routing to headphone//0x0c,0x15 by SB
		WriteReg_I2C_SOUND(0x16, 0x00);	// Headphone volume=0//0x16,0x75 by SB
		WriteReg_I2C_SOUND(0x18, 0x00);	//No need to enable Mixer M and Mixer P, AINL Voulme, 0dB Gain 0x18,0x75 by SB
		WriteReg_I2C_SOUND(0x19, 0x00);	//No need to enable Mixer M and Mixer P, AINR Voulme, 0dB Gain 0x19,0x75 by SB
		//////////////////////////////////////////////////////////////////////
		WriteReg_I2C_SOUND(0x09, 0x00);	//AINL,AINR are powered down
		WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
		WriteReg_I2C_SOUND(0x2E, 0x00);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000)
		WriteReg_I2C_SOUND(0x30, 0x10);	//SPK driver Gain=6.0dB (P1, R48, D6-D4=001)
		WriteReg_I2C_SOUND(0x2D, 0x02);	//SPK powered up (P1, R45, D1=1)
		I2C_SOUND_ChangePage(0x00);
		//////////////////////////////////////////////////////////////////////////
		WriteReg_I2C_SOUND(0x3F, 0x90);	//DAC powered up, Soft step 1 per Fs. (P0, R63, D7=1, D5-D4=01, D3-D2=00, D1-D0=00)
		// 1<<7|1<<4|2 by SB
		WriteReg_I2C_SOUND(0x41, 0x00);	//DAC digital gain 0dB (P0, R65, D7-D0=00000000) cnDacValueOn by SB
		WriteReg_I2C_SOUND(0x40, 0x04);	//DAC volume not muted. (P0, R64, D3=0, D2=1) 1<<4 by SB
	}
//=============================================================================================
	uint8_t answer2CPU(uint8_t cmd[]) {
		uint8_t keyboard = 0xFF;
//		uint16_t ans[10];
		uint8_t i;
		uint8_t myCS = 0;					//   COMMON PART FOR EVERY COMMAND
		uint8_t myLength;
		isReceiverDisabled=1;

		cmd2Execute=0;
		cmd[0]&=~0x100;
		if ((cmd[0] == 0x11)||(cmd[0] == 0x12)||(cmd[0] == 0x13)||(cmd[0] == 0x14)||(cmd[0] == 0x15)) {GPIOC->ODR &= ~(1 << 6);}//reset BF
		if (cmd[0] == 0x11) {
//			GPIOC->ODR &= ~(1 << 6);
			GPIOC->ODR &=~ GPIO_ODR_OD6;
		}//reset BF
		ans[0] = cmd[0]|0x80;
//==================================================================================================
			if ((cmd[0] >= 0x10)&&(cmd[0] < 0x16)) { //answer is keyboard + stuff information                  0003
//		if (cmd[0] != 0x10) {
//			GPIOC->ODR &= ~(1 << 6);
				if ((GPIOA->IDR & (1 << 4)) == 0) {
					keyboard &= 0b11111110;
				}
				if ((GPIOA->IDR & (1 << 1)) == 0) {
					keyboard &= 0b11111101;
				}
				if ((GPIOA->IDR & (1 << 0)) == 0) {
					keyboard &= 0b11111011;
				}
				if ((GPIOA->IDR & (1 << 13)) == 0) {
					keyboard &= 0b11110111;
				}
				if ((GPIOA->IDR & (1 << 14)) == 0) {
					keyboard &= 0b11101111;
				}

				keyboard = ~keyboard;
//				keyboard&=bf;
				myLength = 0x04;
				ans[1] = myLength-0x02;
				ans[2] = keyboard;				//			MANAGEMENT COMMANDS

				for (i = 0; i < myLength-1; i++) {
					myCS = myCS + ans[i];
				}
				myCS = 0 - myCS;
				ans[3] = myCS;
				i=0;
//======================================================================================================================================

				while(!(USART2->ISR & USART_ISR_TXE)){};
				USART2->TDR = ans[0]|0x0100;
				for(i=1;i<myLength;i++)
				  {												//answer2cpu was given
				    while(!(USART2->ISR & USART_ISR_TXE)){};
				    USART2->TDR = (uint8_t)ans[i];
				  }
//				HAL_Delay(1);
				USART2->CR1 |= USART_CR1_RE;
				isReceiverDisabled=0;
//				BFEN=1;
//=======================================================================================================================================
				if (cmd[0] == 0x11) {//Show full screen background;
//					GPIOC->ODR &= ~(1 << 6);//reset BF
					picNum = cmd[2];
					cmd2Execute=0x11;
//					cmd[0]=0xFF;
					bf4me=0x00; //reset BF flag for me
				}
//=======================================================================================================================================
				if (cmd[0] == 0x12) {				//show small image
//					GPIOC->ODR &= ~(1 << 6);//reset BF
					imX = cmd[2];
					imY = cmd[3];
					picNum=cmd[4];
//					weoShowSmallImage(dataASCII[i], ASCII_X, ASCII_Y);
					cmd2Execute=0x12;
//					cmd[0]=0xFF;
					bf4me=0x00; //reset BF flag for me
				}
				if (cmd[0] == 0x13) {			//show ASCII code(s)
//					GPIOC->ODR &= ~(1 << 6);//reset BF
					imX = cmd[2];
					imY = cmd[3];
					fontInfo= cmd[4];
					color=fontInfo|0xF0;
					strLen = cmd[1] - 0x04;
					for (i = 0; i <strLen; i++) {
					dataASCII[i] = cmd[i+5];
				}
					cmd2Execute=0x13;
//					cmd[0]=0xFF;
					bf4me=0x00; //reset BF flag for me
				}
				if (cmd[0] == 0x14) {			//издать звук
					soundNum = cmd[2];
					cmd2Execute=0x14;
//					cmd[0]=0xFF;
					bf4me=0x00; //reset BF flag for me
				}
				if (cmd[0] == 0x15) {
					volume = cmd[2];
					contrast = cmd[3];
					cmd2Execute=0x15;
//					cmd[0]=0xFF;
					bf4me=0x00; //reset BF flag for me
				}
				if (cmd[0] == 0x16) {
					volume = cmd[3];
					contrast = cmd[4];
					cmd2Execute=0x16;
//					cmd[0]=0xFF;
					bf4me=0x00; //reset BF flag for me
				}
//				weoDrawRectangleFilled(0x00, 0x00, 0x7F, 0x7F, 0xFF, Image);
//				USART2->ICR|=USART_ICR_ORECF;
			}
//==========================================================================================================================
			if (cmd[0] == 0x00) { //request of pcb type
				myLength = 0x14; //20 bytes length answer
				ans[1] = myLength-0x02;
				for (i = 0; i < 17; i++) {
					ans[i + 2] = PCB_type[i];
				}
				for (i = 0; i < myLength-1; i++) {	//			PCB TYPE REQUEST
					myCS = myCS + ans[i];
				}
				myCS = 0 - myCS;
				ans[myLength - 1] = myCS;
				while(!(USART2->ISR & USART_ISR_TXE)){};
				USART2->TDR = ans[0]|0x0100;
				 while(ans[i])
				  {
				    i++;
				    while(!(USART2->ISR & USART_ISR_TXE)){};
				    USART2->TDR = (uint8_t)ans[i];
				  }
//				 cmd2Execute=0x00;
			}
//============================================================================================================================
			if (cmd[0] == 0x01) { //request of pcb revision
				myLength = 0x0B; //19 bytes length answer
				ans[1] = myLength-0x02;
				for (i = 0; i < 17; i++) {
					ans[i + 2] = PCB_rev[i];
				}
				for (i = 0; i < myLength-1; i++) {//			PCB REVISION REQUEST
					myCS = myCS + ans[i];
				}
				myCS = 0 - myCS;
				ans[myLength - 1] = myCS;
				while(!(USART2->ISR & USART_ISR_TXE)){};
				USART2->TDR = ans[0]|0x0100;
				 while(ans[i])
					  {
					    i++;
					    while(!(USART2->ISR & USART_ISR_TXE)){};
					    USART2->TDR = (uint8_t)ans[i];
					  }
//				 cmd2Execute=0x01;
			}
//================================================================================================================================
			if (cmd[0] == 0x02) { //request of emitter SN
				myLength = 0x13; //19 bytes length answer
				ans[1] = myLength-0x02;
				for (i = 0; i < 17; i++) {
					ans[i + 2] = EmitterSN[i];
				}
				for (i = 0; i < myLength-1; i++) {//			EMITTER SN REQUEST
					myCS = myCS + ans[i];
				}
				myCS = 0 - myCS;
				ans[myLength - 1] = myCS;
				while(!(USART2->ISR & USART_ISR_TXE)){};
				USART2->TDR = ans[0]|0x0100;
				while(ans[i])
				  {
				    i++;
				    while(!(USART2->ISR & USART_ISR_TXE)){};
				    	USART2->TDR = (uint8_t)ans[i];
				  }
//				cmd2Execute=0x02;
			}
//===============================================================================================================================
			if (cmd[0] == 0x03) { //request of current consumption
				myLength = 0x04; //4 bytes length answer
				ans[1] = myLength-0x02;
				ans[2] = currentConsumption;
				ans[3] = myCS;
				for (i = 0; i < myLength-1; i++) {
					myCS = myCS + ans[i];     //    CURRENT CONSUMPTION REQUEST
				}
				myCS = 0 - myCS;
				ans[myLength - 1] = myCS;
				while(!(USART2->ISR & USART_ISR_TXE)){};
					USART2->TDR = ans[0]|0x0100;
					while(ans[i])
						{
						  i++;
						  while(!(USART2->ISR & USART_ISR_TXE)){};
						     USART2->TDR = (uint8_t)ans[i];
						}
//					cmd2Execute=0x03;
			}
			if (cmd[0] == 0x04) { //request of current consumption
				myLength = 0x04; //4 bytes length answer
				ans[1] = myLength-0x02;
				uartSpeed = cmd[2];
				ans[2] = currentUARTspeed;
				ans[3] = myCS;
				for (i = 0; i < myLength-1; i++) {
					myCS = myCS + ans[i];     //    CURRENT CONSUMPTION REQUEST
				}
				myCS = 0 - myCS;
				ans[myLength - 1] = myCS;
				while(!(USART2->ISR & USART_ISR_TXE)){};
				USART2->TDR = ans[0]|0x0100;
				while(ans[i])
				{
				  i++;
				  while(!(USART2->ISR & USART_ISR_TXE)){};
				  USART2->TDR = (uint8_t)ans[i];
				}
//				cmd2Execute=0x04;
			}
//			weoDrawRectangleFilled(0x00, 0x00, 0x7F, 0x7F, 0xFF, coala);
			USART2->ICR|=USART_ICR_ORECF;
}
//==================================================================================================================================
	void MEM_Reset(void) {
		uint8_t memCMD;
		HAL_Delay(1); //200 ms by Andrew
		GPIOB->ODR &= ~(1 << 9); //reset cs
		memCMD = 0x66;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 5); //reset enable
		GPIOB->ODR |= 1 << 9; // set cs
		asm("NOP");
		__NOP();
		asm("NOP");
		__NOP();			//May be less NOPs?
		asm("NOP");
		__NOP();
		GPIOB->ODR &= ~(1 << 9);			//reset cs
		memCMD = 0x99;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 5); //reset memory
		GPIOB->ODR |= 1 << 9; // set cs
		HAL_Delay(1); //200 ms by Andrew
//=============================================================================================================
		memCMD = 0xB7;												//	Activation 0f 4-bytes address mode
		GPIOB->ODR &= ~(1 << 9); //reset cs
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 5); //reset enable
		GPIOB->ODR |= 1 << 9; // set cs
		HAL_Delay(1); //200 ms by Andrew
	}
//==================================================================================================================================
	uint8_t weoShowFullScreen(uint8_t picNum) {
		uint8_t memCMD,width,height,addr_l,addr_L,addr_h,addr_H;
		uint8_t MEM_Buffer[8192], firstImAddrArray[4],addrArray[4];
		uint16_t i, len;
		uint32_t addrInfo,addr,firstImAddr;
		memCMD = 0x13; //read command with 4-byte address
//		picNum=0x02;
		addr=(picNum)*0x2000;
//		addr=0x00003800;
		addrArray[0]=addr & 0xFF;
		addrArray[1]=(addr >> 8) & 0xFF;
		addrArray[2]=(addr >> 16) & 0xFF;
		addrArray[3]=(addr >> 24) & 0xFF;

		GPIOB->ODR &= ~(1 << 9); //reset cs
			HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 50); //read command with 4-byte address
			HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[3], 1, 50); //send address
			HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[2], 1, 50); //send address
			HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[1], 1, 50); //send address
			HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[0], 1, 50); //send address
			HAL_SPI_Receive(&hspi2, (uint8_t*) &MEM_Buffer,8192, 5000);
		GPIOB->ODR |= 1 << 9; // set cs

		weoDrawRectangleFilled(0x00, 0x00, 0x7F, 0x7F, 0xFF, MEM_Buffer); // Здесь ещё работает
		cmd2Execute=0;
//		while(BFEN==0){};
		GPIOC->ODR |= 1 << 6;	//set BF
	}
//==========================================================================================================================

	uint8_t weoShowFullScreenDMA(uint8_t picNum) {
		uint8_t memCMD, imByte;
//		uint8_t MEM_Buffer[8192];
		uint8_t DUMMY_Buffer[8192], firstImAddrArray[4],addrArray[4];
		uint16_t i;
		uint32_t addrInfo,addr;
		GPIOC->ODR &= ~(1 << 6);

		weoDrawRectangleInit(0x00, 0x00, 0x7F, 0x7F); // Здесь ещё работает

		len=8192;
		dma_spi_cnt=len;
		memCMD = 0x13; //read command with 4-byte address

		addr=((picNum)*0x2000);

		addrArray[0]=addr & 0xFF;
		addrArray[1]=(addr >> 8) & 0xFF;
		addrArray[2]=(addr >> 16) & 0xFF;
		addrArray[3]=(addr >> 24) & 0xFF;

		GPIOB->ODR &= ~(1 << 9); //reset cs

		HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 50); //read command with 4-byte address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[3], 1, 50); //send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[2], 1, 50); //send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[1], 1, 50); //send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[0], 1, 50); //send address
		HAL_SPI_Receive_DMA(&hspi2, (uint8_t*) &MEM_Buffer ,len);
	}
//==========================================================================================================================
	uint8_t weoShowSmallImage(uint8_t picNum, uint8_t imX, uint8_t imY) {

		uint8_t memCMD,width,height,addr_l,addr_L,addr_h,addr_H;
		uint8_t MEM_Buffer[8192];
		uint8_t imInfo[2],addrArray[4];
		uint16_t i;
		uint32_t addr,addrData;
		addr=0x00000000;
		memCMD = 0x13; //read command with 4-byte address
		//look at info about image
		addr=(picNum*0x2000)+0x200000;// the right path is to multiply picNum * image repeat period!
//		addr=(picNum*0x2000);

		addrArray[0]=addr & 0xFF;
		addrArray[1]=(addr >> 8) & 0xFF;
		addrArray[2]=(addr >> 16) & 0xFF;
		addrArray[3]=(addr >> 24) & 0xFF;

		GPIOB->ODR &= ~(1 << 9); //reset cs
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 50); //read command with 4-byte address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[3], 1, 50); //send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[2], 1, 50); //send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[1], 1, 50); //send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[0], 1, 50); //send address
		HAL_SPI_Receive(&hspi2, (uint8_t*) &imInfo,2, 5000);
		GPIOB->ODR |= 1 << 9; // set cs

		width=imInfo[0];
		height=imInfo[1];

//		if(height % 2 ==0){
//			height-=0x01;
//		}

//		len=(width*(height/2));

//		if(width % 2 !=0){
//			width-=0x01;
//		}
		len=((width+0)*((height)/2));

		addrData=addr+0x02;
		addrArray[0]=addrData & 0xFF;
		addrArray[1]=(addrData >> 8) & 0xFF;
		addrArray[2]=(addrData >> 16) & 0xFF;
		addrArray[3]=(addrData >> 24) & 0xFF;

		USART2->ICR|=USART_ICR_ORECF;
		memCMD = 0x13; //read command with 4-byte address

		GPIOB->ODR &= ~(1 <<9);	//reset cs
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 50);//read command with 4-byte address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[3],1, 50);	//send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[2],1, 50);	//send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[1],1, 50);	//send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[0],1, 50);	//send address
		HAL_SPI_Receive(&hspi2, (uint8_t*) &MEM_Buffer,len, 5000);// 7 information bits about the image
		GPIOB->ODR |= 1 << 9;	// set cs

		decY=1;
//		if(imY % 2 !=0){
//			decY=2;
//		}
//		imX = 0;
//		imY = 0;
//		weoDrawRectangleFilled(imX, imY, imX+width-1, imY+height-decY, 0xFF,MEM_Buffer);//classic	// Здесь ещё работает 0xFF - затычка
		weoDrawRectangleFilled(imX, imY, imX + width-1, (imY + height-1), 0xFF, MEM_Buffer);
		cmd2Execute=0;
//		while(BFEN==0){};
		GPIOC->ODR |= 1 << 6;	//set BF
	}
	uint8_t weoShowSmallImageDMA(uint8_t picNum, uint8_t imX, uint8_t imY) {

		uint8_t memCMD,width,height,addr_l,addr_L,addr_h,addr_H;
		uint8_t MEM_Buffer[8192], imInfo[2],addrArray[4];
		uint16_t i;
		uint32_t addr,addrData;
		addr=0x00000000;
		memCMD = 0x13; //read command with 4-byte address
		//look at info about image
		addr=(picNum*0x2000)+0x200000;;// the right path is to multiply picNum * image repeat period!

		addrArray[0]=addr & 0xFF;
		addrArray[1]=(addr >> 8) & 0xFF;
		addrArray[2]=(addr >> 16) & 0xFF;
		addrArray[3]=(addr >> 24) & 0xFF;

		GPIOB->ODR &= ~(1 << 9); //reset cs
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 50); //read command with 4-byte address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[3], 1, 50); //send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[2], 1, 50); //send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[1], 1, 50); //send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[0], 1, 50); //send address
		HAL_SPI_Receive(&hspi2, (uint8_t*) &imInfo,2, 5000);
		GPIOB->ODR |= 1 << 9; // set cs

		width=imInfo[0];
		height=imInfo[1];
		addrData=addr+0x02;
		len=((width/2)*(height+1));
		addrArray[0]=addrData & 0xFF;
		addrArray[1]=(addrData >> 8) & 0xFF;
		addrArray[2]=(addrData >> 16) & 0xFF;
		addrArray[3]=(addrData >> 24) & 0xFF;

		USART2->ICR|=USART_ICR_ORECF;
		memCMD = 0x13; //read command with 4-byte address

		GPIOB->ODR &= ~(1 <<9);	//reset cs
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 50);//read command with 4-byte address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[3],1, 50);	//send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[2],1, 50);	//send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[1],1, 50);	//send address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrArray[0],1, 50);	//send address
		HAL_SPI_Receive_DMA(&hspi2, (uint8_t*) &MEM_Buffer ,len);
	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	uint8_t soundLoad(uint8_t soundNum, uint8_t soundReady, uint8_t isSoundOver) {
			uint8_t memCMD,addr_l,addr_L,addr_h,addr_H;
			uint8_t soundInfo[9],addrINFO[4],addr[4],length[4];
			uint32_t i;
			memCMD = 0x13; //read command with 4-byte address

			if(isSoundOver == 1){return;}

			if(soundReady == 0){
				if (curBuf == 0){

					address=startAddressForSoundInfo+(soundNum*0x09);

					addrINFO[0]=address & 0xFF;
					addrINFO[1]=(address >> 8) & 0xFF;
					addrINFO[2]=(address >> 16) & 0xFF;
					addrINFO[3]=(address >> 24) & 0xFF;

					GPIOB->ODR &= ~(1 << 9); //reset cs
					HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 50); //read command with 4-byte address
					HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrINFO[3], 1, 50); //send address
					HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrINFO[2], 1, 50); //send address
					HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrINFO[1], 1, 50); //send address
					HAL_SPI_Transmit(&hspi2, (uint8_t*) &addrINFO[0], 1, 50); //send address
					HAL_SPI_Receive(&hspi2, (uint8_t*) &soundInfo,9, 5000);//9 bits of soundInfo
					GPIOB->ODR |= 1 << 9; // set cs

					addr[0]=soundInfo[4];
					addr[1]=soundInfo[3];
					addr[2]=soundInfo[2];
					addr[3]=soundInfo[1];


					length[0]=soundInfo[8];
					length[1]=soundInfo[7];
					length[2]=soundInfo[6];
					length[3]=soundInfo[5];

					len|=length[3];
					len<<=8;
					len|=length[2];
					len<<=8;
					len|=length[1];
					len<<=8;
					len|=length[0];

					addrSound|=addr[3];
					addrSound<<=8;
					addrSound|=addr[2];
					addrSound<<=8;
					addrSound|=addr[1];
					addrSound<<=8;
					addrSound|=addr[0];

					soundLen = len;
					bufCount = len / bufLen;
			}
//			if(curBuf == (bufCount)){
//				return;
//			}
//				addrSound += (bufLen * curBuf);
				if (curBuf != 0){
					addr[0]=addrSound & 0xFF;
					addr[1]=(addrSound >> 8) & 0xFF;
					addr[2]=(addrSound >> 16) & 0xFF;
					addr[3]=(addrSound >> 24) & 0xFF;
				}

//			if(soundReady == 0){
				GPIOB->ODR &= ~(1 << 9); //reset FLASH CS
				HAL_SPI_Transmit(&hspi2, (uint8_t*) &memCMD, 1, 50); //read command with 4-byte address
				HAL_SPI_Transmit(&hspi2, (uint8_t*) &addr[3], 1, 50); //send address
				HAL_SPI_Transmit(&hspi2, (uint8_t*) &addr[2], 1, 50); //send address
				HAL_SPI_Transmit(&hspi2, (uint8_t*) &addr[1], 1, 50); //send address
				HAL_SPI_Transmit(&hspi2, (uint8_t*) &addr[0], 1, 50); //send address
				HAL_SPI_Receive(&hspi2, (uint8_t*)SOUND1, bufLen,5000);
//			GPIOB->ODR |= 1 << 9;	//set FLASH CS
				for (i = 0; i < bufLen; i++){
					SOUND2[i] = SOUND1[i];
				}
//				while(1){
//					HAL_I2S_Transmit(&hi2s1, (uint16_t*)SOUND2, bufLen, 5000);
//					HAL_Delay(500);
//				}
//			}
//			soundReady = 1;
				if ((curBuf == 0) && (soundReady == 0)){
					HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)SOUND2, bufLen);
//				curBuf = 0;
				}
			}
			if (soundReady ==1){
//				if (curBuf != 0){
					HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)SOUND2, bufLen);
					HAL_Delay(500);
//					while (!soundReady){}
//			soundReady = 1;
//			curBuf = 0;
//				}
			}

//			HAL_Delay(500);
//			SoundPlay();
		}
//=========================================================================================================================
	uint8_t soundPlay(uint8_t soundNum) {
//		HAL_Delay(500);

//		HAL_Delay(500);
			I2C_SOUND_ChangePage(0x01);
			WriteReg_I2C_SOUND(0x01, 0x00);
			I2C_SOUND_ChangePage(0x00);
			WriteReg_I2C_SOUND(0x41, 0x30);// 0x81 - 0x30 available
		//	I2C_SOUND_ChangePage(0x00);
			I2C_SOUND_ChangePage(0x01);
			WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
			WriteReg_I2C_SOUND(0x2E, 0x24);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000) FF- speaker muted, 0x00 - 0x74 - available
			curBuf = 0;
			soundLoad(soundNum, 0, 0);
//			HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)SOUND, 16384/2); //HAL_MAX_DELAY

//			HAL_I2S_Transmit(&hi2s1, (uint16_t*)SOUND,4096,5000);
//		squeak_double(signal);
		}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void MEM_Write(uint32_t addr) {
		uint8_t dat;
		GPIOB->ODR &= ~(1 <<9);	//reset cs
		dat = 0x06;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &dat, 1, 5);//write enable Is it neccessary, if WP pin is used????????????????????????????
		GPIOB->ODR |= 1 << 9;	// set cs
//	GPIOC->ODR |= 1 << 14;// Write Enable
		asm("NOP");
		asm("NOP");
		addr *= DATA_COUNT;
		GPIOB->ODR &= ~(1 <<9);	//reset cs
		dat = 0x12;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &dat, 1, 5);//write command with 4-byte address
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addr, 4, 5);	//send address
		for (uint16_t i = 0; i < DATA_COUNT; i++) {
			HAL_SPI_Transmit(&hspi2, (uint8_t*) &MEM_Buffer[i], 1, 5000);	//
		}
		GPIOB->ODR |= 1 << 9;	// set cs
		asm("NOP");
		asm("NOP");
		GPIOB->ODR &= ~(1 <<9);	//reset cs
		dat = 0x04;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &dat, 1, 5);//write disable Is it neccessary, if WP pin is used????????????????????????????
		GPIOB->ODR |= 1 << 9;	// set cs
//    GPIOC->ODR &= ~(1 << 14);//Write Disable
	}
	uint32_t MEM_GetID(void) {
		uint8_t dat;
		uint8_t tmp[1] = { 0x00 };
		dat = 0x9E;
		GPIOB->ODR &= ~(1 <<9);	//reset cs
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &dat, 1, 50);	//read ID command
		HAL_SPI_Receive(&hspi2, tmp, 1, 1000);
		MEM_ID = (uint32_t) tmp[0];
		MEM_ID <<= 8;                    			//MSB
		HAL_SPI_Receive(&hspi2, tmp, 1, 1000);
		MEM_ID += (uint32_t) tmp[0];
		MEM_ID <<= 8;
		HAL_SPI_Receive(&hspi2, tmp, 1, 1000);
		MEM_ID += (uint32_t) tmp[0];
		GPIOB->ODR |= 1 << 9;                    			// set cs
		return (MEM_ID);
	}
	void MEM_EraseImg(unsigned long addr) {
		GPIOB->ODR &= ~(1 << 9);                    			//reset cs
		uint8_t dat;
		dat = 0x06;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &dat, 1, 5); //write enable Is it neccessary, if WP pin is used????????????????????????????
		GPIOB->ODR |= 1 << 9;                    			// set cs
		//	GPIOC->ODR |= 1 << 14;// Write Enable
		asm("NOP");
		asm("NOP");
		addr = addr * PAGE_SIZE;
		GPIOB->ODR &= ~(1 << 9);    //reset cs
		dat = 0x21;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &dat, 1, 5); //subSector (4096 bytes) erase command
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addr, 4, 5);    //send address
		addr = (addr + 0x01) * PAGE_SIZE;
		dat = 0x21;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &dat, 1, 5); //subSector (4096 bytes) erase command
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &addr, 4, 5);    //send address
		GPIOB->ODR |= 1 << 9;    // set cs
		asm("NOP");
		asm("NOP");
		GPIOB->ODR &= ~(1 << 9);    //reset cs
		dat = 0x04;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &dat, 1, 5); //write disable Is it neccessary, if WP pin is used????????????????????????????
		GPIOB->ODR |= 1 << 9;    // set cs
		//    GPIOC->ODR &= ~(1 << 14);//Write Disable
	}
//==================================================================================================================================

	uint16_t Scount(void){
		LIS3DHreadData();

	}
//====================================================================================================================
	uint8_t cmdExecute(uint8_t cmd2Execute){
		if(cmd[0]==0x10){return;}	// protection against short peaks while cmd 10h
//		if(soundReady==0){return;}
//		if(cmd[0]==00){return;}
		if (bf4me!=0x00){return;}	// protection against false BF resets
		USART2->ICR|=USART_ICR_ORECF;

		if(cmd2Execute==0x01){

				}
		if(cmd2Execute==0x02){

				}
		if(cmd2Execute==0x03){

				}
		if(cmd2Execute==0x04){

				}
		if(cmd2Execute==0x10){

				}
		if(cmd2Execute==0x11){
			GPIOC->ODR &=~ GPIO_ODR_OD6;
			bf4me=0x11;	//set BF flag 4 me
//			if(cmd2Execute!=0){GPIOC->ODR &= ~(1 << 6);}	//reset BF
			weoShowFullScreen(picNum);
//			weoShowFullScreenDMA(picNum);
			USART2->ICR|=USART_ICR_ORECF;
				}
		if(cmd2Execute==0x12){
			bf4me=0x12;	//set BF flag 4 me
//			if(cmd2Execute!=0){GPIOC->ODR &= ~(1 << 6);}	//reset BF
//			weoShowSmallImageDMA(picNum,imX,imY);
			weoShowSmallImage(picNum,imX,imY);
				}
		if(cmd2Execute==0x13){
			bf4me=0x13;	//set BF flag 4 me
//			if(cmd2Execute!=0){GPIOC->ODR &= ~(1 << 6);}	//reset BF

//			printASCIIarray_old(imX,imY, strLen,dataASCII);
			printASCIIarray(imX,imY,strLen,fontInfo,dataASCII);
				}
		if(cmd2Execute==0x14){
//			if(soundReady!=1){return;}
			bf4me=0x14;	//set BF flag 4 me
			if(soundNum==0x01){
//				if(soundReady!=0){
//				soundReady=0;
//				USART2->ICR|=USART_ICR_ORECF;
////				squeak_single(signal);
//				USART2->ICR|=USART_ICR_ORECF;
//				}
			}
			if(soundNum==0x02){
//				soundReady=0;
//				squeak_double(signal);
			}
			if(soundNum==0x03){
//				soundReady=0;
//				squeak_triple(signal);
				}
			if(soundNum==0x04){
//				soundReady=0;
//				squeak_long(signal);
				}
//			if(soundNum!=1){
//			HAL_Delay(500);
//			}
			cmd2Execute=0;
//			while(BFEN==0){};
			GPIOC->ODR |= 1 << 6;	//set BF

		}
		if(cmd2Execute==0x15){
			bf4me=0x15;	//set BF flag 4 me
			I2C_SOUND_ChangePage(0x01);
//			WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
			if(volume==0x00){
				I2C_SOUND_ChangePage(0x01);
				WriteReg_I2C_SOUND(0x2E,0xFF);// mute
			}
			I2C_SOUND_ChangePage(0x01);
			WriteReg_I2C_SOUND(0x2E, volume);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000) FF- speaker muted, 0x00 - 0x74 - available

			if(contrast==0x00){
				weoClear();
			}
			else{
				GPIOA->ODR &= ~(1 << 6);	//reset cs
				GPIOA->ODR &= ~(1 << 7);	// reset dc
				USART_AS_SPI_sendCMD(0x81);	//Contrast Level
				USART_AS_SPI_sendCMD(contrast*0x10);
				GPIOA->ODR |= 1 << 7;	//set dc
				GPIOA->ODR |= 1 << 6;	//set cs
			}
//			bf4me=0x15;	//set BF flag 4 me
			cmd2Execute=0;
//			while(BFEN==0){};
			GPIOC->ODR |= 1 << 6;	//set BF
		}
		if(cmd2Execute==0x16){
			bf4me=0x16;	//set BF flag 4 me
				}
		if(cmd2Execute==0x00){

				}
		if(cmd2Execute==0x00){

				}
		if(cmd2Execute=0x00){

				}
//			}
//		}
		USART2->ICR|=USART_ICR_ORECF;
	}
//========================================================================================================================
	uint8_t printASCIIarray(uint8_t imX,uint8_t imY,uint8_t strLen,uint8_t fontInfo,uint8_t dataASCII[]){
			uint8_t X_increment,ASCII_height,X_width,ASCII_X,decY,fontCur,contrast,contrastH,contrastL;
//			uint8_t weoBuffer1[49],weoBuffer2[49],weoBuffer[49];
			uint16_t i,j,k, symLen;
			ASCII_X=imX;

			contrastL = (fontInfo & 0xF0)>>4;
			contrastH = (contrastL)<<4;
			contrast = contrastH|contrastL;
//			contrast = (fontInfo & 0xF0)>>4;
//			contrast=0x33;

			decY=0x01;
			if(imY % 2 !=0){
				decY=0x02;
			}
			if((fontInfo & 0x00)==0){
				fontCur=0;
			}
			if((fontInfo & 0x01)==1){
				fontCur=1;
			}
			if((fontInfo & 0x02)==2){
				fontCur=2;
			}
//			fontCur=2;
			if(fontCur==0){
				symLen=49;
				uint8_t weoBuffer[symLen];
				uint8_t weoBuffer1[symLen];
				uint8_t weoBuffer2[symLen];
				X_increment=0x07;
				ASCII_height=0x0E;
				for(i=0;i<strLen;i++){
					for(j=0;j<symLen;j++){
						weoBuffer[j]=F1[dataASCII[i]][j];
							}
//					for (k=0;k<symLen;k++){
//							weoBuffer1[k]=(weoBuffer[k]&0x0F)&contrast;
//							weoBuffer2[k]=((weoBuffer[k]&0xF0)>>4)&contrast;
//						}
//					for (k=0;k<symLen;k++){
//							weoBuffer[k]=(weoBuffer2[k]<<4)|weoBuffer1[k];
//						}
					for (k=0;k<symLen;k++){
						weoBuffer[k]=weoBuffer[k] & contrast;
					}
				weoDrawRectangleFilled(ASCII_X, imY, (ASCII_X+X_increment-1), imY + ASCII_height - decY, 0xFF, weoBuffer);
				ASCII_X += X_increment;
				}
				for(i=0;i<symLen;i++){
									weoBuffer[j]=0x00;
							}
			}
			if(fontCur==1){
				symLen=63;
				uint8_t weoBuffer[symLen];
				uint8_t weoBuffer1[symLen];
				uint8_t weoBuffer2[symLen];
				X_increment=0x07;
				ASCII_height=0x12;
				for(i=0;i<strLen;i++){
					for(j=0;j<symLen;j++){
						weoBuffer[j]=F2[dataASCII[i]][j];
							}
//					for (k=0;k<symLen;k++){
//							weoBuffer1[k]=(weoBuffer[k]&0x0F)&contrast;
//							weoBuffer2[k]=((weoBuffer[k]&0xF0)>>4)&contrast;
//						}
//					for (k=0;k<symLen;k++){
//							weoBuffer[k]=(weoBuffer2[k]<<4)|weoBuffer1[k];
//						}
					for (k=0;k<symLen;k++){
						weoBuffer[k]=weoBuffer[k] & contrast;
					}
				weoDrawRectangleFilled(ASCII_X, imY, (ASCII_X+X_increment-1), imY + ASCII_height - decY, 0xFF, weoBuffer);
				ASCII_X += X_increment;
				}
				for(i=0;i<symLen;i++){
									weoBuffer[j]=0x00;
							}
			}
			if(fontCur==2){
				symLen=304;
				uint8_t weoBuffer[symLen];
				uint8_t weoBuffer1[symLen];
				uint8_t weoBuffer2[symLen];
				X_increment=0x10;
				ASCII_height=0x26;
				for(i=0;i<strLen;i++){
					if(dataASCII[i]==0x20){dataASCII[i]=0x00;}
					if(dataASCII[i]==0x21){dataASCII[i]=0x01;}
					if(dataASCII[i]==0x25){dataASCII[i]=0x02;}
					if(dataASCII[i]==0x28){dataASCII[i]=0x03;}
					if(dataASCII[i]==0x29){dataASCII[i]=0x04;}
					if(dataASCII[i]==0x2a){dataASCII[i]=0x05;}
					if(dataASCII[i]==0x2b){dataASCII[i]=0x06;}
					if(dataASCII[i]==0x2c){dataASCII[i]=0x07;}
					if(dataASCII[i]==0x2d){dataASCII[i]=0x08;}
					if(dataASCII[i]==0x2e){dataASCII[i]=0x09;}
					if(dataASCII[i]==0x2f){dataASCII[i]=0x0a;}
					if(dataASCII[i]==0x30){dataASCII[i]=0x0b;}
					if(dataASCII[i]==0x31){dataASCII[i]=0x0c;}
					if(dataASCII[i]==0x32){dataASCII[i]=0x0d;}
					if(dataASCII[i]==0x33){dataASCII[i]=0x0e;}
					if(dataASCII[i]==0x34){dataASCII[i]=0x0f;}
					if(dataASCII[i]==0x35){dataASCII[i]=0x10;}
					if(dataASCII[i]==0x36){dataASCII[i]=0x11;}
					if(dataASCII[i]==0x37){dataASCII[i]=0x12;}
					if(dataASCII[i]==0x38){dataASCII[i]=0x13;}
					if(dataASCII[i]==0x39){dataASCII[i]=0x14;}
					if(dataASCII[i]==0x3a){dataASCII[i]=0x15;}
					if(dataASCII[i]==0x3b){dataASCII[i]=0x16;}
					if(dataASCII[i]==0x3c){dataASCII[i]=0x17;}
					if(dataASCII[i]==0x3d){dataASCII[i]=0x18;}
					if(dataASCII[i]==0x3e){dataASCII[i]=0x19;}
					if(dataASCII[i]==0x3f){dataASCII[i]=0x1a;}
					if(dataASCII[i]==0x5b){dataASCII[i]=0x1b;}
					if(dataASCII[i]==0x5c){dataASCII[i]=0x1c;}
					if(dataASCII[i]==0x5d){dataASCII[i]=0x1d;}
					if(dataASCII[i]==0x5f){dataASCII[i]=0x1e;}
					if(dataASCII[i]==0x7c){dataASCII[i]=0x1f;}
				}
				for(i=0;i<strLen;i++){
					for(j=0;j<symLen;j++){
						weoBuffer[j]=F3[dataASCII[i]][j];
							}
//					for (k=0;k<symLen;k++){
//							weoBuffer1[k]=(weoBuffer[k]&0x0F)&contrast;
//							weoBuffer2[k]=((weoBuffer[k]&0xF0)>>4)&contrast;
//						}
//					for (k=0;k<symLen;k++){
//							weoBuffer[k]=(weoBuffer2[k]<<4)|weoBuffer1[k];
//						}
					for (k=0;k<symLen;k++){
						weoBuffer[k]=weoBuffer[k] & contrast;
					}
				weoDrawRectangleFilled(ASCII_X, imY, ASCII_X+X_increment-1, imY + ASCII_height - decY, 0xFF, weoBuffer);
				ASCII_X += X_increment;
				}
				for(i=0;i<symLen;i++){
									weoBuffer[j]=0x00;
							}
			}
			cmd2Execute=0;
//			while(BFEN==0){};
			GPIOC->ODR |= 1 << 6;	//set BF

	}
//=============================================================================================================
	void squeak_generate(void){
//			    uint16_t nsamples = sizeof(signal) / sizeof(signal[0]);
			    uint16_t k = 0;
			    while(k < nsamples) {
			        double t = ((double)k/2.0)/((double)nsamples);
//			       signal[k] = 32767*sin(100.0 * TAU * t); // left
			        k += 1;
			    }
		}
//=============================================================================================================
	void squeak_single(uint16_t* signal){

		I2C_SOUND_ChangePage(0x01);
		WriteReg_I2C_SOUND(0x01, 0x00);
		I2C_SOUND_ChangePage(0x00);
		WriteReg_I2C_SOUND(0x41, 0x30);// 0x81 - 0x30 available
	//	I2C_SOUND_ChangePage(0x00);
		I2C_SOUND_ChangePage(0x01);
		WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
		WriteReg_I2C_SOUND(0x2E, 0x24);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000) FF- speaker muted, 0x00 - 0x74 - available
		HAL_I2S_Transmit_DMA(&hi2s1, (const uint16_t*)signal, nsamples); //HAL_MAX_DELAY
		USART2->ICR|=USART_ICR_ORECF;
		USART2->ICR|=USART_ICR_FECF;
		USART2->ICR|=USART_ICR_NECF;
//		HAL_Delay(100);
//		GPIOC->ODR |= 1 << 6;	//set BF
//		cmd2Execute=0;
	}
//==========================================================================================================================
	void squeak_long(uint16_t* signal){

		I2C_SOUND_ChangePage(0x01);
		WriteReg_I2C_SOUND(0x01, 0x00);
		I2C_SOUND_ChangePage(0x00);
		WriteReg_I2C_SOUND(0x41, 0x30);// 0x81 - 0x30 available
	//	I2C_SOUND_ChangePage(0x00);
		I2C_SOUND_ChangePage(0x01);
		WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
		WriteReg_I2C_SOUND(0x2E, 0x24);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000) FF- speaker muted, 0x00 - 0x74 - available
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
		HAL_I2S_Transmit(&hi2s1, (uint16_t*)signal, nsamples,5000);
//		HAL_Delay(500);
		GPIOC->ODR |= 1 << 6;	//set BF
		cmd2Execute=0;
	}
//=============================================================================================================
	void squeak_double(uint16_t* signal){

		I2C_SOUND_ChangePage(0x01);
		WriteReg_I2C_SOUND(0x01, 0x00);
		I2C_SOUND_ChangePage(0x00);
		WriteReg_I2C_SOUND(0x41, 0x30);// 0x81 - 0x30 available
	//	I2C_SOUND_ChangePage(0x00);
		I2C_SOUND_ChangePage(0x01);
		WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
		WriteReg_I2C_SOUND(0x2E, 0x24);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000) FF- speaker muted, 0x00 - 0x74 - available
		HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)signal, nsamples);
		HAL_Delay(100);
		HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)signal, nsamples);
//		HAL_Delay(500);
	}
//=============================================================================================================
	void squeak_triple(uint16_t* signal){

		I2C_SOUND_ChangePage(0x01);
		WriteReg_I2C_SOUND(0x01, 0x00);
		I2C_SOUND_ChangePage(0x00);
		WriteReg_I2C_SOUND(0x41, 0x30);// 0x81 - 0x30 available
	//	I2C_SOUND_ChangePage(0x00);
		I2C_SOUND_ChangePage(0x01);
//		HAL_Delay(1000);
		WriteReg_I2C_SOUND(0x10, 0x00);	//Headphone is muted// 1<<6 by SB
		WriteReg_I2C_SOUND(0x2E, 0x24);	//SPK attn. Gain =0dB (P1, R46, D6-D0=000000) FF- speaker muted, 0x00 - 0x74 - available
		HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)signal, nsamples);
		HAL_Delay(100);
		HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)signal, nsamples);
		HAL_Delay(100);
		HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)signal, nsamples);
//		HAL_Delay(500);
//		GPIOC->ODR |= 1 << 6;	//set BF
//		cmd2Execute=0;
	}
//=============================================================================================================
	void LIS3DHsendCMD(uint8_t reg, uint8_t data) {
		uint8_t buf[] = { reg, data };
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) 0x32, buf, 2, 1000);	//32h - address for writing
	}
//=============================================================================================================
	uint8_t LIS3DHreadReg(uint8_t reg){
			uint8_t value;
//			HAL_I2C_Master_Receive(&hi2c1, (uint16_t) 0x33, reg,&value, 1, 0x10000);	//33h - address for reading
//			HAL_I2C_Mem_Read(&hi2c1, (uint8_t) 0x33, reg,I2C_MEMADD_SIZE_8BIT,&value, 1, 0x10000);	//33h - address for reading
			HAL_I2C_Mem_Read(&hi2c1, (uint8_t) 0x32, reg,1,&value, 1, 0x10000);	//33h - address for reading
			xVal=value;
			return value;
	}
//=============================================================================================================
//	void LIS3DHreadData(void){
//			accelBuff[0]=LIS3DHreadReg(OUTXH);
//			accelBuff[1]=LIS3DHreadReg(OUTXL);
//			accelBuff[2]=LIS3DHreadReg(OUTYH);
//			accelBuff[3]=LIS3DHreadReg(OUTYL);
//			accelBuff[4]=LIS3DHreadReg(OUTZH);
//			accelBuff[5]=LIS3DHreadReg(OUTZL);
//			xVal=accelBuff[0];
//			xVal=xVal<<8;
//			xVal|=accelBuff[1];
//			yVal=accelBuff[2];
//			yVal=yVal<<8;
//			yVal|=accelBuff[3];
//			zVal=accelBuff[4];
//			zVal=zVal<<8;
//			zVal|=accelBuff[5];
//	}
//=============================================================================================================
//	uint16_t Accel_ReadAcc(void){
//	        int16_t buffer[3] = {0};
//	        uint16_t tmp16 = 0;
//	        Accel_GetXYZ(buffer);
//	  xVal = buffer[0];
//	  yVal = buffer[1];
//	  zVal = buffer[2];
//	}
//=============================================================================================================
	void LIS3DHsetup(void){
		uint8_t CTRL_REG1_val=0x00;
		uint8_t CTRL_REG4_val=0x00;
		uint8_t CTRL_REG5_val=0x00;
		uint8_t CTRL_REG6_val=0x00;
		uint8_t FIFO_CTRL_REG_val=0x00;
		uint8_t FIFO_SRC_REG_val=0x00;
		uint8_t INT_1_CFG_val=0x00;
		uint8_t INT_1_SRC_val=0x00;

		LIS3DHsendCMD(CTRL_REG1,(CTRL_REG1_val|accelDataRate_25_Hz|Xen|Yen|Zen));//data rate selection
//		LIS3DHsendCMD(CTRL_REG2,);//HPFilter
//		LIS3DHsendCMD(CTRL_REG3,);
		LIS3DHsendCMD(CTRL_REG4,(CTRL_REG4_val|BDU|FULL_SCALE_2G|HR));
//		LIS3DHsendCMD(CTRL_REG5,CTRL_REG4_val|FIFO_EN);
//		LIS3DHsendCMD(CTRL_REG6,);
//		LIS3DHsendCMD(FIFO_CTRL_REG,FIFO_CTRL_REG_val);	//	2B configured
//		LIS3DHsendCMD(FIFO_SRC_REG,FIFO_SRC_REG_val);	//	2B configured
//		LIS3DHsendCMD(INT_1_CFG,INT_1_CFG_val);	//	2B configured
//		LIS3DHsendCMD(INT_1_SRC_val,INT_1_SRC_val);	//	2B configured
	}
	void GPIO_2_3state(void){
//		GPIOB->CRH &= ~(0x)
	}
//============================================================================================================

//=============================================================================================================
uint8_t test[49] = {
	    //∙∙∙∙∙∙∙∙∙∙∙∙∙∙
	    //∙∙█████████∙∙∙
	    //∙∙∙███████∙∙∙∙
	    //∙∙∙∙█████∙∙∙∙∙
	    //∙∙∙∙∙███∙∙∙∙∙∙
	    //∙∙∙∙∙∙█∙∙∙∙∙∙∙
	    //∙∙∙∙∙∙∙∙∙∙∙∙∙∙
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	    0x00, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00,
	    0x00, 0x0f, 0xff, 0xff, 0xff, 0x00, 0x00,
	    0x00, 0x00, 0xff, 0xff, 0xf0, 0x00, 0x00,
	    0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
uint8_t aim[49]={
	    //██████████████
	    //█∙∙∙███∙∙∙∙∙∙█
	    //█∙∙∙█∙∙██∙∙∙∙█
	    //█∙∙∙█∙∙∙∙██∙∙█
	    //█∙██████████∙█
	    //█∙∙∙█∙∙∙∙∙∙∙∙█
	    //██████████████
	    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	    0xf0, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x0f,
	    0xf0, 0x00, 0xf0, 0x0f, 0xf0, 0x00, 0x0f,
	    0xf0, 0x00, 0xf0, 0x00, 0x0f, 0xf0, 0x0f,
	    0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f,
	    0xf0, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x0f,
	    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
uint8_t frame[99]={
	    //██████████████████
	    //█∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙█
	    //█∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙█
	    //█∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙█
	    //█∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙█
	    //█∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙█
	    //█∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙█
	    //█∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙█
	    //█∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙█
	    //█∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙█
	    //██████████████████
	    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
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
