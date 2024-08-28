/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union
{
		uint8_t 	B8[4];
		uint32_t	B32;
}B32_B8_TypeDef;

typedef enum
{
	COM_OK			=0x00U,
	COM_ERROR		=0x01U,
	COM_CHKERR	=0x02U
}Com_StatusTypeDef;

typedef enum
{
	OTP_SUCCESS	=0x01U,
	OTP_Fail=0x00U
}OTP_ProgStatusTpyeDef;

typedef struct
{
	uint16_t RMS_1;			//XX.XXX	A		0.001A/LSB
	uint16_t RMS_2;
	uint16_t RMS_3;
	uint16_t RMS_4;
	uint16_t RMS_5;
	uint16_t RMS_6;
	uint16_t RMS_7;
	uint16_t RMS_8;
	uint16_t RMS_9;
	uint16_t RMS_10;
	uint16_t RMS_V;			//XXX.X	 V			0.1Volts/LSB
	uint16_t WATT_1;		//XXXX.X W		    0.1Watt/LSB
	uint16_t WATT_2;
	uint16_t WATT_3;
	uint16_t WATT_4;
	uint16_t WATT_5;
	uint16_t WATT_6;
	uint16_t WATT_7;
	uint16_t WATT_8;
	uint16_t WATT_9;
	uint16_t WATT_10;
	uint16_t Energy_1;		//XXXXXX	kWh   Degree of electricity/LSB, maximum 65,535 degrees of electricity
	uint16_t Energy_2;
	uint16_t Energy_3;
	uint16_t Energy_4;
	uint16_t Energy_5;
	uint16_t Energy_6;
	uint16_t Energy_7;
	uint16_t Energy_8;
	uint16_t Energy_9;
	uint16_t Energy_10;
	uint16_t Energy_Sum;
	uint16_t Period;		//XX.XX	Hz		0.01Hz/LSB
	uint16_t TPS1;			//XX.XX   			0.01 °C/LSB
	uint16_t TPS2;
}Elect_StructDef;

typedef struct
{
	uint16_t	Cnt1;
	uint16_t	Cnt2;
	uint16_t	Cnt3;
	uint16_t	Cnt4;
	uint16_t	Cnt5;
	uint16_t	Cnt6;
	uint16_t	Cnt7;
	uint16_t	Cnt8;
	uint16_t	Cnt9;
	uint16_t	Cnt10;
	uint16_t  Cnt_Sum;
}Eng_CFCnt_StructDef;

//Data for electrical energy calculation
typedef struct
{
	uint16_t Energy001;		 //Electricity accumulation of more than 1 degree
	uint16_t cnt_remainder;	 //pulse base of 1 degree of electricity
}Eng_Cal_StructDef;


typedef struct {
	float Current1;
	float Current2;
	float Current3;
	float Current4;
	float Current5;

	float AP1; //Active Power
	float AP2;
	float AP3;
	float AP4;
	float AP5;

    float energy_value1;
    float energy_value2;
    float energy_value3;
    float energy_value4;
    float energy_value5;

    float Voltage;
    float Freq;
} BL0910Data;

//typedef struct{
//	bool swa;
//	bool swb;
//	bool swc;
//	bool swd;
//	bool swe;
//
//}SwitchTypedef;  //5 digital input value (Manual Button)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Current_K	    19154
#define Voltage_k	    9000
#define Power_K		    54805
#define Energy_K	    8972
#define Energy_Sum_K 	560

//#define Power_K		    3425
//#define Current_K	    11971 193653.14
#define Sys_Rst								0
#define CalReg_RST							1

#define Addr_VERSION						0x00
#define	Addr_1_WAVE							0x01
#define	Addr_2_WAVE							0x02
#define	Addr_3_WAVE							0x03
#define	Addr_4_WAVE							0x04
#define	Addr_5_WAVE							0x05
#define	Addr_6_WAVE							0x06
#define	Addr_7_WAVE							0x07
#define	Addr_8_WAVE							0x08
#define	Addr_9_WAVE							0x09
#define	Addr_10_WAVE						0x0A
#define	Addr_V_WAVE							0x0B

#define Addr_1_RMS							0x0C
#define Addr_2_RMS							0x0D
#define Addr_3_RMS							0x0E
#define Addr_4_RMS							0x0F
#define Addr_5_RMS							0x10
#define Addr_6_RMS							0x11
#define Addr_7_RMS							0x12
#define Addr_8_RMS							0x13
#define Addr_9_RMS							0x14
#define Addr_10_RMS							0x15

#define Addr_V_RMS							0x16
#define Addr_1_FAST_RMS						0x17
#define Addr_2_FAST_RMS						0x18
#define Addr_3_FAST_RMS						0x19
#define Addr_4_FAST_RMS						0x1A
#define Addr_5_FAST_RMS						0x1B
#define Addr_6_FAST_RMS						0x1C
#define Addr_7_FAST_RMS						0x1D
#define Addr_8_FAST_RMS						0x1E
#define Addr_9_FAST_RMS						0x1F
#define Addr_10_FAST_RMS					0x20
#define Addr_V_FAST_RMS						0x21

#define Addr_WATT1							0x22 //Active power
#define Addr_WATT2							0x23
#define Addr_WATT3							0x24
#define Addr_WATT4							0x25
#define Addr_WATT5							0x26
#define Addr_WATT6							0x27
#define Addr_WATT7							0x28
#define Addr_WATT8							0x29
#define Addr_WATT9							0x2A
#define Addr_WATT10							0x2B
#define Addr_WATT							0x2C //REG watt total

#define Addr_VAR							0x2D
#define Addr_VA								0x2E
#define Addr_CF1_CNT						0x2F
#define Addr_CF2_CNT						0x30
#define Addr_CF3_CNT						0x31
#define Addr_CF4_CNT						0x32
#define Addr_CF5_CNT						0x33
#define Addr_CF6_CNT						0x34
#define Addr_CF7_CNT						0x35
#define Addr_CF8_CNT						0x36
#define Addr_CF9_CNT						0x37
#define Addr_CF10_CNT						0x38
#define Addr_CF_CNT							0x39
#define Addr_CFQ_CNT						0x3A
#define Addr_CFS_CNT						0x3B
#define Addr_ANGLE1							0x3C
#define Addr_ANGLE2							0x3D
#define Addr_ANGLE3							0x3E
#define Addr_ANGLE4							0x3F
#define Addr_ANGLE5							0x40
#define Addr_ANGLE6							0x41
#define Addr_ANGLE7							0x42
#define Addr_ANGLE8							0x43
#define Addr_ANGLE9							0x44
#define Addr_ANGLE10						0x45
#define Addr_1_FAST_RMS_HOLD		0x46
#define Addr_2_FAST_RMS_HOLD		0x47
#define Addr_3_FAST_RMS_HOLD		0x48
#define Addr_4_FAST_RMS_HOLD		0x49
#define Addr_PF								0x4A
#define Addr_LINE_WATTHR					0x4B
#define Addr_LINE_VARHR						0x4C
#define Addr_SIGN							0x4D
#define Addr_PERIOD							0x4E

#define Addr_STATUS1						0x54
#define Addr_STATUS2						0x55
#define Addr_STATUS3						0x56
#define Addr_5_FAST_RMS_HOLD		0x57
#define Addr_6_FAST_RMS_HOLD		0x58
#define Addr_7_FAST_RMS_HOLD		0x59
#define Addr_8_FAST_RMS_HOLD		0x5A
#define Addr_9_FAST_RMS_HOLD		0x5B
#define Addr_10_FAST_RMS_HOLD		0x5C
#define Addr_VAR1							0x5D
#define Addr_TPS1							0x5E
#define Addr_TPS2							0x5F

#define Addr_GAIN1							0x60
#define Addr_GAIN2							0x61

#define Addr_PHASE1_2						0x64
#define Addr_PHASE3_4						0x65
#define Addr_PHASE5_6						0x66
#define Addr_PHASE7_8						0x67
#define Addr_PHASE9_10						0x68
#define Addr_PHASE11						0x69
#define Addr_VAR_PHCAL_I					0x6A
#define Addr_VAR_PHCAL_V					0x6B
#define Addr_RMSGN1							0x6C
#define Addr_RMSGN2							0x6D
#define Addr_RMSGN3							0x6E
#define Addr_RMSGN4							0x6F
#define Addr_RMSGN5							0x70
#define Addr_RMSGN6							0x71
#define Addr_RMSGN7							0x72
#define Addr_RMSGN8							0x73
#define Addr_RMSGN9							0x74
#define Addr_RMSGN10						0x75
#define Addr_RMSGN11						0x76
#define Addr_RMSOS1							0x77
#define Addr_RMSOS2							0x78
#define Addr_RMSOS3							0x79
#define Addr_RMSOS4							0x7A
#define Addr_RMSOS5							0x7B
#define Addr_RMSOS6							0x7C
#define Addr_RMSOS7							0x7D
#define Addr_RMSOS8							0x7E
#define Addr_RMSOS9							0x7F
#define Addr_RMSOS10						0x80
#define Addr_RMSOS11						0x81
#define Addr_WA_LOS1_2						0x82
#define Addr_WA_LOS3_4						0x83
#define Addr_WA_LOS5_6						0x84
#define Addr_WA_LOS7_8						0x85
#define Addr_WA_LOS9_10						0x86
#define Addr_VAR_LOS						0x87
#define Addr_VAR_WATT_CREEP					0x88
#define Addr_WA_CREEP2						0x89
#define	Addr_RMS_CREEP						0x8A
#define Addr_FAST_RMS_CTRL					0x8B
#define Addr_I_V_PKLVL						0x8C

#define Addr_SAGCYC_ZXTOUT					0x8E
#define Addr_SAGLVL_LINECYC					0x8F
#define Addr_FLAG_CTRL						0x90
#define Addr_FLAG_CTRL1						0x91
#define Addr_FLAG_CTRL2						0x92
#define Addr_ADC_PD							0x93
#define Addr_TPS_CTRL						0x94
#define Addr_TPS2_A_B						0x95
#define Addr_MODE1							0x96
#define Addr_MODE2							0x97
#define Addr_MODE3							0x98

#define Addr_MASK1							0x9A
#define Addr_MASK2							0x9B

#define Addr_RST_ENG						0x9D
#define Addr_USR_WRPROT					    0x9E
#define Addr_SOFT_RESET						0x9F

#define Addr_CHGN1							0xA0
#define Addr_CHGN2							0xA1
#define Addr_CHGN3							0xA2
#define Addr_CHGN4							0xA3
#define Addr_CHGN5							0xA4
#define Addr_CHGN6							0xA5
#define Addr_CHGN7							0xA6
#define Addr_CHGN8							0xA7
#define Addr_CHGN9							0xA8
#define Addr_CHGN10							0xA9
#define Addr_CHGN11							0xAA
#define Addr_CHOS1							0xAB
#define Addr_CHOS2							0xAC
#define Addr_CHOS3							0xAD
#define Addr_CHOS4							0xAE
#define Addr_CHOS5							0xAF
#define Addr_CHOS6							0xB0
#define Addr_CHOS7							0xB1
#define Addr_CHOS8							0xB2
#define Addr_CHOS9							0xB3
#define Addr_CHOS10							0xB4
#define Addr_CHOS11							0xB5
#define Addr_WATTGN1						0xB6
#define Addr_WATTGN2						0xB7
#define Addr_WATTGN3						0xB8
#define Addr_WATTGN4						0xB9
#define Addr_WATTGN5						0xBA
#define Addr_WATTGN6						0xBB
#define Addr_WATTGN7						0xBC
#define Addr_WATTGN8						0xBD
#define Addr_WATTGN9						0xBE
#define Addr_WATTGN10						0xBF
#define Addr_WATTOS1						0xC0
#define Addr_WATTOS2						0xC1
#define Addr_WATTOS3						0xC2
#define Addr_WATTOS4						0xC3
#define Addr_WATTOS5						0xC4
#define Addr_WATTOS6						0xC5
#define Addr_WATTOS7						0xC6
#define Addr_WATTOS8						0xC7
#define Addr_WATTOS9						0xC8
#define Addr_WATTOS10						0xC9
#define Addr_VARGN							0xCA
#define Addr_VAROS							0xCB
#define Addr_VAGN							0xCC
#define Addr_VAOS							0xCD
#define Addr_CFDIV						    0xCE


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Variables for electrical energy calculation, used for parameter transmission
Eng_Cal_StructDef Eng_Cal;
// The pulse base of 10 channels is less than 1 degree of electricity
Eng_CFCnt_StructDef Eng_CFCnt;
Elect_StructDef BL0910_Elect;
Com_StatusTypeDef	BL0910_ComSta;
B32_B8_TypeDef BL0910_Data;
uint8_t flg_FrameStart;
BL0910Data BL0910_package;
uint8_t relay_state[5] = {0, 0, 0, 0, 0};  // Assume all relays are off initially

static float vol=0.0,freq=0.0;
static float I1=0.0,I2=0.0,I3=0.0,I4=0.0,I5=0.0,
						 I6=0.0,I7=0.0,I8=0.0,I9=0.0,I10=0.0;
static float P1=0.0,P2=0.0,P3=0.0,P4=0.0,P5=0.0,
						 P6=0.0,P7=0.0,P8=0.0,P9=0.0,P10=0.0;
//static float fv = 0.01,fi=0.00003549,fp=0.002793561566656,ff=0.01;
static float fv = 0.01,fi=0.001,fp=0.1,ff=0.01;

//static const float BL0910_IREF = (12875.0f * 33.0f) / (1.097f * 2000.0f / 1000.0f); // magic1 * Rburden1 / Vref * CT ratio
//static const float BL0910_UREF = (13162.0f * 220.0f) / (1.097f * 132.0f); // magic2 * Rburden2 / Vref * PT primary resistance
//static const float BL0910_PREF = (40.4125f * 33.0f * 220.0f * 1000.0f / 2000.0f) / (1.097f * 1.097f * 132.0f); // magic3 * Rburden1 * Rburden2 / Vref * Vref * PT primary resistance * CT ratio
//static const float BL0910_EREF = (3600000.0f * 16.0f * BL0910_PREF) / (4194304.0f * 0.032768f * 16.0f);

volatile uint8_t relay_flag = 0;  // Flag to indicate relay control
volatile uint8_t relay_flag1 = 0;  // Flag to indicate relay control
volatile uint8_t button_pressed = 0;
volatile uint8_t button_state = 0;
 uint8_t mode1 = 0;
 uint8_t mode2 = 0;
 uint8_t mode3 = 0;
 uint8_t mode4 = 0;
 uint8_t mode5 = 0;
char rx_relay;
char rx_relay1;
uint8_t toggle_BT[5];
//SwitchTypedef BT_Buffer;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t ChkSum_Cal(uint8_t Cmd,uint8_t REG_Addr,uint8_t *pData,uint8_t Size);
void BL0910_SPI_Write(uint8_t REG_Addr,B32_B8_TypeDef *pData);
void BL0910_SPI_Read(uint8_t REG_Addr,B32_B8_TypeDef *pData);
void BL0910_Uart_Write(uint8_t REG_Addr,B32_B8_TypeDef *pData);
void BL0910_Uart_Read(uint8_t REG_Addr,B32_B8_TypeDef *pData);
void BL0910_SoftRst(uint8_t	Rst_Type);
void BL0910_Init(void);
void BL0910_Elect_Proc(void);
void Energy_Cal(uint32_t Cnt);

void BL0910_Write(uint8_t REG_Addr,B32_B8_TypeDef *pData);
void BL0910_Read(uint8_t REG_Addr,B32_B8_TypeDef *pData);
void BL0910_Printf_All(void);
void stateRELAY(void);
//void stateRELAY2(void);
void updateSwitch(void);
//void switch_update(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == UART4)
  {
    relay_flag = 1;  // Set flag to indicate data received
    HAL_UART_Receive_IT(&huart4, (uint8_t*)&rx_relay, 1); // receive next character

  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    relay_flag1 = 1;  // Set the flag to indicate a button press

//    if (GPIO_Pin == GPIO_PIN_3)  // Button 1
//       {
//    	 static uint8_t press_count = 0;
//
//    	        // Increment press count with each press
//    	        press_count++;
//
//    	        if (press_count == 1)
//    	        {
//    	            mode1 = 1;  // First press: turn on relay1
//    	        }
//    	        else if (press_count == 2)
//    	        {
//    	            mode1 = 0;  // Second press: turn off relay1
//    	        }
//    	        else if (press_count == 3)
//    	        {
//    	            mode1 = 1;  // Third press: turn on relay1 (or any other specific action)
//    	            press_count = 0;  // Reset the count after the third press
//    	        }
    	  //  }

     //  }
//       if (GPIO_Pin == GPIO_PIN_4)  // Button 2
//       {
//           mode2 = !mode2;  // Toggle mode2 between 1 and 0
//       }
//   //    if (GPIO_Pin == GPIO_PIN_5)  // Button 3
//   //    {
//   //        mode3 = !mode3;  // Toggle mode3 between 1 and 0
//   //    }
//       if (GPIO_Pin == GPIO_PIN_11)  // Button 4
//       {
//           mode4 = !mode4;  // Toggle mode4 between 1 and 0
//       }
//       if (GPIO_Pin == GPIO_PIN_12)  // Button 5
//       {
//           mode5 = !mode5;  // Toggle mode5 between 1 and 0
//       }
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  BL0910_Init();
  uint32_t loop_timer = HAL_GetTick();//Timer counter
  HAL_UART_Receive_IT(&huart4, (uint8_t*)&rx_relay, 1); // start receiving
  HAL_Delay(50);
  HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
  HAL_GPIO_WritePin(TRI1_GPIO_Port, TRI1_Pin, 0);
  HAL_GPIO_WritePin(TRI2_GPIO_Port, TRI2_Pin, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ( HAL_GetTick() - loop_timer > 2000 ){
	 	  loop_timer = HAL_GetTick();
	 	   BL0910_Elect_Proc();

			vol = BL0910_Elect.RMS_V * fv; //Voltage (V) Volt
			freq = BL0910_Elect.Period * ff; //Frequency Hz (Hertz)

			I1 = BL0910_Elect.RMS_1 * fi; //Current 1 (A) Ampere
			I2 = BL0910_Elect.RMS_2 * fi;
			I3 = BL0910_Elect.RMS_3 * fi;
			I4 = BL0910_Elect.RMS_4 * fi;
			I5 = BL0910_Elect.RMS_5 * fi;
			I6 = BL0910_Elect.RMS_6 * fi;
			//I6 = 1.0245;
			I7 = BL0910_Elect.RMS_7 * fi;
			I8 = BL0910_Elect.RMS_8 * fi;
			I9 = BL0910_Elect.RMS_9 * fi;
			I10 = BL0910_Elect.RMS_10 * fi;
			P1 = BL0910_Elect.WATT_1 * fp; //Active Power1 W (Watt)
			P2 = BL0910_Elect.WATT_2 * fp;
			P3 = BL0910_Elect.WATT_3 * fp;
			P4 = BL0910_Elect.WATT_4 * fp;
			P5 = BL0910_Elect.WATT_5 * fp;
			P6 = BL0910_Elect.WATT_6 * fp;
			P7 = BL0910_Elect.WATT_7 * fp;
			P8 = BL0910_Elect.WATT_8 * fp;
			P9 = BL0910_Elect.WATT_9 * fp;
			P10 = BL0910_Elect.WATT_10 * fp;

//			I6 =  BL0910_Elect.RMS_6/ BL0910_IREF*100; //Random
//			I7 =  BL0910_Elect.RMS_7/ BL0910_IREF*100;
//			I8=   BL0910_Elect.RMS_8/ BL0910_IREF*100;
//		    I9 =  BL0910_Elect.RMS_9/ BL0910_IREF*100;
//			I10 = BL0910_Elect.RMS_10/ BL0910_IREF*100;


//			P6 = vol* I6;
//			P7 = vol* I7;
//			P8 = vol* I8;
//			P9 = vol* I9;
//			P10 = vol* I10;

			BL0910_package.Current1 = I6;
			BL0910_package.Current2 = I7;
			BL0910_package.Current3 = I8;
			BL0910_package.Current4 = I9;
			BL0910_package.Current5 = I10;

			BL0910_package.AP1 = P6;
			BL0910_package.AP2 = P7;
			BL0910_package.AP3 = P8;
			BL0910_package.AP4 = P9;
			BL0910_package.AP5 = P10;


			BL0910_package.energy_value1 = P6*1/ 1000;
			BL0910_package.energy_value2 = P7*1/ 1000;
			BL0910_package.energy_value3 = P8*1/ 1000;
			BL0910_package.energy_value4 = P9*1/ 1000;
			BL0910_package.energy_value5 = P10*1/ 1000;

	 	    BL0910_package.Voltage = vol;
			BL0910_package.Freq = freq;

			HAL_UART_Transmit(&huart4, &BL0910_package, sizeof(BL0910_package),100);
			HAL_UART_Transmit(&huart2, &BL0910_package, sizeof(BL0910_package),100);
			HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin); //LED_A Indicator
	 	 	  }
	  if (relay_flag == 1)
	      {   HAL_Delay(500);
	          stateRELAY();  // Call relay control function
	          HAL_Delay(50);
	          relay_flag = 0;  // Clear flag
	      }
//	  if (GPIO_Pin == GPIO_PIN_3)  // Button 1
//	         {
//	             mode1 = !mode1;  // Toggle mode1 between 1 and 0
//
//	         }
	  //updateSwitch();
	  // Control relays based on the button pressed
//	  if (relay_flag1 == 1)
//	     {
//
//		  HAL_Delay(500);
////RL1
//			if (mode1 == 0) {
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//				HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, 1);
//			}
//			if (mode1 == 1) {
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, 0);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//				HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, 0);
//				//RL2
//			}
//			if (mode2 == 0) {
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//				HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, 1);
//			}
//			if (mode2 == 1) {
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, 0);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//				HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, 0);
//			}
//			//RL3
//			if (mode4 == 0) {
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
//			}
//			if (mode4 == 1) {
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, 0);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 0);
//			}
//			//RL4
//			if (mode5 == 0) {
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//				//HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
//			}
//			if (mode5 == 1) {
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, 0);
//				HAL_Delay(50);
//				HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//				//HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 0);
//			}
//		  HAL_Delay(50);
//	       relay_flag1 = 0;  // Reset the flag after handling the relay
//	     }



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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

/* USER CODE BEGIN 4 */
//CHECKSUM byte is((ADDR+Data_L+Data_M+Data_H)& 0xFF) and then inverted by bit (based on datasheet)
uint8_t ChkSum_Cal(uint8_t Cmd,uint8_t REG_Addr,uint8_t *pData,uint8_t Size)
{
	uint8_t tmp;
	uint8_t i;
	tmp=Cmd+ REG_Addr;
	for (i = 0; i < Size; ++i)
		{
			tmp+=pData[i];
		}
	tmp=~tmp;
	return tmp;

}
void BL0910_Uart_Write(uint8_t REG_Addr,B32_B8_TypeDef *pData)
{
	uint8_t lb_SendData[6];
	lb_SendData[0]=pData->B8[0];
	lb_SendData[1]=pData->B8[1];
	lb_SendData[2]=pData->B8[2];
	lb_SendData[5]=ChkSum_Cal(0x00,REG_Addr,lb_SendData,3);
	lb_SendData[0]=0xCA;
	lb_SendData[1]=REG_Addr;
	lb_SendData[2]=pData->B8[0];
	lb_SendData[3]=pData->B8[1];
	lb_SendData[4]=pData->B8[2];
	HAL_UART_Transmit(&huart1,lb_SendData,6,200);

}
void BL0910_Uart_Read(uint8_t REG_Addr,B32_B8_TypeDef *pData)
{
	HAL_StatusTypeDef	sta_UARTRec;
	uint8_t lb_SendData[2];
	uint8_t lb_RecData[4];
	uint8_t	lb_ChkSumCal;
	lb_SendData[0]=0x35;
	lb_SendData[1]=REG_Addr;
	HAL_UART_Transmit(&huart1,lb_SendData,2,200);
	sta_UARTRec = HAL_UART_Receive(&huart1,lb_RecData,4,400);
	if (sta_UARTRec==HAL_OK)
		{
			lb_ChkSumCal=ChkSum_Cal(0x00,REG_Addr,lb_RecData,3);
			if(lb_ChkSumCal==lb_RecData[3])
				{
					pData->B8[0]=lb_RecData[0];
					pData->B8[1]=lb_RecData[1];
					pData->B8[2]=lb_RecData[2];
					pData->B8[3]=lb_RecData[3];				//CHKSUM
					BL0910_ComSta=COM_OK;
				}
			else
				BL0910_ComSta=COM_CHKERR;
		}
	else			//Check communicate status
		{
				BL0910_ComSta=COM_ERROR;
		}
}
void BL0910_SoftRst(uint8_t	Rst_Type)
{
	BL0910_Data.B32=0x5555;
	BL0910_Uart_Write(Addr_USR_WRPROT, &BL0910_Data);
	HAL_Delay(1);
	BL0910_Uart_Read(Addr_USR_WRPROT,&BL0910_Data);
	BL0910_Data.B32=0x5A5A5A;
	BL0910_Uart_Write(Addr_SOFT_RESET, &BL0910_Data);
	if (Rst_Type==Sys_Rst)
	{
		BL0910_Data.B32=0x5A5A5A;
		BL0910_Uart_Write(Addr_SOFT_RESET, &BL0910_Data);
	}
	else
	{
		BL0910_Data.B32=0x55AA55;
		BL0910_Uart_Write(Addr_SOFT_RESET, &BL0910_Data);
	}
	HAL_Delay(1);

}
void BL0910_Init(void)
{
	BL0910_Data.B8[0]=0x55;
	BL0910_Data.B8[1]=0x55;
	BL0910_Data.B8[2]=0x00;
	BL0910_Write(Addr_USR_WRPROT, &BL0910_Data);
	HAL_Delay(1);
	BL0910_Data.B32=0x010600;
	BL0910_Write(Addr_MODE3, &BL0910_Data);

	BL0910_Data.B32=0x333330;
	BL0910_Write(Addr_GAIN1, &BL0910_Data);
	BL0910_Data.B32=0x033333;
	BL0910_Write(Addr_GAIN2, &BL0910_Data);

	BL0910_Data.B32=Current_K*8/100;
	BL0910_Write(Addr_RMS_CREEP, &BL0910_Data);
	BL0910_Data.B32 = Power_K / 10;
	BL0910_Write(Addr_VAR_WATT_CREEP, &BL0910_Data);

	BL0910_Data.B32 = 0x1FFF;
	BL0910_Write(Addr_RST_ENG, &BL0910_Data);

	BL0910_Data.B32 = 0x5A5A5A;
	BL0910_Write(Addr_SOFT_RESET, &BL0910_Data);
	HAL_Delay(2);
	// ر д
	BL0910_Data.B8[0] = 0x00;
	BL0910_Data.B8[1] = 0x00;
	BL0910_Data.B8[2] = 0x00;
	BL0910_Write(Addr_USR_WRPROT, &BL0910_Data);


}
void BL0910_Elect_Proc(void){
	B32_B8_TypeDef tmp_D;
	//Read the effective value of 10 channels of currentֵ
	BL0910_Read(Addr_1_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	//Remove the check Byte
	tmp_D.B32=tmp_D.B32*100/Current_K; //Current_K is reduced by 10 times, *100 is magnification 100 times, so the total amplification of current is 1000 times.
	BL0910_Elect.RMS_1=tmp_D.B32;
	BL0910_Read(Addr_2_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Current_K;
	BL0910_Elect.RMS_2=tmp_D.B32;
	BL0910_Read(Addr_3_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Current_K;
	BL0910_Elect.RMS_3=tmp_D.B32;
	BL0910_Read(Addr_4_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Current_K;
	BL0910_Elect.RMS_4=tmp_D.B32;
	BL0910_Read(Addr_5_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Current_K;
	BL0910_Elect.RMS_5=tmp_D.B32;
	BL0910_Read(Addr_6_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Current_K;
	BL0910_Elect.RMS_6=tmp_D.B32;
	BL0910_Read(Addr_7_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Current_K;
	BL0910_Elect.RMS_7=tmp_D.B32;
	BL0910_Read(Addr_8_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Current_K;
	BL0910_Elect.RMS_8=tmp_D.B32;
	BL0910_Read(Addr_9_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Current_K;
	BL0910_Elect.RMS_9=tmp_D.B32;
	BL0910_Read(Addr_10_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Current_K;
	BL0910_Elect.RMS_10=tmp_D.B32;

	//Read the effective value of voltage ֵ
	BL0910_Read(Addr_V_RMS, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=tmp_D.B32*100/Voltage_k;
	BL0910_Elect.RMS_V=tmp_D.B32;

	//Read 10 active power
	BL0910_Read(Addr_WATT1, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607) //Determine whether the power is reversed and do the supplementary code conversion.
		{
			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	//Power_K is magnified by 10 times, *100 is magnified by 100 times, so the total power magnification is 10 times.
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_1=tmp_D.B32;
	BL0910_Read(Addr_WATT2, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607)
		{
			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_2=tmp_D.B32;
	BL0910_Read(Addr_WATT3, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607)
		{
			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_3=tmp_D.B32;
	BL0910_Read(Addr_WATT4, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607)
		{
			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_4=tmp_D.B32;
	BL0910_Read(Addr_WATT5, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607)
		{
			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_5=tmp_D.B32;
	BL0910_Read(Addr_WATT6, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607)
		{
			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_6=tmp_D.B32;
	BL0910_Read(Addr_WATT7, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607)
		{
			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_7=tmp_D.B32;
	BL0910_Read(Addr_WATT8, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607)
		{


			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_8=tmp_D.B32;
	BL0910_Read(Addr_WATT9, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607)
		{
			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_9=tmp_D.B32;
	BL0910_Read(Addr_WATT10, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	if (tmp_D.B32>=8388607)
		{
			tmp_D.B32=0x1000000-tmp_D.B32;
		}
	tmp_D.B32=tmp_D.B32*100/Power_K;
	BL0910_Elect.WATT_10=tmp_D.B32;
	//Reading frequency
	BL0910_Read(Addr_PERIOD, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32=10000000*100/tmp_D.B32;
	BL0910_Elect.Period=tmp_D.B32;
	// Read 10 active pulse counts to calculate the accumulation of electrical energy.
	BL0910_Read(Addr_CF1_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt1;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_1+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt1=Eng_Cal.cnt_remainder;
	BL0910_Read(Addr_CF2_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt2;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_2+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt2=Eng_Cal.cnt_remainder;
	BL0910_Read(Addr_CF3_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt3;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_3+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt3=Eng_Cal.cnt_remainder;
	BL0910_Read(Addr_CF4_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt4;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_4+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt4=Eng_Cal.cnt_remainder;
	BL0910_Read(Addr_CF5_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt5;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_5+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt5=Eng_Cal.cnt_remainder;
	BL0910_Read(Addr_CF6_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt6;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_6+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt6=Eng_Cal.cnt_remainder;
	BL0910_Read(Addr_CF7_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt7;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_7+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt7=Eng_Cal.cnt_remainder;
	BL0910_Read(Addr_CF8_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt8;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_8+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt8=Eng_Cal.cnt_remainder;
	BL0910_Read(Addr_CF9_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt9;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_9+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt9=Eng_Cal.cnt_remainder;
	BL0910_Read(Addr_CF10_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt10;
	Energy_Cal(tmp_D.B32);
	BL0910_Elect.Energy_10+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt10=Eng_Cal.cnt_remainder;
	// ܵ
	BL0910_Read(Addr_CF_CNT, &BL0910_Data);
	tmp_D.B32=BL0910_Data.B32;
	tmp_D.B8[3]=0;
	tmp_D.B32+=Eng_CFCnt.Cnt_Sum;
	if(tmp_D.B32>=Energy_Sum_K)
		{
			Eng_Cal.Energy001=tmp_D.B32/Energy_Sum_K;
			Eng_Cal.cnt_remainder=tmp_D.B32%Energy_Sum_K;
		}
	else
		{
			Eng_Cal.Energy001=0;
			Eng_Cal.cnt_remainder=tmp_D.B32;
		}
	BL0910_Elect.Energy_Sum+=Eng_Cal.Energy001;
	Eng_CFCnt.Cnt_Sum=Eng_Cal.cnt_remainder;

}
void Energy_Cal(uint32_t Cnt){
	if (Cnt>=Energy_K)
		{
			Eng_Cal.Energy001 = Cnt / Energy_K;
			Eng_Cal.cnt_remainder = Cnt % Energy_K;
		}
	else
		{
			Eng_Cal.Energy001 = 0;
			Eng_Cal.cnt_remainder = Cnt;
		}
}
void BL0910_Write(uint8_t REG_Addr,B32_B8_TypeDef *pData){

	BL0910_Uart_Write(REG_Addr , pData);
}
void BL0910_Read(uint8_t REG_Addr,B32_B8_TypeDef *pData){

	BL0910_Uart_Read(REG_Addr , pData);
}
/*void updateSwitch(void){
	if (mode1 == 0) {
		rx_relay1 = 'A';
				}
	if (mode1 == 1) {
	    rx_relay1 = 'B';
					//RL2
				}
	if (mode2 == 0) {;
		rx_relay1 = 'C';
	}
	if (mode2 == 1) {
		 rx_relay1 = 'D';
	}
	//RL3
	if (mode4 == 0) {
		 rx_relay1 = 'E';
	}
	if (mode4 == 1) {
		 rx_relay1 = 'F';
	}
	//RL4
	if (mode5 == 0) {
		 rx_relay1 = 'H';
	}
	if (mode5 == 1) {
		 rx_relay1 = 'I';
	}
}*/

void stateRELAY(void){
	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
	//HAL_Delay(50);
	// RL 1
	if ( rx_relay == 'A' ){
		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
			HAL_Delay(50);
		 	HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, 1);
		 	HAL_Delay(50);
		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
		    HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin,1);
	}
	if ( rx_relay == 'B' ){
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
				HAL_Delay(50);
			 	HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, 0);
			 	HAL_Delay(50);
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
		        HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin,0);
    }

	// RL 2
	if ( rx_relay == 'C' ){
		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
			HAL_Delay(50);
		 	HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, 1);
		 	HAL_Delay(50);
		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
		    HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin,1);
	}
	if ( rx_relay == 'D' ){
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
				HAL_Delay(50);
			 	HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, 0);
			 	HAL_Delay(50);
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
		        HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin,0);
    }
	// RL 3
	if ( rx_relay == 'E' ){
		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
			HAL_Delay(50);
		 	HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, 1);
		 	HAL_Delay(50);
		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
		    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,1);
	}
	if ( rx_relay == 'F' ){
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
				HAL_Delay(50);
			 	HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, 0);
			 	HAL_Delay(50);
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
		        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,0);
    }
	// RL 4
	if ( rx_relay == 'G' ){
		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
			HAL_Delay(50);
		 	HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, 1);
		 	HAL_Delay(50);
		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);

	}
	if ( rx_relay == 'H' ){
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
				HAL_Delay(50);
			 	HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, 0);
			 	HAL_Delay(50);
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);

    }
	// RL 5
		if ( rx_relay == 'I' ){
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
				HAL_Delay(50);
			 	HAL_GPIO_WritePin(RL5_GPIO_Port, RL5_Pin, 1);
			 	HAL_Delay(50);
			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);

		}
		if ( rx_relay == 'J' ){
				 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
					HAL_Delay(50);
				 	HAL_GPIO_WritePin(RL5_GPIO_Port, RL5_Pin, 0);
				 	HAL_Delay(50);
				 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);

	    }
}
//void stateRELAY(void){
//	//HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//	//HAL_Delay(50);
//	// RL 1
//	if ( rx_relay == 'A' || toggle_BT[0] == 1 ){
//		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//			HAL_Delay(50);
//		 	HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, 1);
//		 	HAL_Delay(50);
//		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//		    HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin,1);
//	}
//	if ( rx_relay == 'B' || toggle_BT[0] == 0){
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//			 	HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, 0);
//			 	HAL_Delay(50);
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//		        HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin,0);
//    }
//
//	// RL 2
//	if ( rx_relay == 'C' || toggle_BT[1] == 1){
//		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//			HAL_Delay(50);
//		 	HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, 1);
//		 	HAL_Delay(50);
//		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//		    HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin,1);
//	}
//	if ( rx_relay == 'D' || toggle_BT[1] == 0){
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//			 	HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, 0);
//			 	HAL_Delay(50);
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//		        HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin,0);
//    }
//	// RL 3
//	if ( rx_relay == 'E' || toggle_BT[2] == 1){
//		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//			HAL_Delay(50);
//		 	HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, 1);
//		 	HAL_Delay(50);
//		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//		    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,1);
//	}
//	if ( rx_relay == 'F' || toggle_BT[2] == 0){
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//			 	HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, 0);
//			 	HAL_Delay(50);
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//		        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,0);
//    }
//	// RL 4
//	if ( rx_relay == 'G'|| toggle_BT[3] == 1 ){
//		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//			HAL_Delay(50);
//		 	HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, 1);
//		 	HAL_Delay(50);
//		 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//
//	}
//	if ( rx_relay == 'H'|| toggle_BT[3] == 0 ){
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//			 	HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, 0);
//			 	HAL_Delay(50);
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//
//    }
//	// RL 5
//		if ( rx_relay == 'I' || toggle_BT[4] == 1){
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//				HAL_Delay(50);
//			 	HAL_GPIO_WritePin(RL5_GPIO_Port, RL5_Pin, 1);
//			 	HAL_Delay(50);
//			 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//
//		}
//		if ( rx_relay == 'J' || toggle_BT[4] == 0){
//				 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 1);
//					HAL_Delay(50);
//				 	HAL_GPIO_WritePin(RL5_GPIO_Port, RL5_Pin, 0);
//				 	HAL_Delay(50);
//				 	HAL_GPIO_WritePin(CON_CoilRL_GPIO_Port, CON_CoilRL_Pin, 0);
//
//	    }
//}
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
