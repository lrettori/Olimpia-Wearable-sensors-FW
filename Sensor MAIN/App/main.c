/** Sensor_MAIN
  *****************************************************************************
  * @file    main.c
  * @author  Lorenzo Rettori
  * @version v2.0
  * @date    2024
  * @brief   This formware provides the firmware to collect inertial data from
  *          a LSM9DS1 device, then send the data to a Bluetooth module using
  *          a serial port.
  *          This application is aimed to program Wrist and Foot sensor modules
  *          of the OLIMPIA sensor network and it uses the following electronic
  *          components:
  *             - STM32F10x board.
  *             - LSM9DS1 inertial measurement unit
  *
  *          Main modifications from version 1.0:
  *          - 

  *****************************************************************************
  * THE PRESENT FIRMWARE AIMS AT THE MANAGEMENT AND COORDINATION OF THE WIRELESS
  * SENSOR NETWORK OF HANDi GLOVE DEVICE. DIFFERENT WORKING MODES ARE 
  * IMPLEMNETED TO ACQUIRE SENSOR DATA. A CALIBRATION PROCEDURE IS PROVIDED AND 
  * A KALAMAN FILTER LIBRARY IS IMPLEMENTED TO EXTRAPOLATE 3D SPACE HAND 
  * POSITION. A VIRTUAL COM ROUTINE IS PROVIDED AT THE START-UP FO THE DEVICE 
  * FOR THE SERIAL PROGRAMMING OF THE BLUETOOTH MODULE BY MEANS AT2 COMMANDS 
  * (REFER TO UM1547 USER MANUAL)
  ****************************************************************************/

// ____________________________________________________________________________
// Includes -------------------------------------------------------------------
#include <assert.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include "string.h"

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "arm_comm.h"

// Timer and interrupts handlers
#include "stm32f10x_it.h"
#include "misc.h"

// Sensor : the adopted sensor includes a 3D accelerometer, a 3D gyroscope and 
// a 3D magnetometer. The magnetometer is not employed for this specific
// application.
#include "LSM9DS1.h"

// DEVICE ---------------------------------------------------------------------
// This application can be used to program both SensHand main board and SensFoot
// devices. Nothing needs to be changed excluding DEVICE_ADDRESS that has to be
// set as indicated:
// - Left SensHand  : 0
// - Right SensHand : 1
// - Left SensFoot  : 2
// - Right SensFoot : 3
//
//uint8_t DEVICE_ADDRESS = (uint8_t) '0';
//uint8_t DEVICE_ADDRESS = (uint8_t) '1';
//uint8_t DEVICE_ADDRESS = (uint8_t) '2';
uint8_t DEVICE_ADDRESS = (uint8_t) '3';

// data_len is initialized for SensHand devices, that needs longer arrays. Its
// value is changed in main() if DEVICE_ADDRESS is 2 or 3, so if the application
// is for a SensFoot device.
int data_len = 36;
volatile uint8_t FINGERS = 1;

//static uint16_t counter = 0; /////

// Private variables ----------------------------------------------------------
// Sampling frequency is needed to determine how often the sensor will produce a
// reading to be sent. Sampling frequency is set as ~111 Hz, the equivalent of a
// sampling period of 9ms.
int Fs = 1/(9*10^-3);                           // Sampling frequency [Hz]
int TIM_LED = 0;

//
volatile uint8_t flag     = 1;
volatile uint8_t flag_can = 1;
uint16_t REC;

// Sensor variables
LSM_DATA m_data;

// LED ------------------------------------------------------------------------
#define LED_PIN                         GPIO_Pin_1
#define LED_PORT                        GPIOB
#define LED_RCC_PERIPH                  RCC_APB2Periph_GPIOB
#define LED_MODE                        GPIO_Mode_Out_PP
#define LED_SPEED                       GPIO_Speed_50MHz

// USART BLE Module COMM ------------------------------------------------------
// Sensor data are sent from Micro to BLE Module (BMD-300) using serial 
// communiation through USART2. Associated parameters are listed below.
#define BLE_USART                       USART2
#define BLE_PORT                        GPIOA

#define BLE_RCC_PERIPH                  RCC_APB1Periph_USART2
#define BLE_RCC_PORT                    RCC_APB2Periph_GPIOA

// Pins
#define BLE_CTS_PIN                     GPIO_Pin_0
#define BLE_RTS_PIN                     GPIO_Pin_1
#define BLE_TX_PIN                      GPIO_Pin_2
#define BLE_RX_PIN                      GPIO_Pin_3

// Mode
#define BLE_RX_MODE                     GPIO_Mode_IN_FLOATING
#define BLE_TX_MODE                     GPIO_Mode_AF_PP
#define BLE_CTS_MODE                    GPIO_Mode_AF_PP
#define BLE_RTS_MODE                    GPIO_Mode_AF_PP

#define BLE_GPIO_SPEED                  GPIO_Speed_50MHz

// SENSOR variables -----------------------------------------------------------
// Useful to start sensor after some cycles, so that it is properly initialized.
u8 START_UP = 1;
int count_start_up = 0;

// Sensor data to be sent
uint8_t vec[38];
//uint8_t vec[36];
//uint8_t vec[37];

// CAN Bus variables ----------------------------------------------------------
// CAN module will be necessary for SensHand device as it handles communitation
// between the main board and the fingers. Two fingers whould be involved in 
// that case, namely Thumb and Index finger.
CanTxMsg TxMessage;
CanRxMsg RxMessage;

#define FIN_CAN                         CAN1
#define CAN_PORT                        GPIOB
#define CAN_RCC_PORT                    RCC_APB2Periph_GPIOB

// Pins
#define CAN_RX_PIN                      GPIO_Pin_8
#define CAN_TX_PIN                      GPIO_Pin_9

// Mode
#define CAN_RX_MODE                     GPIO_Mode_IPU
#define CAN_TX_MODE                     GPIO_Mode_AF_PP

#define CAN_GPIO_SPEED                  GPIO_Speed_50MHz

#define COR_CAN_ID                      0x0011  
#define FIN1_CAN_ID                     0x0012                  // Thumb    
#define FIN2_CAN_ID                     0x0013                  // Index finger

#define OFFSET                          6

u8 count_finger    = 0;                 // [CHECK]
u8 f1_data[18];
u8 f2_data[18];

// ____________________________________________________________________________
// ____________________________________________________________________________
#define RCC_APB1Periph_ALL (RCC_APB1Periph_TIM2| RCC_APB1Periph_TIM3|\
        RCC_APB1Periph_TIM4| RCC_APB1Periph_TIM5| RCC_APB1Periph_TIM6|\
        RCC_APB1Periph_TIM7| RCC_APB1Periph_WWDG| RCC_APB1Periph_SPI2|\
        RCC_APB1Periph_SPI3| RCC_APB1Periph_USART2| RCC_APB1Periph_USART3|\
        RCC_APB1Periph_I2C1| RCC_APB1Periph_I2C2| RCC_APB1Periph_USB|\
        RCC_APB1Periph_CAN1| RCC_APB1Periph_BKP| RCC_APB1Periph_PWR)

#define RCC_APB2Periph_ALL (RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB |\
        RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOD| RCC_APB2Periph_AFIO|\
        RCC_APB2Periph_ADC2| RCC_APB2Periph_TIM1| RCC_APB2Periph_SPI1|\
        RCC_APB2Periph_TIM8|RCC_APB2Periph_USART1|RCC_APB2Periph_ADC3)

#define RCC_AHBPeriph_ALL (RCC_AHBPeriph_DMA1|RCC_AHBPeriph_DMA2|\
        RCC_AHBPeriph_SRAM|RCC_AHBPeriph_FLITF|RCC_AHBPeriph_CRC)

#ifdef __cplusplus
 extern "C" {
#endif
ErrorStatus HSEStartUpStatus;

// ____________________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________
void RCC_Config(void)             
{
  RCC_LSICmd(DISABLE);
  RCC_PLLCmd(DISABLE);
  RCC_HSEConfig(RCC_HSE_OFF); 
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);                                                    
  HSEStartUpStatus = RCC_WaitForHSEStartUp();	                                
  if(HSEStartUpStatus == SUCCESS)  
  {
    FLASH_SetLatency(FLASH_Latency_2);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_3); /*12MHz*/ 
    RCC_PLLCmd(ENABLE);
    
    // while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) { }
    
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK );
    // while(RCC_GetSYSCLKSource() != 0x08) {}
    
    RCC_HCLKConfig(RCC_SYSCLK_Div1);    /*12Mhz*/	                                                        
    RCC_PCLK2Config(RCC_HCLK_Div1);     /*APB2:12MHz*/		                                
    RCC_PCLK1Config(RCC_HCLK_Div1);     /*APB1:12MHz*/
  }
}

// ____________________________________________________________________________
void RCC_CLK_DISABLE()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_ALL, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALL, DISABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ALL, DISABLE);
}

// ____________________________________________________________________________
// Pc = ((Prescaler + 1)(Period + 1))/TIM_CLOCK
// Prescaler and Period are uint16_t so they have to be below 65536
//
// Our specific TIM_CLOCK = 72 MHz            
void TIM_Config(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  TIM_TimeBaseInitTypeDef TIM2_TimeBaseInitStruct; 
  //
  TIM2_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;
  TIM2_TimeBaseInitStruct.TIM_Prescaler         = (3600 - 1)/Fs;
  TIM2_TimeBaseInitStruct.TIM_Period            = (2000 - 1);
  TIM2_TimeBaseInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
  TIM2_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2,&TIM2_TimeBaseInitStruct);  
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
}

// ____________________________________________________________________________
// UART serial port needs to be configured in order to send data to the BLE
// module.
void UART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStr;
  RCC_APB2PeriphClockCmd(BLE_RCC_PORT, ENABLE);
                               
  GPIO_InitStr.GPIO_Pin   = BLE_TX_PIN;			
  GPIO_InitStr.GPIO_Speed = BLE_GPIO_SPEED;
  GPIO_InitStr.GPIO_Mode  = BLE_TX_MODE;
  GPIO_Init(BLE_PORT, &GPIO_InitStr);
  
  GPIO_InitStr.GPIO_Pin   = BLE_RX_PIN;	                                	
  GPIO_InitStr.GPIO_Speed = BLE_GPIO_SPEED;
  GPIO_InitStr.GPIO_Mode  = BLE_RX_MODE;
  GPIO_Init(BLE_PORT, &GPIO_InitStr);
  
  GPIO_InitStr.GPIO_Speed = BLE_GPIO_SPEED;
  GPIO_InitStr.GPIO_Pin   = BLE_CTS_PIN;
  GPIO_InitStr.GPIO_Mode  = BLE_CTS_MODE;
  GPIO_Init(BLE_PORT, &GPIO_InitStr);
  
  GPIO_InitStr.GPIO_Pin   = BLE_RTS_PIN;
  GPIO_InitStr.GPIO_Mode  = BLE_RTS_MODE;
  GPIO_Init(BLE_PORT, &GPIO_InitStr);
  //
  USART_InitTypeDef USART_InitStr;
  RCC_APB1PeriphClockCmd(BLE_RCC_PERIPH, ENABLE);    

  USART_Cmd(BLE_USART, DISABLE);
  USART_DeInit(BLE_USART);
//  USART_InitStr.USART_BaudRate            = 460800;
  USART_InitStr.USART_BaudRate            = 38400;
//  USART_InitStr.USART_BaudRate            = 19200;
//  USART_InitStr.USART_BaudRate            = 9600;
  USART_InitStr.USART_WordLength          = USART_WordLength_8b;                       
  USART_InitStr.USART_StopBits            = USART_StopBits_1;                            
  USART_InitStr.USART_Parity              = USART_Parity_No;                  
  USART_InitStr.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx; 
  USART_InitStr.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
//  USART_InitStr.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(BLE_USART, &USART_InitStr);
  USART_Cmd(BLE_USART, ENABLE);
  USART_ITConfig(BLE_USART,USART_IT_RXNE, ENABLE);
}

// ____________________________________________________________________________
// CAN BUS configuration. CAN BUS communication is used only for SensHand 
// devices as it is needed to send data from fingers to the main board, then
// they are sent to the BLE module.
void CAN_Config(void)
{
  // GPIO Init ----------------------------------------------------------------
  RCC_APB2PeriphClockCmd(CAN_RCC_PORT, ENABLE);                         
  GPIO_InitTypeDef       GPIO_InitStruct;
  
  GPIO_InitStruct.GPIO_Pin   = CAN_RX_PIN;              // CAN1 RX Pin
  GPIO_InitStruct.GPIO_Mode  = CAN_RX_MODE;
  GPIO_Init(CAN_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.GPIO_Pin   = CAN_TX_PIN;              // CAN1 TX Pin
  GPIO_InitStruct.GPIO_Mode  = CAN_TX_MODE;
  GPIO_InitStruct.GPIO_Speed = CAN_GPIO_SPEED;
  GPIO_Init(CAN_PORT, &GPIO_InitStruct);
  
  // CAN Init -----------------------------------------------------------------
  CAN_InitTypeDef               CAN_InitStruct;
  CAN_FilterInitTypeDef         CAN_FiltInitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  
  CAN_DeInit(FIN_CAN);
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);
  
  // Configure CAN RX and TX Pins
  CAN_InitStruct.CAN_TTCM = DISABLE;  
  CAN_InitStruct.CAN_ABOM = DISABLE; 
  CAN_InitStruct.CAN_AWUM = DISABLE;
  CAN_InitStruct.CAN_NART = DISABLE;
  CAN_InitStruct.CAN_RFLM = DISABLE; 
  CAN_InitStruct.CAN_TXFP = ENABLE;
  CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;  
  
  // EVAL from RCC_PCLK1Config
  CAN_InitStruct.CAN_SJW = CAN_SJW_1tq; 
  CAN_InitStruct.CAN_BS1 = CAN_BS1_2tq;
  CAN_InitStruct.CAN_BS2 = CAN_BS2_3tq;
  CAN_InitStruct.CAN_Prescaler = 2;                             // 1MHz

  CAN_Init(FIN_CAN, &CAN_InitStruct);
  CAN_FiltInitStruct.CAN_FilterNumber         = 2;
  CAN_FiltInitStruct.CAN_FilterMode           = CAN_FilterMode_IdList;
  CAN_FiltInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
  CAN_FiltInitStruct.CAN_FilterIdHigh         = COR_CAN_ID<<5;
  CAN_FiltInitStruct.CAN_FilterIdLow          = 0x0000;
  CAN_FiltInitStruct.CAN_FilterMaskIdHigh     = 0x0000;         //0x011
  CAN_FiltInitStruct.CAN_FilterMaskIdLow      = 0x0000;
  CAN_FiltInitStruct.CAN_FilterFIFOAssignment = CAN_FIFO1;
  CAN_FiltInitStruct.CAN_FilterActivation     = ENABLE;
  CAN_FilterInit(&CAN_FiltInitStruct);

  // Transmit structure preparation
  TxMessage.StdId = 0x321;                                                      // standard identifier
  TxMessage.ExtId = 0x01;                                                       // extended identifier
  TxMessage.RTR   = CAN_RTR_DATA;                                               // configure the type of frame for the transmitted message
  TxMessage.IDE   = CAN_ID_STD;                                                 // configure the type of the identifier for the message that will be transmitted
  TxMessage.DLC   = 8;                                                          // lenght of the frame that will be transmitted
  
  RxMessage.StdId   = 0x00;
  RxMessage.ExtId   = 0x00;
  RxMessage.IDE     = CAN_ID_STD;
  RxMessage.DLC     = 0;
  RxMessage.FMI     = 0;
  RxMessage.Data[0] = 0x00;
  
  // Enable interrupts
  CAN_ITConfig(FIN_CAN, CAN_IT_FMP1, ENABLE);
}

// ____________________________________________________________________________
// Timer and UART interrupts
// NVIC configuration is necessary to set interrupts and event handlers for 
// TIMER, USART and CAN, which are necessary to properly handle data streaming.
void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Set the Vector Table base location at 0x08003000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  
  NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

// ____________________________________________________________________________
// Sensor initialization. It requires the initialization of the I2C module as 
// as te corresponding GPIO. The all the information of the sensors -acc, and 
// gyr- are defined. The magnetometer, even if available, is not employed for 
// this application, so it is not initialized.
// This functions sets sensor specifics in terms of sensitivity (Sens), 
// output data rate (ODR) and measurement range (FS). More precisely:
// 
//      - Accelerometer : readings are in g.
//              - Sensitivity (Sens)     = 0.244 mg/LSB
//              - Output data rate (ODR) = 952 Hz (normal mode)
//              - Measurement range (FS) = ± 8 g
//      - Gyroscope : readings are in dps.
//              - Sensitivity (Sens)     = 70 mdps/LSB
//              - Output data rate (ODR) = 952 Hz (normal mode)
//              - Measurement range (FS) = ± 2000 dps
void Sensor_Init(void)
{
  // I2C ----------------------------------------------------------------------
  I2C_InitTypeDef  I2C_InitStr;
  GPIO_InitTypeDef GPIO_InitStr;
                                                                              
  RCC_APB1PeriphClockCmd(LSM_I2C_RCC_Periph, ENABLE);                           
  RCC_APB2PeriphClockCmd(LSM_I2C_RCC_Port, ENABLE);
  
  GPIO_InitStr.GPIO_Pin   =  LSM_I2C_SCL_Pin | LSM_I2C_SDA_Pin;             
  GPIO_InitStr.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStr.GPIO_Mode  = GPIO_Mode_AF_OD;
  GPIO_Init(LSM_I2C_Port, &GPIO_InitStr);
  
  I2C_DeInit(LSM_I2C);
  I2C_InitStr.I2C_Mode                = I2C_Mode_I2C;                                    
  I2C_InitStr.I2C_DutyCycle           = I2C_DutyCycle_2;
  I2C_InitStr.I2C_OwnAddress1         = 0x00;
  I2C_InitStr.I2C_Ack                 = I2C_Ack_Enable;
  I2C_InitStr.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStr.I2C_ClockSpeed          = LSM_I2C_Speed;   
  I2C_Init(LSM_I2C, &I2C_InitStr);                                       
  I2C_Cmd(LSM_I2C, ENABLE);                                                    

  // Acc and gyr init ---------------------------------------------------------
  LSM9DS1_AG_ConfigTypeDef  LSM_Acc_Gyr_InitStr;
  
  LSM9DS1_A_ConfigTypeDef   LSM_Acc_InitStr;
  LSM9DS1_G_ConfigTypeDef   LSM_Gyr_InitStr;
  
  LSM_Acc_InitStr.Axes_Enable = LSM9DS1_A_XYZEN;
  LSM_Gyr_InitStr.Axes_Enable = LSM9DS1_G_XYZEN;
  
  LSM_Acc_InitStr.FS  = LSM9DS1_A_FS_8;
  LSM_Gyr_InitStr.FS  = LSM9DS1_G_FS_2000;
  LSM_Acc_InitStr.ODR = LSM9DS1_A_ODR_952;
  LSM_Gyr_InitStr.ODR = LSM9DS1_G_ODR_952;
  
  LSM_Acc_InitStr.Sens = LSM9DS1_Acc_Sens_8;
  LSM_Gyr_InitStr.Sens = LSM9DS1_Gyr_Sens_2000;  
  LSM_Acc_InitStr.HR   = LSM9DS1_A_HR_ENABLE;
  
  LSM9DS1_A_Config(&LSM_Acc_InitStr); 
  LSM9DS1_G_Config(&LSM_Gyr_InitStr);
  
  LSM_Acc_Gyr_InitStr.Address_Inc  = LSM9DS1_AG_ADD_INC_ENABLE;
  LSM_Acc_Gyr_InitStr.Data_Update  = LSM9DS1_AG_BDU_Continuos;
  LSM_Acc_Gyr_InitStr.Endianess    = LSM9DS1_AG_Little_Endian;
  LSM9DS1_AG_Config(&LSM_Acc_Gyr_InitStr);
}

// ____________________________________________________________________________
void LED_Config(void)
{
  RCC_APB2PeriphClockCmd(LED_RCC_PERIPH, ENABLE);
  GPIO_InitTypeDef       GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin   = LED_PIN;
  GPIO_InitStructure.GPIO_Mode  = LED_MODE;
  GPIO_InitStructure.GPIO_Speed = LED_SPEED;
  GPIO_Init(LED_PORT, &GPIO_InitStructure);
  
  GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
}

// ____________________________________________________________________________
void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  GPIOx->ODR ^= GPIO_Pin;
}

// ____________________________________________________________________________
// Function that reads data from IMU sensors. Data are stored in a vector and
// sent through a serial port (USART2).
// This function is called in the timer_handler so there is a reading available
// at every timer tick (namely, 9ms). <- [NOT REALLY]
void Read_SensorData(void) {
  // Sensor Data --------------------------------------------------------------
  LSM_DataProcess(&m_data);
  
//  // Sensor 1 : IMU on the main board. It is the only one present on SensFoot
//  // devices and it is the wrist sensor for SensHand devices.
//  vec[0]  = (uint8_t)(((int16_t)m_data.sAcc[0]));
//  vec[1]  = (uint8_t)(((int16_t)m_data.sAcc[0])>>8);
//  vec[2]  = (uint8_t)(((int16_t)m_data.sAcc[1]));
//  vec[3]  = (uint8_t)(((int16_t)m_data.sAcc[1])>>8);
//  vec[4]  = (uint8_t)(((int16_t)m_data.sAcc[2]));
//  vec[5]  = (uint8_t)(((int16_t)m_data.sAcc[2])>>8);
//  vec[6]  = (uint8_t)(((int16_t)m_data.sGyr[0]));
//  vec[7]  = (uint8_t)(((int16_t)m_data.sGyr[0])>>8);
//  vec[8]  = (uint8_t)(((int16_t)m_data.sGyr[1]));
//  vec[9]  = (uint8_t)(((int16_t)m_data.sGyr[1])>>8);
//  vec[10] = (uint8_t)(((int16_t)m_data.sGyr[2]));
//  vec[11] = (uint8_t)(((int16_t)m_data.sGyr[2])>>8);
//  
//  // Sensor 2 : available only for SensHand devices, it is the IMU on the Thumb.
//  vec[12] = f1_data[0];
//  vec[13] = f1_data[1];
//  vec[14] = f1_data[2];
//  vec[15] = f1_data[3];
//  vec[16] = f1_data[4];
//  vec[17] = f1_data[5];
//  vec[18] = f1_data[6];
//  vec[19] = f1_data[7];
//  vec[20] = f1_data[8];
//  vec[21] = f1_data[9];
//  vec[22] = f1_data[10];
//  vec[23] = f1_data[11];
//  
//  // Sensor 3 : available only for SensHand devices, it is the IMU on the Index
//  // finger. 
//  vec[24] = f2_data[0];
//  vec[25] = f2_data[1];
//  vec[26] = f2_data[2];
//  vec[27] = f2_data[3];
//  vec[28] = f2_data[4];
//  vec[29] = f2_data[5];
//  vec[30] = f2_data[6];
//  vec[31] = f2_data[7];
//  vec[32] = f2_data[8];
//  vec[33] = f2_data[9];
//  vec[34] = f2_data[10];
//  vec[35] = f2_data[11];
  
//    // Versione mista contatore/costante
////  for (uint8_t i = 0; i < (data_len / 2); i++) 
//  for (uint8_t i = 0; i < (6); i++) // Versione con rampe solo nei sensori polsi, e il resto dei dati li prelevo dalle dita
//  {
//    if ((i-1) % 3 == 0) 
//    {
//      //      vec[2 * i] = (uint8_t)(counter & 0x00FF);
//      //      vec[2 * i + 1] = (uint8_t)((counter & 0xFF00) >> 8);
//      // Versione con start e stop byte
//      vec[2 * i + 1] = (uint8_t)(counter & 0x00FF);
//      vec[2 * i + 2] = (uint8_t)((counter & 0xFF00) >> 8);
//    }
//    else
//    {
//      //      vec[2 * i] = i * 10;
//      //      vec[2 * i + 1] = i * 10;
//      // Versione con start e stop byte
//      vec[2 * i + 1] = i * 10;
//      vec[2 * i + 2] = i * 10;
//    }
//  }
  
  // Versione con start e stop bytes
  vec[0] = (uint8_t) 'D';
  // Sensor 1 : IMU on the main board. It is the only one present on SensFoot
  // devices and it is the wrist sensor for SensHand devices.
  vec[1]  = (uint8_t)(((int16_t)m_data.sAcc[0]));
  vec[2]  = (uint8_t)(((int16_t)m_data.sAcc[0])>>8);
  vec[3]  = (uint8_t)(((int16_t)m_data.sAcc[1]));
  vec[4]  = (uint8_t)(((int16_t)m_data.sAcc[1])>>8);
  vec[5]  = (uint8_t)(((int16_t)m_data.sAcc[2]));
  vec[6]  = (uint8_t)(((int16_t)m_data.sAcc[2])>>8);
  vec[7]  = (uint8_t)(((int16_t)m_data.sGyr[0]));
  vec[8]  = (uint8_t)(((int16_t)m_data.sGyr[0])>>8);
  vec[9]  = (uint8_t)(((int16_t)m_data.sGyr[1]));
  vec[10]  = (uint8_t)(((int16_t)m_data.sGyr[1])>>8);
  vec[11] = (uint8_t)(((int16_t)m_data.sGyr[2]));
  vec[12] = (uint8_t)(((int16_t)m_data.sGyr[2])>>8);
  
  // Sensor 2 : available only for SensHand devices, it is the IMU on the Thumb.
  vec[13] = f1_data[0];
  vec[14] = f1_data[1];
  vec[15] = f1_data[2];
  vec[16] = f1_data[3];
  vec[17] = f1_data[4];
  vec[18] = f1_data[5];
  vec[19] = f1_data[6];
  vec[20] = f1_data[7];
  vec[21] = f1_data[8];
  vec[22] = f1_data[9];
  vec[23] = f1_data[10];
  vec[24] = f1_data[11];
  
  // Sensor 3 : available only for SensHand devices, it is the IMU on the Index
  // finger. 
  vec[25] = f2_data[0];
  vec[26] = f2_data[1];
  vec[27] = f2_data[2];
  vec[28] = f2_data[3];
  vec[29] = f2_data[4];
  vec[30] = f2_data[5];
  vec[31] = f2_data[6];
  vec[32] = f2_data[7];
  vec[33] = f2_data[8];
  vec[34] = f2_data[9];
  vec[35] = f2_data[10];
  vec[36] = f2_data[11];

  vec[data_len + 1] = (uint8_t) 'L';
  
//  if (counter < 65535)
//counter++;
//else
//counter = 0;    
  
  // --------------------------------------------------------------------------
  // Send to serial port : data are then sent through the UART to the Bluetooth
  // module. The while loop assures that the trasmission is complete before 
  // going on with other tasks. Only useful information is sent, so if we are 
  // using SensFoot devices (1 IMU), data_len = 12 and only the first 12
  // elements of vec are sent.
  // If the trasmission is complete, "flag" is toggled, so the whole procedure
  // can go on.
//  for(int i = 0; i < data_len; i++) {
for(int i = 0; i < data_len + 2; i++) { // versione con start e stop byte
//    for(int i = 0; i < data_len + 1; i++) {
    USART_SendData(USART2 , vec[i]);
    while(USART_GetFlagStatus(USART2 , USART_FLAG_TXE) == RESET);
  }
  
  if (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != RESET) {
    flag = 1;
  }
  
  // While the Micro is streaming, the Blue LED blinks with 1 HZ frequency.
  TIM_LED++;
  if (TIM_LED == 50) {
    GPIO_TogglePin(LED_PORT, LED_PIN);
    TIM_LED = 0;
  }
}

// ____________________________________________________________________________
// CAN Transmit
void TX_CanBus(uint8_t ADDR, uint32_t ID)
{
  u8 MAIL;
  // Prepare CAN transmission to Thumb. 
  TxMessage.StdId   = ID;
  TxMessage.Data[0] = ADDR;
  TxMessage.DLC     = 1;
  
  MAIL = CAN_Transmit(FIN_CAN, &TxMessage);
  while (CAN_TransmitStatus(FIN_CAN, MAIL) != CANTXOK);
}

// ____________________________________________________________________________
// Interrupt handler is called at each timer tick.
// The timer handler is called each 9ms. In order to avoid initialization issues
// at the beginning, the function waits for 100 cycles (900 ms) then initializes 
// the inertial sensor.
// After that, it starts with the procedure to collect all sensors data and send
// them to the UART. The procedure can be summarized as follows.
//
// START: flag = 1; flag_can = 1
//        FINGERS = 1; only if for SensHand devices
//
//      - if (flag)
//            flag = 0
//            if (flag_can)
//                if (FINGERS)
//                    flag_can = 0;
//                    collects data from fingers as explained in CAN_IntHandler.
//                    Receiving data from the Thumb is necessary to proceed with
//                    receiving data from the Index Finger, which is necessary
//                    to toggle flag_can and go on.
//
//                    flag_can = 1;
//                }
//                data are received fron the UART, which is expecting 1 byte to 
//                send sensor readings. This activates USART_IntHandler.
//
//                The handler composes the final array and send it to the UART, 
//                so to the BLE Module.
//                     
//                flag = 1;
//            }
//        }
//
void TIM2_IntHandler(void) 
{
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  if (START_UP) {
    count_start_up++;
    if (count_start_up == 100) {
      Sensor_Init();
      START_UP = 0;
    }
  }
  else { 
    if (flag) {
      flag = 0;
      if (flag_can) {
        // First, eventual data from the fingers are collected. FINGERS is 1 
        // only for SensHand devices otherwise this portion is just skipped. 
        // After collecting data, the
        if (FINGERS) {
          flag_can = 0;
          TX_CanBus(0x01, FIN1_CAN_ID);
        }
        
        // All fingers are recorded, at this point USART communication is 
        // handled. The USART is read and if a byte sent from the BLE Module is
        // present, it activated the event handler, allowing to creat the final
        // data array and send it back to the BLE Module.
//        REC  = USART_ReceiveData(USART2); ///// forse non serve
      }
    }
  }
}

// ____________________________________________________________________________
// CAN Interrupt handler 
// This function is necessary to handle data coming from CAN BUS linkers. Finger
// send arrays are sent using the first two terms are unsed to identify the IMU 
// (either Thumb or Index Finger) and the specific sensor (either Accelerometer 
// or Gyroscope):
// Arrays are expected to have lenght 8: 2 elements to correcly identify them
// and 6 elements to store actual data.
//
// This function is activated after sending a request to the Thumb, then acc and
// gyr arrays from the Thumb are collected in f1_data. Each of them updates the
// counter count_finger. 
//
// When count_finger = 2 (namely acc array and gyr array), then a request is
// sent to the Index finger and follows the same procedure, saving data in 
// f2_data.
// When count_finger = 2 again, then all finger information has been collected,
// flag_can is togglet to proceed and send data through the UART. 
//
void CAN_IntHandler(void)
{
  CAN_ClearITPendingBit(FIN_CAN, CAN_IT_FMP1);
  CAN_Receive(FIN_CAN, CAN_FIFO1, & RxMessage);
  
  switch(RxMessage.Data[0])
  {
  case 0x01:                                    // Thumb    
    // Save accelerometer data
    if (RxMessage.Data[1] == 'A') {
      for (int i = 0; i < 6; i++) {
        f1_data[i] = RxMessage.Data[i + 2];
      }
      count_finger++;
    }
    // Save gyroscope data
    if (RxMessage.Data[1] == 'G') {
      for (int i = 0; i < 6; i++) {
        f1_data[i + 6] = RxMessage.Data[i + 2];
      }
      count_finger++;
    }
    
    // After receiving all Thumb data, send message to the Index Finger. It 
    // should be noted that correcly collecting data from Thumb is necessary to
    // send a request to the Index Finger
    if (count_finger == 2) {
      count_finger = 0;
      TX_CanBus(0x02, FIN2_CAN_ID);
    }
    break;
  case 0x02:                                     // Index Finger
    // Save accelerometer data
    if (RxMessage.Data[1] == 'A') {
      for (int i = 0; i < 6; i++) {
        f2_data[i] = RxMessage.Data[i + 2];
      }
      count_finger++;
    }
    // Save gyroscope data
    if (RxMessage.Data[1] == 'G') {
      for (int i = 0; i < 6; i++) {
        f2_data[i + 6] = RxMessage.Data[i + 2];
      }
      count_finger++;
    }
    // After receiving all Index data, it proceeds with the USART interrupts. It
    // should be noted that correctly collecting data from Index Finger is 
    // necessary to toggle flag_can and go on with collecting data.
    if (count_finger == 2) {
      count_finger = 0;
      flag_can = 1;
    }
    break;   
  }
}

// ____________________________________________________________________________
// Interrupt handler is called at each UART event
// After collecting information from the fingers, the application collects from
// the UART. If a byte is received this event handler is activated, clearing 
// the received bit, creating the array to be sent and sending it.
void USART_IntHandler(void)
{
  ///// MODIFICA
  uint16_t receivedData = USART_ReceiveData(USART2);
  
  if ((uint8_t)(receivedData & (uint16_t)0x00FF) == 'E')
  {
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    Read_SensorData();
  }
  /////
  
  
  ///// Originale:
//  USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//  Read_SensorData();
  
}

// ____________________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________
void main(void)
{ 
  RCC_CLK_DISABLE();
  
  RCC_Config();  
  LED_Config();
  TIM_Config();
  UART_Config();
  
  CAN_Config();
  NVIC_Config();
  
  // If Device 2 or 3 are selected, data to be sent are only 12 bytes long and
  // fingers communication is unnecessary.
  if (DEVICE_ADDRESS == '2' || DEVICE_ADDRESS == '3') {
    data_len = 12;
    FINGERS  = 0;
  } 
  
  GPIO_WriteBit(LED_PORT,LED_PIN,Bit_SET);
  while (1) { }
}

// ____________________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________


// ____________________________________________________________________________
#ifdef  DEBUG
  /* User can add his own implementation to report the file name and line 
   * number. 
   * ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)*/
  void assert_failed(u8* file, u32 line) 
  {
    /*Infinite loop */
    while (1) { }
  }
#endif
// ____________________________________________________________________________
