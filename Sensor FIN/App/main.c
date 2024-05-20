/* Sensor_FIN
 ******************************************************************************
 * @file    main.c
 * @author  Michelangelo Guaitolini
 * @version v1.0.0
 * @date    2023
 * @brief   This formware provides the firmware to collect inertial data from
 *          a LSM9DS1 device, then send the data to main sensor module using CAN
 *          BUS communication.
 *          This application is aimed to program Thumb and Index Finger sensor 
 *          modules included in SensHand devices of the OLIMPIA sensor network 
 *          and it uses the following electronic components:
 *             - STM32F10x board.
 *             - LSM9DS1 inertial measurement unit
 ******************************************************************************
 * THE PRESENT FIRMWARE AIMS AT THE MANAGEMENT OF THE NODES WHICH CONSTITUTE THE 
 * FINGERS OF THE WIRELESS SENSOR NETWORK OF HANDi GLOVE DEVICE. DIFFERENT 
 * WORKING MODES ARE IMPLEMNETED TO ACQUIRE SENSOR DATA. A CALIBRATION PROCEDURE 
 * IS PROVIDED AND A KALAMAN FILTER LIBRARY IS IMPLEMENTED TO EXTRAPOLATE 3D 
 * SPACE HAND POSITION 
 *****************************************************************************/

// ____________________________________________________________________________
// Include --------------------------------------------------------------------
#include <assert.h>
#include <ctype.h>
#include <stdint.h>

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_usart.h"

#include "arm_comm.h"

// Timer and interrupts handlers
#include "stm32f10x_it.h"
#include "misc.h"

// Sensor
#include "LSM9DS1.h"

// DEVICE SETTINGS ------------------------------------------------------------
// The application is the same for the Thumb and the Index Finger. The user only
// has to manually select the correct address of the CAN BUS, while everything
// else is the same for both cases.

#define FINGER 0x01                             // Thumb
//#define FINGER 0x02                             // Index

// TIMER variables ------------------------------------------------------------
// Sampling frequency is needed to determine how often the sensor will produce a
// reading to be sent. Sampling frequency is set as ~111 Hz, the equivalent of a
// sampling period of 9ms.
int Fs = 1/(9*10^-3);                           // Sampling frequency [Hz]
int TIM_LED = 0;

// LED ------------------------------------------------------------------------
#define LED_PIN                     GPIO_Pin_1
#define LED_PORT                    GPIOB
#define LED_RCC_PERIPH              RCC_APB2Periph_GPIOB
#define LED_MODE                    GPIO_Mode_Out_PP
#define LED_SPEED                   GPIO_Speed_10MHz

// SENSOR variables -----------------------------------------------------------
// This application has to handle just an IMU, so vec has to be 12 bytes length
// since data are uint16_t and consist in 3D acceleration and 3D gyroscope 
// values (6 readings).
LSM_DATA m_data;                                // Raw data structure
uint8_t vec[12];                                // Sensor data to be sent

u8 START_UP = 1;
int count_start_up = 0;

// TEST variables -------------------------------------------------------------
uint8_t bytes[sizeof(float)];

// CAN variables --------------------------------------------------------------
#define __CAN1_USED__

#define FIN_CAN                         CAN1
#define CAN_PORT                        GPIOB
#define CAN_RCC_PORT                    RCC_APB2Periph_GPIOB

// Pins
#define CAN_RX_PIN                      GPIO_Pin_8
#define CAN_TX_PIN                      GPIO_Pin_9

// Mode
#define CAN_RX_MODE                     GPIO_Mode_IPU      
#define CAN_TX_MODE                     GPIO_Mode_AF_PP

#define CAN_GPIO_SPEED                  GPIO_Speed_10MHz

CanTxMsg TxMessage;
CanRxMsg RxMessage;

// ____________________________________________________________________________
// ____________________________________________________________________________
#define RCC_APB1Periph_ALL (RCC_APB1Periph_TIM2| RCC_APB1Periph_TIM3|\
                            RCC_APB1Periph_TIM4| RCC_APB1Periph_TIM5|\
                            RCC_APB1Periph_TIM6| RCC_APB1Periph_TIM7|\
                            RCC_APB1Periph_WWDG| RCC_APB1Periph_SPI2|\
                            RCC_APB1Periph_SPI3| RCC_APB1Periph_USART2|\
                            RCC_APB1Periph_USART3| RCC_APB1Periph_I2C1|\
                            RCC_APB1Periph_I2C2| RCC_APB1Periph_USB|\
                            RCC_APB1Periph_CAN1| RCC_APB1Periph_BKP|\
                            RCC_APB1Periph_PWR)

#define RCC_APB2Periph_ALL (RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB|\
                            RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOD|\
                            RCC_APB2Periph_AFIO| RCC_APB2Periph_ADC2|\
                            RCC_APB2Periph_TIM1| RCC_APB2Periph_SPI1|\
                            RCC_APB2Periph_TIM8|RCC_APB2Periph_USART1|\
                            RCC_APB2Periph_ADC3)

#define RCC_AHBPeriph_ALL (RCC_AHBPeriph_DMA1| RCC_AHBPeriph_DMA2|\
                           RCC_AHBPeriph_SRAM|RCC_AHBPeriph_FLITF|\
                           RCC_AHBPeriph_CRC)

#ifdef __cplusplus
 extern "C" {
#endif
ErrorStatus HSEStartUpStatus;

// ____________________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________


// Private functions __________________________________________________________
// ____________________________________________________________________________
void RCC_CLK_DISABLE()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_ALL, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALL, DISABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ALL, DISABLE);
}

// ____________________________________________________________________________
void RCC_Config(void)             
{
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);                                                    
  HSEStartUpStatus = RCC_WaitForHSEStartUp();	                                
  if(HSEStartUpStatus == SUCCESS)
  {	
    FLASH_SetLatency(FLASH_Latency_2);		                                
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);	                  							       
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_10);       /*40MHz*/                       
    RCC_PLLCmd(ENABLE);                  
    
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) { } 
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK );
    while(RCC_GetSYSCLKSource() != 0x08) { }
    
    RCC_HCLKConfig(RCC_SYSCLK_Div1);    /*40MHz*/	                                                        
    RCC_PCLK2Config(RCC_HCLK_Div4);     /*APB2:10MHz*/		                                
    RCC_PCLK1Config(RCC_HCLK_Div1);     /*APB1:40MHz*/
  }
}

// ____________________________________________________________________________
void TIM_Config()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  
  TIM_TimeBaseInitTypeDef TIM2_TimeBaseInitStruct; 
  TIM2_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;
  TIM2_TimeBaseInitStruct.TIM_Prescaler         = (3600 - 1)/Fs;
  TIM2_TimeBaseInitStruct.TIM_Period            = (2000 - 1);
  TIM2_TimeBaseInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
  TIM2_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2,&TIM2_TimeBaseInitStruct);
  
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2,TIM_FLAG_Update,ENABLE);
}

// ____________________________________________________________________________
void CAN_Config(void)
{
  // CAN GPIO -----------------------------------------------------------------
  RCC_APB2PeriphClockCmd(CAN_RCC_PORT, ENABLE);
  
  GPIO_InitTypeDef        GPIO_InitStr;
  GPIO_InitStr.GPIO_Pin   = CAN_TX_PIN;
  GPIO_InitStr.GPIO_Mode  = CAN_TX_MODE;
  GPIO_InitStr.GPIO_Speed = CAN_GPIO_SPEED;
  GPIO_Init(CAN_PORT, &GPIO_InitStr); 
  
  GPIO_InitStr.GPIO_Pin  = CAN_RX_PIN;
  GPIO_InitStr.GPIO_Mode = CAN_RX_MODE;
  GPIO_Init(CAN_PORT, &GPIO_InitStr);
  
  // CAN Init -----------------------------------------------------------------
  CAN_DeInit(FIN_CAN);
  CAN_InitTypeDef        CAN_InitStr;
  CAN_FilterInitTypeDef  CAN_FilterInitStr;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
  CAN_DeInit(FIN_CAN); 
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);
  
  //Configure CAN RX and TX pins
  CAN_InitStr.CAN_TTCM = DISABLE;               // triggered communication mode
  CAN_InitStr.CAN_ABOM = DISABLE;               // automatic bus-off management
  CAN_InitStr.CAN_AWUM = DISABLE;               // automatic wake up mode
  CAN_InitStr.CAN_NART = DISABLE;               // automatic no-trasmission mode
  CAN_InitStr.CAN_RFLM = DISABLE;               // receive FIFO locked mode
  CAN_InitStr.CAN_TXFP = ENABLE;                // transmit FIFO priority
  CAN_InitStr.CAN_Mode = CAN_Mode_Normal;  
  CAN_InitStr.CAN_SJW  = CAN_SJW_1tq; 
  CAN_InitStr.CAN_BS1  = CAN_BS1_4tq;
  CAN_InitStr.CAN_BS2  = CAN_BS2_5tq;
  CAN_InitStr.CAN_Prescaler = 4;                // 1MHz
  CAN_Init(FIN_CAN, &CAN_InitStr);

  CAN_FilterInitStr.CAN_FilterNumber = 2;
  CAN_FilterInitStr.CAN_FilterMode   = CAN_FilterMode_IdList;
  CAN_FilterInitStr.CAN_FilterScale  = CAN_FilterScale_32bit;
  switch (FINGER)
  {
  case 0x01:
    CAN_FilterInitStr.CAN_FilterIdHigh = 0x0012<<5;               // Thumb
    break;
  case 0x02:
    CAN_FilterInitStr.CAN_FilterIdHigh = 0x0013<<5;               // Index
    break;
  }
  
  CAN_FilterInitStr.CAN_FilterIdLow          = 0x0000;
  CAN_FilterInitStr.CAN_FilterMaskIdHigh     = 0x0000;          // 0x011
  CAN_FilterInitStr.CAN_FilterMaskIdLow      = 0x0000;
  CAN_FilterInitStr.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStr.CAN_FilterActivation     = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStr);
  
  CAN_ITConfig(FIN_CAN, CAN_IT_FMP0, ENABLE);
  
  // Transmit structure preparation
  TxMessage.StdId = 0x0011;                                                      // standard identifier
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
}

// ____________________________________________________________________________
// Timer and CAN interrupts
// NVIC configuration is necessary to set interrupts and event handlers for 
// TIMER and CAN, which are necessary to properly handle data streaming.
void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStr;
  /* Set the Vector Table base location at 0x08003000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
  
  // Timer interrupt
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  
  NVIC_InitStr.NVIC_IRQChannel                   = TIM2_IRQn;
  NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStr.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStr.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStr);
  
  // CAN interrupt
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStr.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn; 
  NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStr.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStr.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStr);
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
//              - Outout data rate (ODR) = 952 Hz (normal mode)
//              - Measurement range (FS) = ± 2000 dps
void Sensor_Init(void)
{
  // I2C ----------------------------------------------------------------------
  I2C_InitTypeDef  I2C_InitStr;
  GPIO_InitTypeDef GPIO_InitStr;
  
  RCC_APB1PeriphClockCmd(LSM_I2C_RCC_Periph, ENABLE);                           
  RCC_APB2PeriphClockCmd(LSM_I2C_RCC_Port, ENABLE);
  
  GPIO_InitStr.GPIO_Pin   =  LSM_I2C_SCL_Pin | LSM_I2C_SDA_Pin;             
  GPIO_InitStr.GPIO_Speed = GPIO_Speed_10MHz;
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
  
  // Acc and gyr Init ---------------------------------------------------------
  LSM9DS1_AG_ConfigTypeDef  LSM_Acc_Gyr_InitStr;
  
  LSM9DS1_A_ConfigTypeDef   LSM_Acc_InitStr;
  LSM9DS1_G_ConfigTypeDef   LSM_Gyr_InitStr;

  LSM_Acc_InitStr.Axes_Enable = LSM9DS1_A_XYZEN;
  LSM_Gyr_InitStr.Axes_Enable = LSM9DS1_G_XYZEN;
  
  LSM_Acc_InitStr.FS   = LSM9DS1_A_FS_8;
  LSM_Gyr_InitStr.FS   = LSM9DS1_G_FS_2000;
  LSM_Acc_InitStr.ODR  = LSM9DS1_A_ODR_952;
  LSM_Gyr_InitStr.ODR  = LSM9DS1_G_ODR_952;
  
  LSM_Acc_InitStr.Sens = LSM9DS1_Acc_Sens_8;
  LSM_Gyr_InitStr.Sens = LSM9DS1_Gyr_Sens_2000;
  LSM_Acc_InitStr.HR   = LSM9DS1_A_HR_ENABLE;
  
  LSM9DS1_A_Config(&LSM_Acc_InitStr);
  LSM9DS1_G_Config(&LSM_Gyr_InitStr);
  
  LSM_Acc_Gyr_InitStr.Address_Inc = LSM9DS1_AG_ADD_INC_ENABLE;
  LSM_Acc_Gyr_InitStr.Data_Update = LSM9DS1_AG_BDU_Continuos;
  LSM_Acc_Gyr_InitStr.Endianess   = LSM9DS1_AG_Little_Endian;
  LSM9DS1_AG_Config(&LSM_Acc_Gyr_InitStr);
}

// ____________________________________________________________________________
void LED_Config(void)
{
  RCC_APB2PeriphClockCmd( LED_RCC_PERIPH, ENABLE);
  
  GPIO_InitTypeDef       GPIO_InitStr;
  GPIO_InitStr.GPIO_Pin   = LED_PIN;
  GPIO_InitStr.GPIO_Mode  = LED_MODE;
  GPIO_InitStr.GPIO_Speed = LED_SPEED;
  GPIO_Init(LED_PORT, &GPIO_InitStr);
}

// ____________________________________________________________________________
void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  // Check the parameters
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->ODR ^= GPIO_Pin;
}

// ____________________________________________________________________________
// Function that reads data from IMU sensors. Data are stored in a vector and 
// sent through CAN BUS. Readings are accelerometer and gyroscope readings,
// and once they are collected into vec they are sent separately via CAN BUS 
// which handles short messages better.
// To easily assess if readings are from accelerometer or gyroscope a front byte
// is added and its value is "A" for accelerometer or "G" for gyroscope.
//
// This function is handled by the CAN1_IntrHandler, which is activated by
// receiving a request on the CAN register by the main board module.
void Read_SensorData(void) {
  u8 MAIL;
  
  // Sensor data
  LSM_DataProcess(&m_data);
  
  vec[0]  = (uint8_t)(((int16_t)m_data.sAcc[0]));               // Acc x
  vec[1]  = (uint8_t)(((int16_t)m_data.sAcc[0])>>8);
  vec[2]  = (uint8_t)(((int16_t)m_data.sAcc[1]));               // Acc y
  vec[3]  = (uint8_t)(((int16_t)m_data.sAcc[1])>>8);
  vec[4]  = (uint8_t)(((int16_t)m_data.sAcc[2]));               // Acc z
  vec[5]  = (uint8_t)(((int16_t)m_data.sAcc[2])>>8);
  vec[6]  = (uint8_t)(((int16_t)m_data.sGyr[0]));               // Gyr x
  vec[7]  = (uint8_t)(((int16_t)m_data.sGyr[0])>>8);
  vec[8]  = (uint8_t)(((int16_t)m_data.sGyr[1]));               // Gyr y
  vec[9]  = (uint8_t)(((int16_t)m_data.sGyr[1])>>8);
  vec[10] = (uint8_t)(((int16_t)m_data.sGyr[2]));               // Gyr z
  vec[11] = (uint8_t)(((int16_t)m_data.sGyr[2])>>8);
  
  // Finger readings are sent after specific requests. The request specifies
  // which finger is expected to send data. Data to be sent will start, then,
  // with a first element specifying the finger, a second element specifying
  // the sensor (if accelerometer of gyroscop), then there will be sensors 
  // readings.
  // Read CAN Port, if the specific FINGER is called, then send data
  if (RxMessage.Data[0] == FINGER) {
    TxMessage.Data[0] = FINGER;
    
    // Send accelerometer data.
    TxMessage.Data[1] = 'A';
    for (int i = 0; i < 6; i++) {
      TxMessage.Data[i + 2] = vec[i];
    }
    MAIL = CAN_Transmit(CAN1, &TxMessage);
    while (CAN_TransmitStatus(FIN_CAN, MAIL) != CANTXOK);
    
    // Send gyroscope data.
    TxMessage.Data[1] = 'G';
    for (int i = 0; i < 6; i++) {
      TxMessage.Data[i + 2] = vec[i + 6];
    }
    MAIL = CAN_Transmit(CAN1, &TxMessage);
    while (CAN_TransmitStatus(FIN_CAN, MAIL) != CANTXOK);
  }
}

// ____________________________________________________________________________
// This interrupt handler handles sensor readings and it is activated every time
// the CAN register get a request from the Main Board Module. 
void CAN1_IntrHandler(void)
{
  if (!START_UP) {
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    //
    TIM_LED++;
    if (TIM_LED == 111) {
      GPIO_TogglePin(LED_PORT, LED_PIN);
      TIM_LED = 0;
    }
    Read_SensorData();
  }
}

// ____________________________________________________________________________
// Interrupt handler is called at each timer tick.
// The timer is not involved in collecting data here but it is useful for a 
// correct startup procedure. Configuring the sensor and the CAN BUS after some
// cycles avoids possible issues.
void TIM2_IntrHandler()
{
  TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
  if (START_UP) {
    count_start_up++;
    if (count_start_up == 100) {
      CAN_Config();
      Sensor_Init();
      START_UP = 0;
    }
  }
}

// ____________________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________
void main(void)
{  
  RCC_CLK_DISABLE();
  RCC_Config();
  TIM_Config();
  
  NVIC_Config();
  LED_Config();
  
  GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
  while(1) {}
}

// ____________________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________


// ____________________________________________________________________________
#ifdef  DEBUG
void assert_failed(u8* file, u32 line)                                          /*User can add his own implementation to report the file name and line number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
{  
  while (1)                                                                     /*Infinite loop */
  {
  }
}
#endif
// ____________________________________________________________________________
