/* HANDi_CORV3.0
  ************************************************************************************
  * @file    USB.c
  * @author  Dario Esposito
  * @version V3.0
  * @date    04/05/2016
  * @brief   This file provides the writing\reading routines to communicate by means  
  *          VirCOM.
  ************************************************************************************
  * 
  *  
  * 
  ************************************************************************************/
#include "USB.h"
#include "DeviceSettings.h"
Int32U USBDataRx,TranSize;
Int8U IN_BUFFER[100];
pInt8U BufferUSB;
u8 BaudChanged=0;
uint32_t Int32DefaultBaud;
u8 USB_VEC[100];
u8 DFU_MODE=0;

/*----------------------------------------------------------------------------*/
void USB_DATA_READ(int USB_NUM, uint16_t *Int16DefaultBaud)
{
  USBDataRx = UsbCdcRead(IN_BUFFER,sizeof(IN_BUFFER)-1);
  if(USBDataRx)
  { 
   TranSize = 0;
   BufferUSB = IN_BUFFER;
   do
   {
     USBDataRx -= TranSize;
     BufferUSB += TranSize;
     TranSize = UartWrite(USB_UART,BufferUSB,USBDataRx);
     if((IN_BUFFER[0]=='a')&&(IN_BUFFER[1]=='t')&&(IN_BUFFER[2]=='$')&&(IN_BUFFER[3]=='b')&&(IN_BUFFER[4]=='d')&&(IN_BUFFER[5]=='r')&& (IN_BUFFER[6]==':') && (IN_BUFFER[TranSize-1]==0x0D))/*ChangeDefaultBaud*/
     {
       int BAUD_INDEX=0;
       int USB_INDEX=7;
       int num=TranSize-1-USB_INDEX;
       char *DefaultBaud = (char *) malloc(sizeof(int) * num);
       while(USB_INDEX<(TranSize-1))
       {
         DefaultBaud[BAUD_INDEX]=IN_BUFFER[USB_INDEX];
         USB_INDEX++;  
         BAUD_INDEX++;
       }
       Int32DefaultBaud=atoi(DefaultBaud);
       *Int16DefaultBaud=Int32DefaultBaud/100;
       USB_NUM=TranSize-1;
       BLE_sendDATA(USB_NUM,IN_BUFFER);
       for(int i=0; i<=(TranSize-1);i++) IN_BUFFER[i]=0; 
       free(DefaultBaud);
     }
   }
   while(USBDataRx != TranSize); 
  }
}
/*----------------------------------------------------------------------------*/
void USB_DATA_SEND(u8 USBDataTx)
{
 if(USBDataTx)
 {
    u8 USB_DATA[]="OK\r\n";
    while(!UsbCdcWrite(USB_DATA,USB_NUM_DATA));
   // BaudChanged=BLE_SOF_RESET(BaudChanged,Int32DefaultBaud); 
 }
 else if(DFU_MODE)
 {
#if defined(LEFT)
   u8 USB_DATA[]="LG";
#elif defined(RIGHT)
   u8 USB_DATA[]="RG";
#endif
   while(!UsbCdcWrite(USB_DATA,DFU_NUM_DATA));
 }
}
/*----------------------------------------------------------------------------*/
void VirCOM_CON(void)
{
  UartInit(USB_UART,3);
  UsbCdcInit();
  USB_Connect(ENABLE);
}
/*----------------------------------------------------------------------------*/
void VirCOM_DIS(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB,DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);
  USB_Connect(DISABLE);
}
/*----------------------------------------------------------------------------*/