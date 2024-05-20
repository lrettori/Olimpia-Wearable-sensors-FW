/* HANDi_FINV1.0
  *****************************************************************************
  * @file    LSM9DS1.c
  * @author  Dario Esposito
  * @version V3.0
  * @date    15/12/2016
  * @brief   This file provides the writing\reading routines to communicate with 
  *          the accelerometer sensor.
  *****************************************************************************
  * 
  *  
  * 
  ****************************************************************************/

#include "LSM9DS1.h"
#include "HAL_LSM9DS1.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"

#define I2C2_DMA_CHANNEL_TX             DMA1_Channel4
#define I2C2_DMA_CHANNEL_RX             DMA1_Channel5
#define I2C2_DR_Address                 0x40005810

LSM9DS1_A_ConfigTypeDef                 LSM_Acc_InitStructure;
LSM9DS1_G_ConfigTypeDef                 LSM_Gyr_InitStructure;
LSM9DS1_M_ConfigTypeDef                 LSM_Mag_InitStructure;

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
void I2C_DMA_Config(I2C_TypeDef* I2Cx, uint8_t* pBuffer, uint32_t lBufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)I2C2_DR_Address;          /* Initialize the DMA_PeripheralBaseAddr member: I2C_DR_ADDRESS=0x40005810 */
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)pBuffer;                 /* Initialize the DMA_MemoryBaseAddr member */
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;             /* Initialize the DMA_DIR member */
  DMA_InitStructure.DMA_BufferSize         = lBufferSize;                       /* Initialize the DMA_BufferSize member */
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;         /* Initialize the DMA_PeripheralInc member */
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;              /* Initialize the DMA_MemoryInc member */
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;       /* Initialize the DMA_PeripheralDataSize member */
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;           /* Initialize the DMA_MemoryDataSize member */
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;                   /* Initialize the DMA_Mode member */
  DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;             /* Initialize the DMA_Priority member */
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;  
  DMA_Cmd(I2C2_DMA_CHANNEL_RX, DISABLE); 
  DMA_Init(I2C2_DMA_CHANNEL_RX,&DMA_InitStructure);                              /* I2C2_DMA_CHANNEL_RX*/           
  DMA_Cmd(I2C2_DMA_CHANNEL_RX, ENABLE);
}

// ----------------------------------------------------------------------------
void LSM_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite)
{
  while(I2C_GetFlagStatus(LSM_I2C, I2C_FLAG_BUSY));                              
  I2C_GenerateSTART(LSM_I2C, ENABLE);                                            
  
  while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_MODE_SELECT));              
  I2C_Send7bitAddress(LSM_I2C, slAddr, I2C_Direction_Transmitter);                 
  
  while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    
  I2C_SendData(I2C2, WriteAddr);                                                
  
  while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));            
  for(uint8_t i=0 ; i<NumByteToWrite ; i++) {
    I2C_SendData(LSM_I2C, pBuffer[i]);                                                
    while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));          
  }
  I2C_GenerateSTOP(LSM_I2C, ENABLE);                                             
}

// ----------------------------------------------------------------------------
void LSM_I2C_DMA_BufferRead(u8 slAddr, u8* pBuffer, u8 ReadAddr, 
                            u16 NumByteToRead)
{
  __IO uint32_t temp = 0;
  __IO uint32_t Timeout = 0;
  /* LSM_I2C->CR2 |= I2C_IT_ERR;   */                                           /* Enable I2C errors interrupts */    
  while(I2C_GetFlagStatus(LSM_I2C, I2C_FLAG_BUSY));                             /* While the bus is busy */
  I2C_GenerateSTART(LSM_I2C, ENABLE);                                           /* Send START condition */
  
  while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_MODE_SELECT));                /* Test on EV5 and clear it */
  I2C_Send7bitAddress(LSM_I2C,slAddr, I2C_Direction_Transmitter);               /* Send LSM303DLH address for write */
  
  while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  /* Test on EV6 and clear it */
  I2C_Cmd(LSM_I2C, ENABLE);                                                     /* Clear EV6 by setting again the PE bit */
  I2C_SendData(LSM_I2C, ReadAddr);                                              /* Send the LSM303DLH_Magn's internal address to write to */
  
  while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));           /* Test on EV8 and clear it */
  I2C_DMA_Config(LSM_I2C,pBuffer, NumByteToRead);                               /* Configure I2Cx DMA channel */       
  LSM_I2C->CR2 |= 0x1000;                                                       /* Set Last bit to have a NACK on the last received byte */
  I2C_DMACmd(LSM_I2C, ENABLE);                                                  /* Enable I2C DMA requests */
  Timeout = 0xFFFF;  
  I2C_GenerateSTART(LSM_I2C, ENABLE);                                           /* Send START condition a second time */
  
  while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_MODE_SELECT));                /* Test on EV5 and clear it */
  {
    if (Timeout-- == 0)
      return;
  }
  Timeout = 0xFFFF;
  I2C_Send7bitAddress(LSM_I2C, slAddr, I2C_Direction_Receiver);                 /* Send LSM303DLH address for read */ 
  while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));     /* Test on EV6 and clear it */
  {
    if (Timeout-- == 0)
      return;
  }
  temp = LSM_I2C->SR2;                                                          /* Clear ADDR flag by reading SR2 register */
  while (!DMA_GetFlagStatus(DMA1_FLAG_TC5));                                    /* Wait until DMA end of transfer */
  DMA_Cmd(I2C2_DMA_CHANNEL_RX, DISABLE);                                            /* Disable DMA Channel */
  DMA_ClearFlag(DMA1_FLAG_TC5);                                                 /* Clear the DMA Transfer Complete flag */ 
  I2C_AcknowledgeConfig(LSM_I2C, DISABLE);                                      /* Disable Ack for the last byte */
  I2C_GenerateSTOP(LSM_I2C, ENABLE);                                            /* Send STOP Condition */
  while ((LSM_I2C->CR1 & 0x0200) == 0x0200);                                    /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
  I2C_AcknowledgeConfig(LSM_I2C, ENABLE);                                       /* Enable Acknowledgement to be ready for another reception */
}

// ----------------------------------------------------------------------------
void LSM_I2C_BufferRead(u8 slAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead) 
{        
//    if(NumByteToRead>1)                                                        
//    ReadAddr |= 0x80; 
    while(I2C_GetFlagStatus(LSM_I2C, I2C_FLAG_BUSY));                           /* While the bus is busy */
    I2C_GenerateSTART(LSM_I2C, ENABLE);                                         /* Send START condition */
    while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_MODE_SELECT));              /* Test on EV5 and clear it */
    I2C_Send7bitAddress(LSM_I2C,slAddr, I2C_Direction_Transmitter);             /* Send LSM303DLH address for write */
    while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));/* Test on EV6 and clear it */
    I2C_Cmd(LSM_I2C, ENABLE);                                                   /* Clear EV6 by setting again the PE bit */
    I2C_SendData(LSM_I2C, ReadAddr);                                            /* Send the LSM303DLH_Magn's internal address to write to */
    while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));         /* Test on EV8 and clear it */
    I2C_GenerateSTART(LSM_I2C, ENABLE);                                         /* Send START condition a second time */
    while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_MODE_SELECT));              /* Test on EV5 and clear it */
    I2C_Send7bitAddress(LSM_I2C, slAddr, I2C_Direction_Receiver);               /* Send LSM303DLH address for read */ 
    while(!I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));   /* Test on EV6 and clear it */
    while(NumByteToRead)                                                        /* While there is data to be read */
    {  
      if(NumByteToRead == 1)
      {
        if(I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))               /* Test on EV7 and clear it */
        {
          *pBuffer = I2C_ReceiveData(LSM_I2C);                                    /* Read a byte from the LSM303DLH */
          pBuffer++;                                                            /* Point to the next location where the byte read will be saved */  
          NumByteToRead--;                                                        /* Decrement the read bytes counter */
          I2C_AcknowledgeConfig(LSM_I2C, DISABLE);                                /* Disable Acknowledgement */ 
          I2C_GenerateSTOP(LSM_I2C, ENABLE);                                      /* Send STOP Condition */
        }

      } 
      else
      {
        if(I2C_CheckEvent(LSM_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))               /* Test on EV7 and clear it */
        {
          *pBuffer = I2C_ReceiveData(LSM_I2C);                                    /* Read a byte from the LSM303DLH */
           pBuffer++;                                                            /* Point to the next location where the byte read will be saved */  
          NumByteToRead--;                                                        /* Decrement the read bytes counter */
        }
        //I2C_AcknowledgeConfig(LSM_I2C, ENABLE);
      }
    }   
    I2C_AcknowledgeConfig(LSM_I2C, ENABLE);                                     /* Enable Acknowledgement to be ready for another reception */
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_A_Config(LSM9DS1_A_ConfigTypeDef *LSM_Acc_Config_Struct)        
{
  u8 CTRL_REG5 = 0x00;
  u8 CTRL_REG6 = 0x00;  
  u8 CTRL_REG7 = 0x00;
  
  CTRL_REG5 |= (u8) (LSM_Acc_Config_Struct->Axes_Enable);
  CTRL_REG6 |= (u8) (LSM_Acc_Config_Struct->ODR|LSM_Acc_Config_Struct->FS); 
  CTRL_REG7 |= (u8) (LSM_Acc_Config_Struct->HR);
  
  LSM_I2C_ByteWrite(LSM9DS1_AG_I2C_ADDRESS, &CTRL_REG5, CTRL_REG5_XL_ADDR,1);
  LSM_I2C_ByteWrite(LSM9DS1_AG_I2C_ADDRESS, &CTRL_REG6, CTRL_REG6_XL_ADDR,1);
  LSM_I2C_ByteWrite(LSM9DS1_AG_I2C_ADDRESS, &CTRL_REG7, CTRL_REG7_XL_ADDR,1);
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_G_Config(LSM9DS1_G_ConfigTypeDef *LSM_Gyr_Config_Struct)        
{
  u8 CTRL_REG1 = 0x00;
  u8 CTRL_REG3 = 0x00;  
  u8 CTRL_REG4 = 0x00;
  
  CTRL_REG1 |= (u8) (LSM_Gyr_Config_Struct->ODR|LSM_Gyr_Config_Struct->FS);          
  CTRL_REG3 |= (u8) (LSM_Gyr_Config_Struct->Low_Power_Mode);
  CTRL_REG4 |= (u8) (LSM_Gyr_Config_Struct->Axes_Enable);
  
  LSM_I2C_ByteWrite(LSM9DS1_AG_I2C_ADDRESS, &CTRL_REG1, CTRL_REG1_G_ADDR,1);
  LSM_I2C_ByteWrite(LSM9DS1_AG_I2C_ADDRESS, &CTRL_REG3, CTRL_REG3_G_ADDR,1);
  LSM_I2C_ByteWrite(LSM9DS1_AG_I2C_ADDRESS, &CTRL_REG4, CTRL_REG4_ADDR,1);
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_AG_Config(LSM9DS1_AG_ConfigTypeDef *LSM_Acc_Gyr_Config_Struct)        
{
  u8 CTRL_REG8 = 0x00;
  
  CTRL_REG8 |= (u8) ( LSM_Acc_Gyr_Config_Struct->Data_Update| LSM_Acc_Gyr_Config_Struct->Endianess| LSM_Acc_Gyr_Config_Struct->Address_Inc);
  
  LSM_I2C_ByteWrite(LSM9DS1_AG_I2C_ADDRESS, &CTRL_REG8, CTRL_REG8_ADDR,1);
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_M_Config(LSM9DS1_M_ConfigTypeDef *LSM_Magn_Config_Struct)     
{
  u8 CTRL_REG1 = 0x00;
  u8 CTRL_REG2 = 0x00;
  u8 CTRL_REG4 = 0x00;
  u8 CTRL_REG5 = 0x00;
  u8 CTRL_REG3 = 0x00;

  
  CTRL_REG1 |= (u8) (LSM_Magn_Config_Struct->ODR|LSM_Magn_Config_Struct->XY_Mode);
  CTRL_REG2 |= (u8) (LSM_Magn_Config_Struct->FS);
  CTRL_REG3 |= (u8) (LSM_Magn_Config_Struct->Low_Power_Mode|LSM_Magn_Config_Struct->Mode);
  CTRL_REG4 |= (u8) (LSM_Magn_Config_Struct->Z_Mode|LSM_Magn_Config_Struct->Endianess);
  CTRL_REG5 |= (u8) (LSM_Magn_Config_Struct->Data_Update);
  

  LSM_I2C_ByteWrite(LSM9DS1_M_I2C_ADDRESS, &CTRL_REG1, CTRL_REG1_M_ADD,1);   
  LSM_I2C_ByteWrite(LSM9DS1_M_I2C_ADDRESS, &CTRL_REG2, CTRL_REG2_M_ADD,1);
  LSM_I2C_ByteWrite(LSM9DS1_M_I2C_ADDRESS, &CTRL_REG3, CTRL_REG3_M_ADD,1);
  LSM_I2C_ByteWrite(LSM9DS1_M_I2C_ADDRESS, &CTRL_REG4, CTRL_REG4_M_ADD,1);
  LSM_I2C_ByteWrite(LSM9DS1_M_I2C_ADDRESS, &CTRL_REG5, CTRL_REG5_M_ADD,1); 
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_A_Read_RawData(s16* out)
{
  u8 buffer[6];
  LSM_I2C_DMA_BufferRead(LSM9DS1_AG_I2C_ADDRESS, buffer, OUT_XL_A_REG_ADD, 6);
  for(int i=0; i<3; i++)
  out[i]=((s16)((u16)buffer[2*i+1] << 8) + buffer[2*i]);
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_G_Read_RawData(s16* out)
{
  u8 buffer[6];
  LSM_I2C_BufferRead(LSM9DS1_AG_I2C_ADDRESS, buffer, OUT_XL_G_REG_ADD, 6);
  for(int i=0; i<3; i++)
  out[i]=((s16)((u16)buffer[2*i+1] << 8) + buffer[2*i]);
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_GA_Read_RawData(s16* out)
{
  u8 buffer[12];
  LSM_I2C_DMA_BufferRead(LSM9DS1_AG_I2C_ADDRESS, buffer, OUT_XL_G_REG_ADD, 12);
  for(int i=0; i<6; i++)
  out[i]=((s16)((u16)buffer[2*i+1] << 8) + buffer[2*i]);
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_M_Read_RawData(s16* out)
{
  u8 buffer[6];
  LSM_I2C_DMA_BufferRead(LSM9DS1_M_I2C_ADDRESS, buffer, OUT_XL_M_REG_ADD, 6);
  for(int i=0; i<3; i++)
  out[i]=((s16)((u16)buffer[2*i+1] << 8) + buffer[2*i]);
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_Acc_Gyr_Config(void)                                                        
{	
  LSM9DS1_AG_ConfigTypeDef  LSM_Acc_Gyr_InitStructure;
  LSM_Acc_InitStructure.ODR         = LSM9DS1_A_ODR_952;
  LSM_Acc_InitStructure.Axes_Enable = LSM9DS1_A_XYZEN;
  
  LSM_Acc_InitStructure.FS          = LSM9DS1_A_FS_2;
  LSM_Acc_InitStructure.Sens        = LSM9DS1_Acc_Sens_2;
   
  LSM_Acc_InitStructure.HR          = LSM9DS1_A_HR_ENABLE;
  LSM9DS1_A_Config(&LSM_Acc_InitStructure);
  
  LSM_Gyr_InitStructure.ODR            = LSM9DS1_G_ODR_952;
  
  LSM_Gyr_InitStructure.FS             = LSM9DS1_G_FS_2000;
  LSM_Gyr_InitStructure.Sens           =LSM9DS1_Gyr_Sens_2000;

  LSM_Gyr_InitStructure.Axes_Enable    = LSM9DS1_G_XYZEN;
  LSM9DS1_G_Config(&LSM_Gyr_InitStructure);
  
  LSM_Acc_Gyr_InitStructure.Address_Inc  = LSM9DS1_AG_ADD_INC_ENABLE;
  LSM_Acc_Gyr_InitStructure.Data_Update  = LSM9DS1_AG_BDU_Continuos;
  LSM_Acc_Gyr_InitStructure.Endianess    = LSM9DS1_AG_Little_Endian;
  LSM9DS1_AG_Config(&LSM_Acc_Gyr_InitStructure);
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_Mag_Config(u8 magn_en)                                                        
{	
  LSM_Mag_InitStructure.ODR            = LSM9DS1_M_ODR_80;
  
  LSM_Mag_InitStructure.FS             = LSM9DS1_M_FS_4;
  LSM_Mag_InitStructure.Sens           = LSM9DS1_Mag_Sens_4;
 
  LSM_Mag_InitStructure.XY_Mode        = LSM9DS1_M_LOW_POWER_XY;
  LSM_Mag_InitStructure.Z_Mode         = LSM9DS1_M_LOW_POWER_Z;
  LSM_Mag_InitStructure.Data_Update    = LSM9DS1_M_BDU_Continuos;
  LSM_Mag_InitStructure.Endianess      = LSM9DS1_M_Little_Endian;
  LSM_Mag_InitStructure.Low_Power_Mode = LSM9DS1_M_LOW_POWER_DIS;
  switch(magn_en)
  {
    case DISABLE:
      LSM_Mag_InitStructure.Mode = LSM9DS1_M_POWER_DOWN; 
    break;
    case ENABLE:
      LSM_Mag_InitStructure.Mode = LSM9DS1_M_CONTINUOUS; 
    break;
  }
  LSM9DS1_M_Config(&LSM_Mag_InitStructure);
}
/*----------------------------------------------------------------------------*/
void LSM9DS1_I2C_Init(void)                                                  
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
                                                                              
  RCC_APB1PeriphClockCmd(LSM_I2C_RCC_Periph, ENABLE);                           
  RCC_APB2PeriphClockCmd(LSM_I2C_RCC_Port, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin =  LSM_I2C_SCL_Pin | LSM_I2C_SDA_Pin;             
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(LSM_I2C_Port, &GPIO_InitStructure);
  
  I2C_DeInit(LSM_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                                    
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed =LSM_I2C_Speed;   
  I2C_Init(LSM_I2C, &I2C_InitStructure);                                       
  I2C_Cmd(LSM_I2C, ENABLE);                                                    
}
/*----------------------------------------------------------------------------*/
void SensorsConfig(uint8_t mag_enable)
{
  LSM9DS1_I2C_Init();
  LSM9DS1_Acc_Gyr_Config();
  LSM9DS1_Mag_Config(mag_enable);
}
/*----------------------------------------------------------------------------*/
void LSM_DataProcess(LSM_DATA *mData)
{
  LSM9DS1_GA_Read_RawData(mData->sGyrAcc);
  LSM9DS1_M_Read_RawData(mData->sMag);
  /*--------------------------RAW_DATA--------------------------------*/  
  mData->m_fGyr[0] = -(((float)((*mData).sGyrAcc[0]))* LSM9DS1_Gyr_Sens_2000)/1000;
  mData->m_fGyr[1] = -(((float)((*mData).sGyrAcc[1]))* LSM9DS1_Gyr_Sens_2000)/1000; 
  mData->m_fGyr[2] = -(((float)((*mData).sGyrAcc[2]))* LSM9DS1_Gyr_Sens_2000)/1000;
  mData->m_fAcc[0] = -(((float)((*mData).sGyrAcc[3]))* LSM9DS1_Acc_Sens_2)/1000;
  mData->m_fAcc[1] = -(((float)((*mData).sGyrAcc[4]))* LSM9DS1_Acc_Sens_2)/1000;
  mData->m_fAcc[2] = -(((float)((*mData).sGyrAcc[5]))* LSM9DS1_Acc_Sens_2)/1000; 
  mData->m_fMag[0] = -(((float)((*mData).sMag[0]))   * LSM9DS1_Mag_Sens_4)/1000;
  mData->m_fMag[1] = -(((float)((*mData).sMag[1]))   * LSM9DS1_Mag_Sens_4)/1000; 
  mData->m_fMag[2] = -(((float)((*mData).sMag[2]))   * LSM9DS1_Mag_Sens_4)/1000;
}

// ----------------------------------------------------------------------------