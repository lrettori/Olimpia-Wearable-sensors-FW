/**
 * @file    LSM9DS1.h
 * @author  Dario Esposito
 * @version V1.0.0
 * @date    14 December 2016
 * @brief   Header for LSM9DS1.c file
 * @details
 *
 */



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM9DS1_H
#define __LSM9DS1_H

/* Includes ------------------------------------------------------------------*/
#include "HAL_LSM9DS1.h"
/* Sensors Address -----------------------------------------------------------*/
#define LSM9DS1_AG_I2C_ADDRESS          0xD4
#define LSM9DS1_M_I2C_ADDRESS           0x38
/* Accelerometer/Gyroscope register map --------------------------------------*/
 
#define ACT_THS_REG_ADDR                0x04  /*Activity threshold register*/
#define ACT_DUR_REG_ADDR                0x05  /*Inactivity duration register*/


// Linear accelerator interrupt
#define INT_GEN_CFG_XL_REG_ADDR         0x06                                    /*Linear accelerator interrupt generator  register*/
#define INT_GEN_THS_X_XL_REG_ADDR       0x07                                    /*Linear accelerator sensor interrupt register*/
#define INT_GEN_THS_Y_XL_REG_ADDR       0x08  
#define INT_GEN_THS_Z_XL_REG_ADDR       0x09  
#define INT_GEN_DUR_XL_REG_ADDR         0x0A  

// Angular rate reference
#define REFERENCE_G_REG_ADDR            0x0B                                    /*Angular rate reference for high-pass filter*/
#define INT1_CTRL_REG_ADDR              0x0C  
#define INT2_CTRL_REG_ADDR              0x0D  
#define WHO_AM_I_REG_ADDR               0x0F  
#define CTRL_REG1_G_ADDR                0x10                                    /*Angular rate sensor register 1*/
#define CTRL_REG2_G_ADDR                0x11  
#define CTRL_REG3_G_ADDR                0x12  
#define ORIENT_CFG_G_REG_ADDR           0x13                                    /*Angular rate sensor sign and orientation*/
#define INT_GEN_SRC_G_REG_ADDR          0x14                                    /*Angular rate sensor interrupt source*/

// Temperature register
#define OUT_TEMP_L_REG_ADDR             0x15                                    /*Temperature register L*/
#define OUT_TEMP_H_REG_ADDR             0x16                                    /*Temperature register H*/

#define STATUS_REG_ADDR                 0x17                                    /*Status register */

// Angular rate data
#define OUT_XL_G_REG_ADD                0x18                                    /*angular rate XL axis*/
#define OUT_XH_G_REG_ADD                0x19                                    /*angular rate XH axis */
#define OUT_YL_G_REG_ADD                0x1A                                    /*angular rate YL axis*/
#define OUT_YH_G_REG_ADD                0x1B                                    /*angular rate YH axis*/
#define OUT_ZL_G_REG_ADD                0x1C                                    /*angular rate ZL axis*/
#define OUT_ZH_G_REG_ADD                0x1D                                    /*angular rate ZH axis*/

#define CTRL_REG4_ADDR                  0x1E                                    /*control register 4 */

#define CTRL_REG5_XL_ADDR               0x1F                                    /*Lienar acceleration control register 5 */
#define CTRL_REG6_XL_ADDR               0x20                                    /*Lienar acceleration control register 6 */
#define CTRL_REG7_XL_ADDR               0x21                                    /*Lienar acceleration control register 7 */

#define CTRL_REG8_ADDR                  0x22                                    /*control register 8 */
#define CTRL_REG9_ADDR                  0x23                                    /*control register 9 */
#define CTRL_REG10_ADDR                 0x24                                    /*control register 10 */

#define INT_GEN_SRC_XL_REG_ADD          0x26                                    /*lienar acceleration interrupt source */

#define OUT_XL_A_REG_ADD                0x28                                    /*linear acceleration XL axis*/
#define OUT_XH_A_REG_ADD                0x29                                    /*linear acceleration  rate XH axis */
#define OUT_YL_A_REG_ADD                0x2A                                    /*linear acceleration  rate YL axis*/
#define OUT_YH_A_REG_ADD                0x2B                                    /*linear acceleration  rate YH axis*/
#define OUT_ZL_A_REG_ADD                0x2C                                    /*linear acceleration  rate ZL axis*/
#define OUT_ZH_A_REG_ADD                0x2D                                    /*linear acceleration  rate ZH axis*/

#define FIFO_CTRL_REG_ADDR              0x2E                                    /*FIFO control register*/
#define FIFO_SRC_REG_ADDR               0x2F                                    /*FIFO  ststus control register*/

#define INT_GEN_CFG_G_REG_ADDR          0x30                                    /*angular rate interrup generator*/

#define INT_GEN_THS_XH_G_REG_ADDR       0x31                                    /*angular rate interrup generator threshold*/
#define INT_GEN_THS_XL_G_REG_ADDR       0x32
#define INT_GEN_THS_YH_G_REG_ADDR       0x33  
#define INT_GEN_THS_YL_G_REG_ADDR       0x34  
#define INT_GEN_THS_ZH_G_REG_ADDR       0x35  
#define INT_GEN_THS_ZL_G_REG_ADDR       0x36  

#define INT_GEN_DUR_G_REG_ADDR          0x37                                    /*angular rate interrup generator duration*/

#define OFFSET_X_REG_L_M_ADD            0x05                                    /*magnetometer X offset*/
#define OFFSET_X_REG_H_M_ADD            0x06                                    /*magnetometer X offset*/
#define OFFSET_Y_REG_L_M_ADD            0x07                                    /*magnetometer Y offset*/
#define OFFSET_Y_REG_H_M_ADD            0x08                                    /*magnetometer Y offset*/
#define OFFSET_Z_REG_L_M_ADD            0x09                                    /*magnetometer Z offset*/
#define OFFSET_Z_REG_H_M_ADD            0x0A                                    /*magnetometer Z offset*/

#define WHO_AM_I_M_REG_ADD              0x0F 

#define CTRL_REG1_M_ADD                 0x20                                    /*magnetometer control register 1-5*/
#define CTRL_REG2_M_ADD                 0x21  
#define CTRL_REG3_M_ADD                 0x22  
#define CTRL_REG4_M_ADD                 0x23  
#define CTRL_REG5_M_ADD                 0x24  

#define STATUS_REG_M_ADD                0x27                                    /*magnetometer status register*/
#define OUT_XL_M_REG_ADD                0x28                                    /*magnetometer X data*/
#define OUT_XH_M_REG_ADD                0x29                                    /*magnetometer X data*/
#define OUT_YL_M_REG_ADD                0x2A                                    /*magnetometer Y data*/
#define OUT_YH_M_REG_ADD                0x2B                                    /*magnetometer Y data*/
#define OUT_ZL_M_REG_ADD                0x2C                                    /*magnetometer Z data*/
#define OUT_ZH_M_REG_ADD                0x2D                                    /*magnetometer Z data*/

#define INT_CFG_M_REG_ADD               0x30                                    /*magnetometer interrupt register*/
#define INT_SRC_M_REG_ADD               0x31                                    /*magnetometer interrupt thershold*/
#define INT_THS_L_M_REG_ADD             0x32                                    /*magnetometer interrupt thershold value*/
#define INT_THS_H_M_REG_ADD             0x33

/* Sensitivity ---------------------------------------------------------------*/
// Accelerometer sensitivity [mg/LSB]
#define LSM9DS1_Acc_Sens_2              0.061   /*!< 2 g full scale */
#define LSM9DS1_Acc_Sens_4              0.122   /*!< 4 g full scale */
#define LSM9DS1_Acc_Sens_8              0.244   /*!< 8 g full scale */ 

// Gyroscope sensitivity [mdps/LSB]
#define LSM9DS1_Gyr_Sens_245            8.75    /*!< 245 dps full scale */
#define LSM9DS1_Gyr_Sens_500            17.50   /*!< 500 dps full scale */
#define LSM9DS1_Gyr_Sens_2000           70      /*!< 2000 dps full scale */ 

// Magnetometer sensitivity [mG/LSB]
#define LSM9DS1_Mag_Sens_4              0.14    /*!< 4 gauss full scale */
#define LSM9DS1_Mag_Sens_8              0.29    /*!< 8 gauss full scale */
#define LSM9DS1_Mag_Sens_12             0.43    /*!< 12 gauss full scale */ 
#define LSM9DS1_Mag_Sens_16             0.58    /*!< 16 gauss full scale */ 
 
/* ODR------------------------------------------------------------------------*/
#define LSM9DS1_A_ODR_10                ((u8)0x20)
#define LSM9DS1_A_ODR_50                ((u8)0x40)
#define LSM9DS1_A_ODR_119               ((u8)0x60)
#define LSM9DS1_A_ODR_238               ((u8)0x80)
#define LSM9DS1_A_ODR_476               ((u8)0xA0)
#define LSM9DS1_A_ODR_952               ((u8)0xC0)

#define LSM9DS1_G_ODR_14_9              ((u8)0x20)
#define LSM9DS1_G_ODR_59_5              ((u8)0x40)
#define LSM9DS1_G_ODR_119               ((u8)0x60)
#define LSM9DS1_G_ODR_238               ((u8)0x80)
#define LSM9DS1_G_ODR_476               ((u8)0xA0)
#define LSM9DS1_G_ODR_952               ((u8)0xC0)

#define LSM9DS1_M_ODR_0_625             ((u8)0x00)
#define LSM9DS1_M_ODR_1_25              ((u8)0x04)
#define LSM9DS1_M_ODR_2_5               ((u8)0x08)
#define LSM9DS1_M_ODR_5                 ((u8)0x0C)
#define LSM9DS1_M_ODR_10                ((u8)0x10)
#define LSM9DS1_M_ODR_20                ((u8)0x14)
#define LSM9DS1_M_ODR_40                ((u8)0x18)
#define LSM9DS1_M_ODR_80                ((u8)0x1C)

/* AXIS-----------------------------------------------------------------------*/
#define LSM9DS1_A_XEN                   ((u8)0x08)
#define LSM9DS1_A_YEN                   ((u8)0x10)
#define LSM9DS1_A_XYEN                  ((u8)0x18)
#define LSM9DS1_A_ZEN                   ((u8)0x20)
#define LSM9DS1_A_ZXEN                  ((u8)0x28)
#define LSM9DS1_A_ZYEN                  ((u8)0x30)
#define LSM9DS1_A_XYZEN                 ((u8)0x38)

#define LSM9DS1_G_XEN                   ((u8)0x08)
#define LSM9DS1_G_YEN                   ((u8)0x10)
#define LSM9DS1_G_XYEN                  ((u8)0x18)
#define LSM9DS1_G_ZEN                   ((u8)0x20)
#define LSM9DS1_G_ZXEN                  ((u8)0x28)
#define LSM9DS1_G_ZYEN                  ((u8)0x30)
#define LSM9DS1_G_XYZEN                 ((u8)0x38)

/* POWER-MODE-----------------------------------------------------------------*/
#define LSM9DS1_A_POWER_DOWN            ((u8)0x00)  /*ODR 00*/

#define LSM9DS1_G_POWER_DOWN            ((u8)0x00)  /*ODR 00*/
#define LSM9DS1_G_LOW_POWER_DIS         ((u8)0x00)  /*LP bit*/
#define LSM9DS1_G_LOW_POWER_EN          ((u8)0x80)  /*LP bit*/

#define LSM9DS1_M_CONTINUOUS            ((u8)0x00)  /*MD bits*/
#define LSM9DS1_M_SINGLE                ((u8)0x01)  /*MD bits*/
#define LSM9DS1_M_POWER_DOWN            ((u8)0x02)  /*MD bits*/

#define LSM9DS1_M_LOW_POWER_DIS         ((u8)0x00)  /*LP bit*/
#define LSM9DS1_M_LOW_POWER_EN          ((u8)0x20)  /*LP bit*/
#define LSM9DS1_M_LOW_POWER_XY          ((u8)0x00)  /*OM XY bits*/
#define LSM9DS1_M_MED_POWER_XY          ((u8)0x20)  /*OM XY bits*/
#define LSM9DS1_M_HIGH_POWER_XY         ((u8)0x40)  /*OM XY bits*/
#define LSM9DS1_M_ULTRA_POWER_XY        ((u8)0x60)  /*OM XY bits*/
#define LSM9DS1_M_LOW_POWER_Z           ((u8)0x00)  /*OM Z bits*/
#define LSM9DS1_M_MED_POWER_Z           ((u8)0x04)  /*OM Z bits*/
#define LSM9DS1_M_HIGH_POWER_Z          ((u8)0x08)  /*OM Z bits*/
#define LSM9DS1_M_ULTRA_POWER_Z         ((u8)0x0C)  /*OM Z bits*/

/* Full-scale-----------------------------------------------------------------*/
#define LSM9DS1_A_FS_2                  ((u8)0x00)
#define LSM9DS1_A_FS_4                  ((u8)0x10)
#define LSM9DS1_A_FS_8                  ((u8)0x18)

#define LSM9DS1_G_FS_245                ((u8)0x00)
#define LSM9DS1_G_FS_500                ((u8)0x08)
#define LSM9DS1_G_FS_2000               ((u8)0x18)

#define LSM9DS1_M_FS_4                  ((u8)0x00)
#define LSM9DS1_M_FS_8                  ((u8)0x20)
#define LSM9DS1_M_FS_12                 ((u8)0x40)
#define LSM9DS1_M_FS_16                 ((u8)0x60)

/* BLE------------------------------------------------------------------------*/
#define LSM9DS1_AG_Little_Endian        ((u8)0x00)
#define LSM9DS1_AG_Big_Endian           ((u8)0x02)
#define LSM9DS1_M_Little_Endian         ((u8)0x00)
#define LSM9DS1_M_Big_Endian            ((u8)0x02)

/* BDU------------------------------------------------------------------------*/
#define LSM9DS1_AG_BDU_Continuos        ((u8)0xC0)
#define LSM9DS1_M_BDU_Continuos         ((u8)0x00)

/*High-resolution-------------------------------------------------------------*/
#define LSM9DS1_A_HR_DISABLE            ((u8)0x00)            
#define LSM9DS1_A_HR_ENABLE             ((u8)0x80) 

/*----------------------------------------------------------------------------*/
#define LSM9DS1_AG_ADD_INC_ENABLE       ((u8)0x04)
#define LSM9DS1_AG_ADD_INC_DISABLE      ((u8)0x00)


// ____________________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________
typedef struct
{
  u8 ODR; 
  u8 Axes_Enable;   
  u8 FS;    
  u8 HR;
  float Sens;
}LSM9DS1_A_ConfigTypeDef;

typedef struct
{
  u8 ODR;  
  u8 Axes_Enable;
  u8 FS;    
  u8 Low_Power_Mode; 
  float Sens;
}LSM9DS1_G_ConfigTypeDef;

typedef struct
{   
  u8 Data_Update;  
  u8 Endianess;
  u8 Address_Inc;
}LSM9DS1_AG_ConfigTypeDef;

typedef struct
{
  u8 Low_Power_Mode;
  u8 Mode;
  u8 ODR;     
  u8 FS;        
  u8 XY_Mode;  
  u8 Z_Mode;
  u8 Data_Update;  
  u8 Endianess;
  float Sens;
}LSM9DS1_M_ConfigTypeDef;

typedef struct 
{
  s16 sGyrAcc[6];
  s16 sAcc[3];
  s16 sGyr[3];
  s16 sMag[3];
  float fAcc[3];
  float fGyr[3];
  float m_fMag[3];
  double m_fEuler[3];
} LSM_DATA;

// Functions __________________________________________________________________
void LSM_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite);
void LSM_I2C_DMA_BufferRead(u8 slAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void LSM_I2C_BufferRead(u8 slAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void LSM9DS1_A_Config(LSM9DS1_A_ConfigTypeDef *LSM_A_Config_Struct);
void LSM9DS1_G_Config(LSM9DS1_G_ConfigTypeDef *LSM_G_Config_Struct);
void LSM9DS1_AG_Config(LSM9DS1_AG_ConfigTypeDef *LSM_AG_Config_Struct);
void LSM9DS1_M_Config(LSM9DS1_M_ConfigTypeDef *LSM_M_Config_Struct);
void LSM9DS1_A_Read_RawData(s16* out);
void LSM9DS1_G_Read_RawData(s16* out);
void LSM9DS1_M_Read_RawData(s16* out);
void LSM9DS1_GA_Read_RawData(s16* out);
void LSM9DS1_Magn_Config(u8 magn_en);
void LSM_DataProcess(LSM_DATA *mData);

#endif /* __LSM9DS1_H */


