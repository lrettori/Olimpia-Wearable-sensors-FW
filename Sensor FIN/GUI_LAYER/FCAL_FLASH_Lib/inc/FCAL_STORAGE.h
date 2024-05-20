#ifndef __FACL_STORAGE_H
#define __FACL_STORAGE_H

#include <stdint.h>
#include "stm32f10x_flash.h"
#include "GUI_LAYER.h"
#include "stm32f10x_can.h"
#include "string.h"


#define FLASH_ADD_CAL_TYPE    0x0807F000
#define FLASH_ADD_ACC         0x0807E000
#define FLASH_ADD_GYRO        0x0807E800
#define FLASH_ADD_MAG         0x0807D800      
#define FACTORY_CAL           0
#define NEW_CAL               1 

void ACC_FACTORY_CAL(void);
void GYRO_FACTORY_CAL(void);
void MAGN_FACTORY_CAL(void);
void FLASH_CAL_COEFF(void);
void MAG_Cal_Storage(uint8_t coeff_cal[12]);
void MAG_FLASH_READ (void);
void ACC_Cal_Storage(uint8_t coeff_cal[24]);
void ACC_FLASH_READ (void);
void GYRO_Cal_Storage(uint8_t coeff_cal[6]);
void GYRO_FLASH_READ (void);
void GYRO_NEW_CAL(CanRxMsg CAN_RX);
int ACC_NEW_CAL(CanRxMsg CAN_RX, int index);
int MAG_NEW_CAL(CanRxMsg CAN_RX, int index);
void IMU_CALIBRATION(float Acc[3],float Gyr[3],float Mag[3], uint8_t CAL_MODE);
void GYRO_Cal_fingers(u8 coeff_cal_t[6],u8 coeff_cal_i[6],u8 coeff_cal_m[6]);
void ACC_Cal_fingers(u8 coeff_cal_t[24],u8 coeff_cal_i[24],u8 coeff_cal_m[24]);
void MAG_Cal_fingers(u8 coeff_cal_t[12],u8 coeff_cal_i[12],u8 coeff_cal_m[12]);

#endif