/**
  * @file    HAL_LSM9DS1.h
  * @author  Dario Esposito
  * @version V1.0.0
  * @date    14 December 2016
  * @brief   Hardware Abstraction Layer for LSM9DS1.
  */

#ifndef __HAL_LSM9DS1_H
#define __HAL_LSM9DS1_H

#include "stm32f10x.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define LSM_I2C                        I2C2
#define LSM_I2C_RCC_Periph             RCC_APB1Periph_I2C2
#define LSM_I2C_Port                   GPIOB
#define LSM_I2C_SCL_Pin                GPIO_Pin_10
#define LSM_I2C_SDA_Pin                GPIO_Pin_11
#define LSM_I2C_RCC_Port               RCC_APB2Periph_GPIOB
#define LSM_I2C_Speed                  400000


#define LSM9DS1_AG_INT1_Pin           GPIO_Pin_6
#define LSM9DS1_AG_INT1_Port          GPIOB
#define LSM9DS1_AG_INT1_RCC_Port      RCC_APB2Periph_GPIOB

#define LSM9DS1_AG_INT2_Pin           GPIO_Pin_7
#define LSM9DS1_AG_INT2_Port          GPIOB
#define LSM9DS1_AG_INT2_RCC_Port      RCC_APB2Periph_GPIOB

#define LSM9DS1_M_INT1_Pin             GPIO_Pin_4
#define LSM9DS1_M_INT1_Port            GPIOB
#define LSM9DS1_M_INT1_RCC_Port        RCC_APB2Periph_GPIOB
   
#define LSM9DS1_DRDY_M_Pin             GPIO_Pin_13
#define LSM9DS1_DRDY_M_Port            GPIOB
#define LSM9DS1_DRDY_M_RCC_Port        RCC_APB2Periph_GPIOB   

#define LSM9DS1_AG_DEN_Pin            GPIO_Pin_5
#define LSM9DS1_AG_DEN_Port           GPIOB
#define LSM9DS1_AG_DEN_RCC_Port       RCC_APB2Periph_GPIOB  
   
#ifdef __cplusplus
}
#endif

#endif /* __HAL_LSM9DS1_H */
