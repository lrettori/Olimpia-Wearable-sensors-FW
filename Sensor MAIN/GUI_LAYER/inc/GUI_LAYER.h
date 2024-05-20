
#include <stdint.h>
#include "includes.h"
#ifndef __GUI_LAYER_H
#define __GUI_LAYER_H

#ifdef __cplusplus
 extern "C" {
#endif 
  
#include "LSM9DS1.h"

void LSM_DataProcess(LSM_DATA *mData);
void Delay(int time);
#endif 
/***************************END OF FILE****************************************/



