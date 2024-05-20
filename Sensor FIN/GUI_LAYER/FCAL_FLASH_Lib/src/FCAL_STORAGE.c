/* HANDi_FINV1.0
  ************************************************************************************
  * @file    FCAL_STORAGE.c
  * @author  Dario Esposito
  * @version V1.0.0
  * @date    15/12/2016
  * @brief   This file provides the lines code fo FLASH memory reading\writing for
  *          device calibration
  ************************************************************************************
  * 
  *  
  * 
  ************************************************************************************/
#include "FCAL_STORAGE.h"
uint16_t CAL_TYPE;
uint16_t g_buf_cal[3],a_buf_cal[12],m_buf_cal[6] ;
uint16_t gyr_cal[3],acc_cal[12],mag_cal[6];
uint8_t G_CAL_COEFF=0;
uint8_t A_CAL_COEFF=0;
uint8_t M_CAL_COEFF=0;
u8 acc_cal_coeff[24];
u8 mag_cal_coeff[12];
float  GYR_COEF[3],ACC_COEF[12],MAG_COEF[6];
FLASH_Status STATUS;
/*------------------------------IMU_CAL_FACTORY-------------------------------*/
 float accx_off=0;
 float accy_off= 0;
 float accz_off=0;
 float acc11= 1;
 float acc12=0;
 float acc13=0;
 float acc21=0;
 float acc22=1;
 float acc23=0;
 float acc31=0;
 float acc32=0;
 float acc33= 1;
 float gyrx_off= 0;
 float gyry_off= 0;
 float gyrz_off= 0;
 float M_OSx= 0.0;
 float M_OSy= 0.0;
 float M_OSz= 0.0;
 float M_SCx= 1;
 float M_SCy= 1;
 float M_SCz=  1;
/*----------------------------------------------------------------------------*/
void ACC_FACTORY_CAL(void)
{
   /*
  F=factory(raw_data)
  N=new cal
  |cal_type|gyr|acc|mag|
  |    0   | F | F | F |
  |    1   | N | F | F |
  |    2   | F | N | F |
  |    3   | N | N | F |
  |    4   | F | F | N |
  |    5   | N | F | N |
  |    6   | F | N | N |
  |    7   | N | N | N |
  */
    memcpy(&CAL_TYPE,(char *)(FLASH_ADD_CAL_TYPE), 2);
    FLASH_Unlock(); 
    if((CAL_TYPE==0)||(CAL_TYPE==2)||(CAL_TYPE>7))
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 0);
    }
    else if(CAL_TYPE==3)
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 1); 
    }
    else if(CAL_TYPE==6)
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 4);
    }
    else if(CAL_TYPE==7)
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 5);
    }
    FLASH_Lock(); 
    NVIC_SystemReset();
}
/*----------------------------------------------------------------------------*/
void GYRO_FACTORY_CAL(void)
{
   /*
  F=factory(raw_data)
  N=new cal
  |cal_type|gyr|acc|mag|
  |    0   | F | F | F |
  |    1   | N | F | F |
  |    2   | F | N | F |
  |    3   | N | N | F |
  |    4   | F | F | N |
  |    5   | N | F | N |
  |    6   | F | N | N |
  |    7   | N | N | N |
  */
    memcpy(&CAL_TYPE,(char *)(FLASH_ADD_CAL_TYPE), 2);
    FLASH_Unlock(); 
    if((CAL_TYPE==0)||(CAL_TYPE==1)||(CAL_TYPE>7))
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 0);
    }
    else if(CAL_TYPE==3)
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 2); 
    }
    else if(CAL_TYPE==5)
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 4);
    }
    else if(CAL_TYPE==7)
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 6);
    }
    FLASH_Lock(); 
    NVIC_SystemReset();
}
/*----------------------------------------------------------------------------*/
void MAGN_FACTORY_CAL(void)
{
   /*
  F=factory(raw_data)
  N=new cal
  |cal_type|gyr|acc|mag|
  |    0   | F | F | F |
  |    1   | N | F | F |
  |    2   | F | N | F |
  |    3   | N | N | F |
  |    4   | F | F | N |
  |    5   | N | F | N |
  |    6   | F | N | N |
  |    7   | N | N | N |
  */
    memcpy(&CAL_TYPE,(char *)(FLASH_ADD_CAL_TYPE), 2);
    FLASH_Unlock(); 
    if((CAL_TYPE==0)||(CAL_TYPE==4)||(CAL_TYPE>7))
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 0);
    }
    else if(CAL_TYPE==5)
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 1); 
    }
    else if(CAL_TYPE==6)
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 2);
    }
    else if(CAL_TYPE==7)
    {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 3);
    }
    FLASH_Lock(); 
    NVIC_SystemReset();
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*------------------------------FLASH R_W-------------------------------------*/
/*FLASH:
|    AD     |  PAGE  |halfword1|halfword2|halfword3|halfword4|halfword5|halfword6|halfword7|halfword8|halfword9|halfword10|halfword11|halfword12|
| 0x0801F7D0|  125   | acc11   |  acc12  |  acc13  |  acc21  |  acc22  |  acc23  |  acc31  |  acc32  |  acc33  | accx_off | accy_off | accz_off |
| 0x0801FBB8|  126   |cal_type |         |         |         |         |         |         |         |         |          |          |          |
| 0x0801FFA0|  127   |gyrx_off |gyry_off |gyrz_off |         |         |         |         |         |         |          |          |          |
*/
/*----------------------------------------------------------------------------*/
void GYRO_FLASH_READ (void)
{
  int n=0;
  for(int  i=0;i<5;i+=2)
  {
   memcpy(&g_buf_cal[n],(char *)(FLASH_ADD_GYRO+i), 2);
   GYR_COEF[n]=((float)((s16)(g_buf_cal[n])))/1000;
   n++;
  } 
}
/*----------------------------------------------------------------------------*/
void GYRO_Cal_Storage(u8 coeff_cal[6])
{
  for(int i=0;i<3;i++)
   gyr_cal[i]=((uint16_t)(coeff_cal[2*i+1]<<8)+coeff_cal[2*i]);
  /*----------------FLASH_STORAGE--------------*/
  memcpy(&CAL_TYPE,(char *)(FLASH_ADD_CAL_TYPE), 2);
  FLASH_Unlock(); 
  int n=0;
      /*
  F=factory(raw_data)
  N=new cal
  |cal_type|gyr|acc|mag|
  |    0   | F | F | F |
  |    1   | N | F | F |
  |    2   | F | N | F |
  |    3   | N | N | F |
  |    4   | F | F | N |
  |    5   | N | F | N |
  |    6   | F | N | N |
  |    7   | N | N | N |
  */
  if((CAL_TYPE==0)||(CAL_TYPE==1)||(CAL_TYPE>7))
  {
    if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 1);
  }
  else if((CAL_TYPE==2)||(CAL_TYPE==3))
  {
    if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 3);
  }
  else if((CAL_TYPE==4)||(CAL_TYPE==5))
  {
    if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 5);
  }
  else if((CAL_TYPE==6)||(CAL_TYPE==7))
  {
    if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 7);
  }

  if (FLASH_ErasePage(FLASH_ADD_GYRO) == FLASH_COMPLETE)
  {
    for(int  i=0;i<5;i+=2)
    {
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_GYRO+i, gyr_cal[n]);
      n++;
      if(STATUS==!FLASH_COMPLETE)
        while(1){}
    }
  }
  n=0;
  FLASH_Lock();
  NVIC_SystemReset();
}
/*----------------------------------------------------------------------------*/
void ACC_FLASH_READ (void)
{
  int n=0;
  for(int  i=0;i<23;i+=2)
  {
   memcpy(&a_buf_cal[n],(char *)(FLASH_ADD_ACC+i), 2);
   ACC_COEF[n]=((float)((s16)(a_buf_cal[n])))/100;
   n++;
  } 
}
/*----------------------------------------------------------------------------*/
void ACC_Cal_Storage(u8 coeff_cal[24])
{
  for(int i=0;i<12;i++)
   acc_cal[i]=((uint16_t)(coeff_cal[2*i+1]<<8)+coeff_cal[2*i]);
  memcpy(&CAL_TYPE,(char *)(FLASH_ADD_CAL_TYPE), 2);
  /*----------------FLASH_STORAGE--------------*/
  FLASH_Unlock(); 
  int n=0;
      /*
  F=factory(raw_data)
  N=new cal
  |cal_type|gyr|acc|mag|
  |    0   | F | F | F |
  |    1   | N | F | F |
  |    2   | F | N | F |
  |    3   | N | N | F |
  |    4   | F | F | N |
  |    5   | N | F | N |
  |    6   | F | N | N |
  |    7   | N | N | N |
  */
  if((CAL_TYPE==0)||(CAL_TYPE==2)||(CAL_TYPE>7))
  {
    if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 2);/*acc new cal*/
  }
  else if((CAL_TYPE==1)||(CAL_TYPE==3))
  {
    if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 3);
  }
  else if((CAL_TYPE==4)||(CAL_TYPE==6))
  {
    if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 6);
  }
  else if((CAL_TYPE==5)||(CAL_TYPE==7))
  {
    if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 7);
  }

  if (FLASH_ErasePage(FLASH_ADD_ACC) == FLASH_COMPLETE)
  {
    for(int  i=0;i<23;i+=2)
    {
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_ACC+i, acc_cal[n]);
      n++;
      if(STATUS==!FLASH_COMPLETE)
        while(1){}
    }
  }
  n=0;
  FLASH_Lock();
  NVIC_SystemReset();
}
/*----------------------------------------------------------------------------*/
void MAG_FLASH_READ (void)
{
  int n=0;
  for(int  i=0;i<12;i+=2)
  {
   memcpy(&m_buf_cal[n],(char *)(FLASH_ADD_MAG+i), 2);
   MAG_COEF[n]=((float)((s16)(m_buf_cal[n])))/100;
   n++;
  } 
}
/*----------------------------------------------------------------------------*/
void MAG_Cal_Storage(u8 coeff_cal[12])
{
  for(int i=0;i<6;i++)
   mag_cal[i]=((uint16_t)(coeff_cal[2*i+1]<<8)+coeff_cal[2*i]);
  memcpy(&CAL_TYPE,(char *)(FLASH_ADD_CAL_TYPE), 2);
  /*----------------FLASH_STORAGE--------------*/
  FLASH_Unlock(); 
  int n=0; 
      /*
  F=factory(raw_data)
  N=new cal
  |cal_type|gyr|acc|mag|
  |    0   | F | F | F |
  |    1   | N | F | F |
  |    2   | F | N | F |
  |    3   | N | N | F |
  |    4   | F | F | N |
  |    5   | N | F | N |
  |    6   | F | N | N |
  |    7   | N | N | N |
  */
   if((CAL_TYPE==0)||(CAL_TYPE==4)||(CAL_TYPE>7))
   {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 4);
   }
   else if((CAL_TYPE==1)||(CAL_TYPE==5))
   {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 5);
   }
   else if((CAL_TYPE==2)||(CAL_TYPE==6))
   {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 6);
   }
   else if((CAL_TYPE==3)||(CAL_TYPE==7))
   {
      if (FLASH_ErasePage(FLASH_ADD_CAL_TYPE) == FLASH_COMPLETE)
        STATUS=FLASH_ProgramHalfWord(FLASH_ADD_CAL_TYPE, 7);
   }

  if (FLASH_ErasePage(FLASH_ADD_MAG) == FLASH_COMPLETE)
  {
    for(int  i=0;i<12;i+=2)
    {
      STATUS=FLASH_ProgramHalfWord(FLASH_ADD_MAG+i, mag_cal[n]);
      n++;
      if(STATUS==!FLASH_COMPLETE)
        while(1){}
    }
  }
  n=0;
  FLASH_Lock();
  NVIC_SystemReset();
}
/*----------------------------------------------------------------------------*/
void GYRO_NEW_CAL(CanRxMsg CAN_RX)
{
  u8 gyro_cal_coeff[6];
  for(int i=0;i<6;i++)
    gyro_cal_coeff[i]=CAN_RX.Data[i];
  G_CAL_COEFF=NEW_CAL;
  GYRO_Cal_Storage(gyro_cal_coeff);
}
/*----------------------------------------------------------------------------*/
int ACC_NEW_CAL(CanRxMsg CAN_RX, int index)
{
  
    int n=0; int i=0;
    for(i=index;i<index+8;i++){acc_cal_coeff[i] = CAN_RX.Data[n];n++;}
    index=i;
    if(index==24){A_CAL_COEFF=NEW_CAL; ACC_Cal_Storage(acc_cal_coeff);}
      return index;
}
/*----------------------------------------------------------------------------*/
int MAG_NEW_CAL(CanRxMsg CAN_RX, int index)
{
    int n=0; int i=0; 
    for(i=index;i<index+6;i++){mag_cal_coeff[i]=CAN_RX.Data[n];n++;}
    index=i;
    if(index==12){M_CAL_COEFF=NEW_CAL; MAG_Cal_Storage(mag_cal_coeff);}
      return index;
}
/*----------------------------------------------------------------------------*/
void FLASH_CAL_COEFF(void)
{
  /*
  F=factory(raw_data)
  N=new cal
  |cal_type|gyr|acc|mag|
  |    0   | F | F | F |
  |    1   | N | F | F |
  |    2   | F | N | F |
  |    3   | N | N | F |
  |    4   | F | F | N |
  |    5   | N | F | N |
  |    6   | F | N | N |
  |    7   | N | N | N |
  */
  memcpy(&CAL_TYPE,(char *)(FLASH_ADD_CAL_TYPE), 2);
  if(CAL_TYPE>7)
    CAL_TYPE=0;
  switch(CAL_TYPE)
  {
     case 0:
     G_CAL_COEFF=FACTORY_CAL; A_CAL_COEFF=FACTORY_CAL; M_CAL_COEFF=FACTORY_CAL;
     break;
     case 1:
     G_CAL_COEFF=NEW_CAL; A_CAL_COEFF=FACTORY_CAL; M_CAL_COEFF=FACTORY_CAL;
     GYRO_FLASH_READ();
     break;
     case 2:
     G_CAL_COEFF=FACTORY_CAL; A_CAL_COEFF=NEW_CAL; M_CAL_COEFF=FACTORY_CAL;
     ACC_FLASH_READ();
     break;
     case 3:
     G_CAL_COEFF=NEW_CAL; A_CAL_COEFF=NEW_CAL; M_CAL_COEFF=FACTORY_CAL;
     GYRO_FLASH_READ(); ACC_FLASH_READ();
     break;
     case 4:
     G_CAL_COEFF=FACTORY_CAL; A_CAL_COEFF=FACTORY_CAL; M_CAL_COEFF=NEW_CAL;
     MAG_FLASH_READ();
     break;
     case 5:
     G_CAL_COEFF=NEW_CAL; A_CAL_COEFF=FACTORY_CAL; M_CAL_COEFF=NEW_CAL;
     GYRO_FLASH_READ(); MAG_FLASH_READ();
     break;
     case 6:
     G_CAL_COEFF=FACTORY_CAL; A_CAL_COEFF=NEW_CAL; M_CAL_COEFF=NEW_CAL;
     ACC_FLASH_READ(); MAG_FLASH_READ();
     break;
     case 7:
     G_CAL_COEFF=NEW_CAL; A_CAL_COEFF=NEW_CAL; M_CAL_COEFF=NEW_CAL;
     GYRO_FLASH_READ(); ACC_FLASH_READ(); MAG_FLASH_READ();
     break;  
  }
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-----------------------------IMU_CALIBRATION--------------------------------*/
void IMU_CALIBRATION(float Acc[3],float Gyr[3],float Mag[3], u8 CAL_MODE)
{
    if(A_CAL_COEFF==FACTORY_CAL)
    {
      Acc[0]= ((acc11*Acc[0])+(acc12*Acc[1])+(acc13*Acc[2])+accx_off);
      Acc[1]= ((acc21*Acc[0])+(acc22*Acc[1])+(acc23*Acc[2])+accy_off);
      Acc[2]= ((acc31*Acc[0])+(acc32*Acc[1])+(acc33*Acc[2])+accz_off);
    }
    else if(A_CAL_COEFF==NEW_CAL)
    {
      Acc[0]= ((ACC_COEF[0]*Acc[0])+(ACC_COEF[1]*Acc[1])+(ACC_COEF[2]*Acc[2])+ACC_COEF[9]);
      Acc[1]= ((ACC_COEF[3]*Acc[0])+(ACC_COEF[4]*Acc[1])+(ACC_COEF[5]*Acc[2])+ACC_COEF[10]);
      Acc[2]= ((ACC_COEF[6]*Acc[0])+(ACC_COEF[7]*Acc[1])+(ACC_COEF[8]*Acc[2])+ACC_COEF[11]);
    }
    if(G_CAL_COEFF==FACTORY_CAL)
    {
      Gyr[0]= Gyr[0]-gyrx_off;
      Gyr[1]= Gyr[1]-gyry_off; 
      Gyr[2]= Gyr[2]-gyrz_off; 
    }
    else if(G_CAL_COEFF==NEW_CAL)
    {
      for(int i=0;i<3;i++)
        Gyr[i]= Gyr[i]-GYR_COEF[i];
    }
    if(CAL_MODE!=2)
    {
      if(M_CAL_COEFF==FACTORY_CAL)
      {
        Mag[0]=(Mag[0]-M_OSx)/M_SCx;
        Mag[1]=(Mag[1]-M_OSy)/M_SCy; 
        Mag[2]=(Mag[2]-M_OSz)/M_SCz;
      }
      else if(M_CAL_COEFF==NEW_CAL)
      {
        Mag[0]= (Mag[0]-MAG_COEF[0])*MAG_COEF[3];
        Mag[1]= (Mag[1]-MAG_COEF[1])*MAG_COEF[4];  
        Mag[2]= (Mag[2]-MAG_COEF[2])*MAG_COEF[5];
      }
    }
}
/*----------------------------------------------------------------------------*/