/* HANDi_FINV1.0
  ************************************************************************************
  * @file    EKF_Lib.c
  * @author  Dario Esposito
  * @version V1.0.0
  * @date    15/12/2016
  * @brief   This file provides the lines code of a low power Kalaman Filter library
  ************************************************************************************
  * 
  *  
  * 
  ************************************************************************************/
#include "EKF_Lib.h"
#include "DeviceSettings.h"

double P_plus[4][4]={{0.1,0,0,0},{0,0.1,0,0},{0,0.0,0.1,0},{0,0,0,0.1}};
double P_minus[4][4];

#if defined(Q1)/*thumb*/
double Q[4][4]={{0.000118210663817081	,-0.000102603855840685	,9.38060549358056e-05	,-0.000109412862912202}, {-0.000102603855840685,	0.000118210663817081,	-0.000109412862912202,	9.38060549358056e-05},
{9.38060549358056e-05,	-0.000109412862912202,	0.000118210663817081,	-0.000102603855840685},{-0.000109412862912202,	9.38060549358056e-05,	-0.000102603855840685,	0.000118210663817081}};
#elif defined(Q2)/*index*/
double Q[4][4]={{0.000383020893363516,	-0.000146156914600104,	-0.000171534837305021,	-6.53291414583908e-05}, {-0.000146156914600104,	0.000383020893363516,	-6.53291414583908e-05,	-0.000171534837305021},
{-0.000171534837305021,	-6.53291414583908e-05	,0.000383020893363516	,-0.000146156914600104},{-6.53291414583908e-05,	-0.000171534837305021,-0.000146156914600104	,0.000383020893363516}};
#elif defined(Q3)/*middle*/
double Q[4][4]={{8.65369909139683e-05	,-6.08436616610496e-05	,-4.25395266241120e-05	,1.68461973711933e-05}, {-6.08436616610496e-05	,8.65369909139683e-05,	1.68461973711933e-05,	-4.25395266241120e-05},
{-4.25395266241120e-05,	1.68461973711933e-05	,8.65369909139683e-05	,-6.08436616610496e-05},{1.68461973711933e-05	,-4.25395266241120e-05,	-6.08436616610496e-05	,8.65369909139683e-05}};
#endif

double Ak[4][4]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
double R1[3][3]={{1e-004,0,0},{0,1e-004,0},{0,0,1e-004}};
double I[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
double q_minus[4],r1[4][4],r2[4][4],Ak_T[4][4],H1[3][4],H_T[4][3],op1[4][3],op2[3][4],op3[3][3], op4[3][3],K[4][3],h1[3],op5[3],e[4],
op6[4][4],op7[4][4],err;
double a,b,c,d;
double Rot_Matrix[4][4]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
double D=0;
double coff[3][3];
double coff_T[3][3];
double Inv[3][3]={{0,0,0},{0,0,0},{0,0,0}};
//double q_plus[4]={0.5,0.5,0.5,0.5};
/*-----------------------------------------------------------------*/
double det(double m[MAX_C][MAX_C], int car)
{
    double determinante = 0;
    //Cardinalità uno
    if (car == 1) determinante = m[0][0];
    //Cardinalità due
    if (car == 2)
        determinante = m[1][1]*m[0][0]-m[0][1]*m[1][0];
    //Cardinalità > 2
    else {
        for (int row = 0; row < car; row++) {
            double sub_m[MAX_C][MAX_C];
            //Sottomatrice di ordine car-1
            for (int i = 0; i < car-1; i++) {
                for (int j = 0; j < car-1; j++) {
                    int sub_row = (i < row ? i : i+1);
                    int sub_col = j+1;
                    sub_m[i][j] = m[sub_row][sub_col];
                }
            }
            //Segno sottomatrice + per pari, - per dispari
            if (row % 2 == 0)
             determinante += m[row][0]*det(sub_m, car-1);
            else
             determinante -= m[row][0]*det(sub_m, car-1);
        }
    }
    return determinante;
}
/*----------------------------------------------------------------------------*/
void coff_inv(double m[MAX_C][MAX_C], int car )
{
        int row=0;
        int col=0;
        int num=car;
        for ( col = 0; col < car; col++)
        {
             for ( row = 0; row < car; row++)
            {
              double sub_m[MAX_C][MAX_C];
              //Sottomatrice di ordine car-1
              for (int i = 0; i < car-1; i++) 
              {
                  for (int j = 0; j < car-1; j++)
                  {
                      int sub_row = (i < row ? i : i+1);
                      int sub_col = (j < col ? j : j+1);
                      sub_m[i][j] = m[sub_row][sub_col];
                  }
              }
              if((row+col)%2==0)
              {
              coff[row][col]= det(sub_m,car-1);
              }
              else
              {
               coff[row][col]= (-1)*(det(sub_m,car-1));
              }
          }
       }     
       for(int j=0;j<num;j++)
       {
         for(int i=0;i<num;i++)
         {
           coff_T[i][j]=coff[j][i];/*coff(T)*/
         }
       }
}
/*----------------------------------------------------------------------------*/
void Step1(double q_plus[4])/*aggiornamento dello stato: qk(-)=Ak*(qk-1(+))*/
{
  for(int j=0;j<4;j++)
  {
    a=Ak[j][0]*q_plus[0];
    b=Ak[j][1]*q_plus[1];
    c=Ak[j][2]*q_plus[2];
    d=Ak[j][3]*q_plus[3];
    q_minus[j]=a+b+c+d;
  }
}
/*----------------------------------------------------------------------------*/
void Step2(void)/*Aggiornamento della varianza d'errore sullo stato P(-): Pk(-)=[Ak*Pk-1(+)*Ak(T)]+Qk-1*/
{
 for(int j=0;j<4;j++)
 {
     for(int i=0;i<4;i++)
     {
       a=Ak[i][0]*P_plus[0][j];
       b=Ak[i][1]*P_plus[1][j];
       c=Ak[i][2]*P_plus[2][j];
       d=Ak[i][3]*P_plus[3][j];
       r1[i][j]=a+b+c+d; /*Ak*Pk-1(+)*/
     }
 }
 for(int j=0;j<4;j++)
 {
   for(int i=0;i<4;i++)
   {
     Ak_T[i][j]=Ak[j][i];/*Trans_matrix(T)*/
   }
 }
 for(int j=0;j<4;j++)
 {
     for(int i=0;i<4;i++)
     {
       a=r1[i][0]*Ak_T[0][j];
       b=r1[i][1]*Ak_T[1][j];
       c=r1[i][2]*Ak_T[2][j];
       d=r1[i][3]*Ak_T[3][j];
       r2[i][j]=a+b+c+d; /*[Ak*Pk-1(+)*Ak(T)]*/
     }
 }
  for(int j=0;j<4;j++)
 {
   for(int i=0;i<4;i++)
     {
       P_minus[i][j]=r2[i][j]+Q[i][j];/*Pk(-)=[Ak*Pk-1(+)*Ak(T)]+Qk-1*/
     }
 }
}
/*----------------------------------------------------------------------------*/
void Step3(double H[3][4],double R[3][3])/*Calcolo del guadagno per accelerometro/magnetometro:K=Pk(-)*H(T)*[1/((H*Pk(-)*H(T))+R)]*/
{

 for(int j=0;j<3;j++)
 {
   for(int i=0;i<4;i++)
   {
     H_T[i][j]=H[j][i];/*H(T)*/
   }
 }
 for(int j=0;j<3;j++)
 {
   for(int i=0;i<4;i++)
   {
    a=P_minus[i][0]* H_T[0][j];
    b=P_minus[i][1]* H_T[1][j];
    c=P_minus[i][2]* H_T[2][j];
    d=P_minus[i][3]* H_T[3][j];
    op1[i][j]=a+b+c+d;/*Pk(-)*H(T)*/
   }
 }
 for(int j=0;j<4;j++)
 {
   for(int i=0;i<3;i++)
   {
     a=H[i][0]*P_minus[0][j];
     b=H[i][1]*P_minus[1][j];
     c=H[i][2]*P_minus[2][j];
     d=H[i][3]*P_minus[3][j];
     op2[i][j]=a+b+c+d;/*H*Pk(-)*/
   }
 }
 for(int j=0;j<3;j++)
 { 
   for(int i=0;i<3;i++)
   {
     a=op2[i][0]*H_T[0][j];
     b=op2[i][1]*H_T[1][j];
     c=op2[i][2]*H_T[2][j];
     d=op2[i][3]*H_T[3][j];
     op3[i][j]=a+b+c+d;/*H*Pk(-)*H(T)*/
   }
 }
 for(int j=0;j<3;j++)
 {
   for(int i=0;i<3;i++)
   {
     op4[i][j]=op3[i][j]+R[i][j];/*(H*Pk(-)*H(T))+R*/
   }
 }
 D=det(op4,3);
 coff_inv(op4, 3);
 for(int j=0;j<3;j++)
 {
     for(int i=0;i<3;i++)
     {
       Inv[i][j]=coff_T[i][j]/D; /*inversa: [1/((H*Pk(-)*H(T))+R)]*/
     }
 }
 for(int j=0;j<3;j++)
 {
   for(int i=0;i<4;i++)
   {
     a=op1[i][0]*Inv[0][j];
     b=op1[i][1]*Inv[1][j];
     c=op1[i][2]*Inv[2][j];
     K[i][j]=a+b+c;/*K=Pk(-)*H(T)*[1/((H*Pk(-)*H(T))+R)]*/
   }
 }
}
/*----------------------------------------------------------------------------*/
void Step4(float in_sensor[3],double h[3],double q_plus[4])/*prima correzione mediante accelerometro: qk(+)=qk(-)+K[zk-h] con ek=K[zk-h]*/
{
  for(int j=0;j<3;j++)
  {
    op5[j]=in_sensor[j]-h[j];/*zk-h*/
  }
  for(int j=0;j<4;j++)
  {
    a=K[j][0]*op5[0];
    b=K[j][1]*op5[1];  
    c=K[j][2]*op5[2];
    e[j]=a+b+c;/*ek=K[zk-h]*/
  }  
  for(int j=0;j<4;j++)
  {
    q_plus[j]=q_minus[j]+e[j];/*qk(+)=qk(-)+e*/
  }
}
/*----------------------------------------------------------------------------*/
void Step5(double H[3][4])/*Aggiornamento della varianza sull'errore di stato per accelerometro: Pk(+)=[I-K*H]Pk(-)*/
{
  for(int j=0;j<4;j++)
  {
    for(int i=0;i<4;i++)
    {
      a= K[i][0]*H[0][j];
      b= K[i][1]*H[1][j];
      c= K[i][2]*H[2][j];
      op6[i][j]=a+b+c;/*K*H*/
    }
  }
   for(int j=0;j<4;j++)
  {
    for(int i=0;i<4;i++)
    {
      op7[i][j]=I[i][j]-op6[i][j];/*[I-K*H]*/
    }
  }
    for(int j=0;j<4;j++)
  {
    for(int i=0;i<4;i++)
    {
      a=op7[i][0]*P_minus[0][j];
      b=op7[i][1]*P_minus[1][j];
      c=op7[i][2]*P_minus[2][j];
      d=op7[i][3]*P_minus[3][j];
      P_plus[i][j]=a+b+c+d;/*Pk(+)=[I-K*H]Pk(-)*/
    }
  }
}
/*----------------------------------------------------------------------------*/
void EKF(float in_acc[3], float in_gyr[3],double q_plus[4])
{
  Rot_Matrix[0][0]=   0.0;
  Rot_Matrix[0][1]= (-in_gyr[0])*0.5*T;
  Rot_Matrix[0][2]= (-in_gyr[1])*0.5*T;
  Rot_Matrix[0][3]= (-in_gyr[2])*0.5*T;
  Rot_Matrix[1][0]=   in_gyr[0] *0.5*T;
  Rot_Matrix[1][1]=   0.0;
  Rot_Matrix[1][2]=   in_gyr[2] *0.5*T;
  Rot_Matrix[1][3]= (-in_gyr[1])*0.5*T;
  Rot_Matrix[2][0]=   in_gyr[1] *0.5*T;
  Rot_Matrix[2][1]= (-in_gyr[2])*0.5*T;
  Rot_Matrix[2][2]=   0.0;
  Rot_Matrix[2][3]=   in_gyr[0] *0.5*T;
  Rot_Matrix[3][0]=   in_gyr[2] *0.5*T;
  Rot_Matrix[3][1]=   in_gyr[1] *0.5*T;
  Rot_Matrix[3][2]= (-in_gyr[0])*0.5*T;
  Rot_Matrix[3][3]=   0.0;
      
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<4;j++)
      Ak[i][j]=I[i][j]+Rot_Matrix[i][j];/*matrice di transizione*/
  }
  Step1(q_plus);
  Step2();
  H1[0][0]= -2*q_minus[2];
  H1[0][1]=  2*q_minus[3];
  H1[0][2]= -2*q_minus[0];
  H1[0][3]=  2*q_minus[1];
  H1[1][0]=  2*q_minus[1];
  H1[1][1]=  2*q_minus[0];
  H1[1][2]=  2*q_minus[3];
  H1[1][3]=  2*q_minus[2];
  H1[2][0]=  2*q_minus[0];
  H1[2][1]= -2*q_minus[1];
  H1[2][2]= -2*q_minus[2];
  H1[2][3]=  2*q_minus[3];
  Step3(H1,R1);
  h1[0]=2*(q_minus[1]*q_minus[3]-q_minus[0]*q_minus[2]);
  h1[1]=2*(q_minus[0]*q_minus[1]+q_minus[2]*q_minus[3]);
  h1[2]=(q_minus[0]*q_minus[0]-q_minus[1]*q_minus[1]-q_minus[2]*q_minus[2]+q_minus[3]*q_minus[3]);
  Step4(in_acc,h1,q_plus);
  Step5(H1);
}