      >�        �      �      �      	�    	4	�    	4	�    	17�	 7F	 0*�	 **	 0+�	 ++			 1,�	 ,,
 -b�	 bb "
" &// /@jj j�1
1	 (8/
/ //!j
j jj"C
C
 :N/'
/' /#/.j(
j( j$j//3
/3 /0/<j4
j4 j1j</A
/A />/OjB
jB j>jOBB Bh�� ��B 
B  BB&�0
�0 ��EB,
B, B(B3��  ��B8
B8 B5B@�0
!�0 ��EBF
BF BBBS�"�# ���2
$�2 ��K�%� & ���0
'�0! ��F�(�") ���"
*�"# ��%�+�$, ���"
*�"% ��%�-�&. ���#
*�#' ��&�/�(0 ���"
*�") ��%�1�*2 ���3�+4 ���
5�, ��$�6�-7 ���8�.9 ���
:�/ ��&�g6�-7 ���e6�-7 ���g1�*2 ���e1�*2 ���;�0< ��� 
=� 1 ��%   > 
(8L`p�����������������������������������������������	�	�	�	�	�	�	�
�
LSM9DS1.h HAL_LSM9DS1.h stm32f10x_i2c.h stm32f10x_dma.h I2C2_DMA_CHANNEL_TX I2C2_DMA_CHANNEL_RX I2C2_DR_Address LSM_Acc_InitStructure LSM_Gyr_InitStructure LSM_Magn_InitStructure I2C_DMA_Config void I2C_DMA_Config(int *, int *, int) I2Cx int * pBuffer lBufferSize int LSM_I2C_ByteWrite void LSM_I2C_ByteWrite(int, int *, int, int) slAddr WriteAddr NumByteToWrite LSM_I2C_DMA_BufferRead void LSM_I2C_DMA_BufferRead(int, int *, int, int) ReadAddr NumByteToRead LSM_I2C_BufferRead void LSM_I2C_BufferRead(int, int *, int, int) LSM9DS1_A_Config void LSM9DS1_A_Config(int *) LSM_Acc_Config_Struct LSM9DS1_G_Config void LSM9DS1_G_Config(int *) LSM_Gyr_Config_Struct LSM9DS1_AG_Config void LSM9DS1_AG_Config(int *) LSM_Acc_Gyr_Config_Struct LSM9DS1_M_Config void LSM9DS1_M_Config(int *) LSM_Magn_Config_Struct LSM9DS1_A_Read_RawData void LSM9DS1_A_Read_RawData(int *) out LSM9DS1_G_Read_RawData void LSM9DS1_G_Read_RawData(int *) LSM9DS1_GA_Read_RawData void LSM9DS1_GA_Read_RawData(int *) LSM9DS1_M_Read_RawData void LSM9DS1_M_Read_RawData(int *) LSM9DS1_Acc_Gyr_Config void LSM9DS1_Acc_Gyr_Config(void) LSM9DS1_Magn_Config void LSM9DS1_Magn_Config(int) magn_en LSM9DS1_I2C_Init void LSM9DS1_I2C_Init(void) SensorsConfig void SensorsConfig(int) mag_enable LSM_DataProcess void LSM_DataProcess(int *) mData    2 +U{��������������������������	�	�	�	�
�
��������������� c:LSM9DS1.c@618@macro@I2C2_DMA_CHANNEL_TX c:LSM9DS1.c@670@macro@I2C2_DMA_CHANNEL_RX c:LSM9DS1.c@722@macro@I2C2_DR_Address c:@LSM_Acc_InitStructure c:@LSM_Gyr_InitStructure c:@LSM_Magn_InitStructure c:@F@I2C_DMA_Config c:LSM9DS1.c@1013@F@I2C_DMA_Config@I2Cx c:LSM9DS1.c@1032@F@I2C_DMA_Config@pBuffer c:LSM9DS1.c@1050@F@I2C_DMA_Config@lBufferSize c:@F@LSM_I2C_ByteWrite c:LSM9DS1.c@2786@F@LSM_I2C_ByteWrite@slAddr c:LSM9DS1.c@2797@F@LSM_I2C_ByteWrite@pBuffer c:LSM9DS1.c@2810@F@LSM_I2C_ByteWrite@WriteAddr c:LSM9DS1.c@2824@F@LSM_I2C_ByteWrite@NumByteToWrite c:@F@LSM_I2C_DMA_BufferRead c:LSM9DS1.c@3985@F@LSM_I2C_DMA_BufferRead@slAddr c:LSM9DS1.c@3996@F@LSM_I2C_DMA_BufferRead@pBuffer c:LSM9DS1.c@4009@F@LSM_I2C_DMA_BufferRead@ReadAddr c:LSM9DS1.c@4022@F@LSM_I2C_DMA_BufferRead@NumByteToRead c:@F@LSM_I2C_BufferRead c:LSM9DS1.c@7257@F@LSM_I2C_BufferRead@slAddr c:LSM9DS1.c@7268@F@LSM_I2C_BufferRead@pBuffer c:LSM9DS1.c@7281@F@LSM_I2C_BufferRead@ReadAddr c:LSM9DS1.c@7294@F@LSM_I2C_BufferRead@NumByteToRead c:@F@LSM9DS1_A_Config c:LSM9DS1.c@10597@F@LSM9DS1_A_Config@LSM_Acc_Config_Struct c:@F@LSM9DS1_G_Config c:LSM9DS1.c@11255@F@LSM9DS1_G_Config@LSM_Gyr_Config_Struct c:@F@LSM9DS1_AG_Config c:LSM9DS1.c@11930@F@LSM9DS1_AG_Config@LSM_Acc_Gyr_Config_Struct c:@F@LSM9DS1_M_Config c:LSM9DS1.c@12343@F@LSM9DS1_M_Config@LSM_Magn_Config_Struct c:@F@LSM9DS1_A_Read_RawData c:LSM9DS1.c@13383@F@LSM9DS1_A_Read_RawData@out c:@F@LSM9DS1_G_Read_RawData c:LSM9DS1.c@13683@F@LSM9DS1_G_Read_RawData@out c:@F@LSM9DS1_GA_Read_RawData c:LSM9DS1.c@13980@F@LSM9DS1_GA_Read_RawData@out c:@F@LSM9DS1_M_Read_RawData c:LSM9DS1.c@14282@F@LSM9DS1_M_Read_RawData@out c:@F@LSM9DS1_Acc_Gyr_Config c:@F@LSM9DS1_Magn_Config c:LSM9DS1.c@15709@F@LSM9DS1_Magn_Config@magn_en c:@F@LSM9DS1_I2C_Init c:@F@SensorsConfig c:LSM9DS1.c@17882@F@SensorsConfig@mag_enable c:@F@LSM_DataProcess c:LSM9DS1.c@18093@F@LSM_DataProcess@mData     ~<invalid loc> C:\Users\Michelangelo\Documents\IAR Embedded Workbench\STM32F10x - SP - sensor\Sensor_Lib\LSM9DS1\src\LSM9DS1.c 