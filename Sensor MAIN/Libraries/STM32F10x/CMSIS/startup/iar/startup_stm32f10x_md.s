;/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
;* File Name          : startup_stm32f10x_md.s
;* Author             : MCD Application Team
;* Version            : V3.1.2
;* Date               : 09/28/2009
;* Description        : STM32F10x Medium Density Devices vector table for 
;*                      EWARM5.x toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == __iar_program_start,
;*                      - Set the vector table entries with the exceptions ISR 
;*                        address.
;*                      After Reset the Cortex-M3 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;********************************************************************************
;* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
;* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
;* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
;* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
;* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
;* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
;*******************************************************************************/
;
;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT:REORDER(3)

        SECTION .intvec:CODE:NOROOT:REORDER(2)

        EXTERN  __iar_program_start
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     __iar_program_start

        DCD     NMI_Handler               ; NMI Handler
        DCD     HardFault_Handler         ; Hard Fault Handler
        DCD     MemManage_Handler         ; MPU Fault Handler
        DCD     BusFault_Handler          ; Bus Fault Handler
        DCD     UsageFault_Handler        ; Usage Fault Handler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SVC_Handler               ; SVCall Handler
        DCD     DebugMon_Handler          ; Debug Monitor Handler
        DCD     0                         ; Reserved
        DCD     PendSV_Handler            ; PendSV Handler
        DCD     SysTick_Handler           ; SysTick Handler

         ; External Interrupts
        DCD     WWDG_IRQHandler           ; Window Watchdog
        DCD     PVD_IRQHandler            ; PVD through EXTI Line detect
        DCD     TAMPER_IRQHandler         ; Tamper
        DCD     RTC_IRQHandler            ; RTC
        DCD     FLASH_IRQHandler          ; Flash
        DCD     RCC_IRQHandler            ; RCC
        DCD     EXTI0_IRQHandler          ; EXTI Line 0
        DCD     EXTI1_IRQHandler          ; EXTI Line 1
        DCD     EXTI2_IRQHandler          ; EXTI Line 2
        DCD     EXTI3_IRQHandler          ; EXTI Line 3
        DCD     EXTI4_IRQHandler          ; EXTI Line 4
        DCD     DMA1_Channel1_IRQHandler  ; DMA1 Channel 1
        DCD     DMA1_Channel2_IRQHandler  ; DMA1 Channel 2
        DCD     DMA1_Channel3_IRQHandler  ; DMA1 Channel 3
        DCD     DMA1_Channel4_IRQHandler  ; DMA1 Channel 4
        DCD     DMA1_Channel5_IRQHandler  ; DMA1 Channel 5
        DCD     DMA1_Channel6_IRQHandler  ; DMA1 Channel 6
        DCD     DMA1_Channel7_IRQHandler  ; DMA1 Channel 7
        DCD     ADC1_2_IRQHandler         ; ADC1 & ADC2
        DCD     USB_HP_CAN1_TX_IRQHandler  ; USB High Priority or CAN1 TX
        DCD     USB_LP_CAN1_RX0_IRQHandler ; USB Low  Priority or CAN1 RX0
        DCD     CAN1_RX1_IRQHandler       ; CAN1 RX1
        DCD     CAN1_SCE_IRQHandler       ; CAN1 SCE
        DCD     EXTI9_5_IRQHandler        ; EXTI Line 9..5
        DCD     TIM1_BRK_IRQHandler       ; TIM1 Break
        DCD     TIM1_UP_IRQHandler        ; TIM1 Update
        DCD     TIM1_TRG_COM_IRQHandler   ; TIM1 Trigger and Commutation
        DCD     TIM1_CC_IRQHandler        ; TIM1 Capture Compare
        DCD     TIM2_IRQHandler           ; TIM2
        DCD     TIM3_IRQHandler           ; TIM3
        DCD     TIM4_IRQHandler           ; TIM4
        DCD     I2C1_EV_IRQHandler        ; I2C1 Event
        DCD     I2C1_ER_IRQHandler        ; I2C1 Error
        DCD     I2C2_EV_IRQHandler        ; I2C2 Event
        DCD     I2C2_ER_IRQHandler        ; I2C2 Error
        DCD     SPI1_IRQHandler           ; SPI1
        DCD     SPI2_IRQHandler           ; SPI2
        DCD     USART1_IRQHandler         ; USART1
        DCD     USART2_IRQHandler         ; USART2
        DCD     USART3_IRQHandler         ; USART3
        DCD     EXTI15_10_IRQHandler      ; EXTI Line 15..10
        DCD     RTCAlarm_IRQHandler       ; RTC Alarm through EXTI Line
        DCD     USBWakeUp_IRQHandler      ; USB Wakeup from suspend

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;

            PUBWEAK NMI_Handler               
            PUBWEAK HardFault_Handler         
            PUBWEAK MemManage_Handler         
            PUBWEAK BusFault_Handler          
            PUBWEAK UsageFault_Handler        
            PUBWEAK SVC_Handler               
            PUBWEAK DebugMon_Handler          
            PUBWEAK PendSV_Handler            
            PUBWEAK SysTick_Handler           
            PUBWEAK WWDG_IRQHandler           
            PUBWEAK PVD_IRQHandler            
            PUBWEAK TAMPER_IRQHandler         
            PUBWEAK RTC_IRQHandler            
            PUBWEAK FLASH_IRQHandler          
            PUBWEAK RCC_IRQHandler            
            PUBWEAK EXTI0_IRQHandler          
            PUBWEAK EXTI1_IRQHandler          
            PUBWEAK EXTI2_IRQHandler          
            PUBWEAK EXTI3_IRQHandler          
            PUBWEAK EXTI4_IRQHandler          
            PUBWEAK DMA1_Channel1_IRQHandler  
            PUBWEAK DMA1_Channel2_IRQHandler  
            PUBWEAK DMA1_Channel3_IRQHandler  
            PUBWEAK DMA1_Channel4_IRQHandler  
            PUBWEAK DMA1_Channel5_IRQHandler  
            PUBWEAK DMA1_Channel6_IRQHandler  
            PUBWEAK DMA1_Channel7_IRQHandler  
            PUBWEAK ADC1_2_IRQHandler         
            PUBWEAK USB_HP_CAN1_TX_IRQHandler 
            PUBWEAK USB_LP_CAN1_RX0_IRQHandler
            PUBWEAK CAN1_RX1_IRQHandler       
            PUBWEAK CAN1_SCE_IRQHandler       
            PUBWEAK EXTI9_5_IRQHandler        
            PUBWEAK TIM1_BRK_IRQHandler       
            PUBWEAK TIM1_UP_IRQHandler        
            PUBWEAK TIM1_TRG_COM_IRQHandler   
            PUBWEAK TIM1_CC_IRQHandler        
            PUBWEAK TIM2_IRQHandler           
            PUBWEAK TIM3_IRQHandler           
            PUBWEAK TIM4_IRQHandler           
            PUBWEAK I2C1_EV_IRQHandler        
            PUBWEAK I2C1_ER_IRQHandler        
            PUBWEAK I2C2_EV_IRQHandler        
            PUBWEAK I2C2_ER_IRQHandler        
            PUBWEAK SPI1_IRQHandler           
            PUBWEAK SPI2_IRQHandler           
            PUBWEAK USART1_IRQHandler         
            PUBWEAK USART2_IRQHandler         
            PUBWEAK USART3_IRQHandler         
            PUBWEAK EXTI15_10_IRQHandler      
            PUBWEAK RTCAlarm_IRQHandler       
            PUBWEAK USBWakeUp_IRQHandler      


        THUMB
        SECTION .text:CODE:NOROOT:REORDER(1)
NMI_Handler               
HardFault_Handler         
MemManage_Handler         
BusFault_Handler          
UsageFault_Handler        
SVC_Handler               
DebugMon_Handler          
PendSV_Handler            
SysTick_Handler           
WWDG_IRQHandler           
PVD_IRQHandler            
TAMPER_IRQHandler         
RTC_IRQHandler            
FLASH_IRQHandler          
RCC_IRQHandler            
EXTI0_IRQHandler          
EXTI1_IRQHandler          
EXTI2_IRQHandler          
EXTI3_IRQHandler          
EXTI4_IRQHandler          
DMA1_Channel1_IRQHandler  
DMA1_Channel2_IRQHandler  
DMA1_Channel3_IRQHandler  
DMA1_Channel4_IRQHandler  
DMA1_Channel5_IRQHandler  
DMA1_Channel6_IRQHandler  
DMA1_Channel7_IRQHandler  
ADC1_2_IRQHandler         
USB_HP_CAN1_TX_IRQHandler 
USB_LP_CAN1_RX0_IRQHandler
CAN1_RX1_IRQHandler       
CAN1_SCE_IRQHandler       
EXTI9_5_IRQHandler        
TIM1_BRK_IRQHandler       
TIM1_UP_IRQHandler        
TIM1_TRG_COM_IRQHandler   
TIM1_CC_IRQHandler        
TIM2_IRQHandler           
TIM3_IRQHandler           
TIM4_IRQHandler           
I2C1_EV_IRQHandler        
I2C1_ER_IRQHandler        
I2C2_EV_IRQHandler        
I2C2_ER_IRQHandler        
SPI1_IRQHandler           
SPI2_IRQHandler           
USART1_IRQHandler         
USART2_IRQHandler         
USART3_IRQHandler         
EXTI15_10_IRQHandler      
RTCAlarm_IRQHandler       
USBWakeUp_IRQHandler      
Default_Handler
        B Default_Handler

        END
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
