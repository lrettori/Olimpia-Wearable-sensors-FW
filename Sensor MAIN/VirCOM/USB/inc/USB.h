#ifndef __USB_H
#define __USB_H

#include "usb_cnfg.h"
#include "usb_desc.h"
#include "usb_hw.h"
#include "usb_t9.h"
#include "usb_hooks.h"
#include "usb_dev_desc.h"
#include "usb_buffer.h"
#include "uart.h"

#include "cd_class.h"
#include "cdc_desc.h"
#include "cdc_cmd.h"

#define USB_UART         UART_3
#define USB_NUM_DATA     4
#define DFU_NUM_DATA     2
#define USB_CONN_PIN     GPIO_Pin_0
#define USB_CONN_PORT    GPIOC
#define USB_CONN_PERIPH  RCC_APB2Periph_GPIOC

void USB_DATA_READ(int USB_NUM, uint16_t *Int16DefaultBaud);
void USB_DATA_SEND(u8 USBDataTx);
void VirCOM_CON(void);
void VirCOM_DIS(void);
#endif

