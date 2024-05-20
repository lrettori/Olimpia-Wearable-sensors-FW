/**
 ******************************************************************************
 * BLUETOOTH - MULTILINK CENTRAL
 ******************************************************************************
 * @file        main.c
 * @author      Lorenzo Rettori
 * @version     v2.0
 * @date        2024
 * @brief       Main program body.
 *              This project handles the central unit of a sensor network (The 
 *              Dongle). The central unit collects data from up to 4 periphelas 
 *              and streams them to the laptop via serial port. 
 *              Bluetooth communication between central and peripherals is 
 *              handled via NUS modules. Peripherals associated with this 
 *              central are handled using firmware in "peripheral BLE" folder.
 *              Peripherals may be of two different types:
 *                      - SensHand: streaming data from 3 IMUs.
 *                      - SensFoot: streaming data from 1 IMU.
 *              
 *              Main modifications from version 1.0:
 *              - The data stored in the input buffers (packets received
 *                from the sensors) is enclosed in the structure to be sent to
 *                the PC via serial port, only if received by all the connected
 *                sensors. This step is performed with a frequency of 10 Hz, and
 *                all the available packets are sent.
 *              - Several small modifications to solve bugs of the previous
 *                version, mainly regarding syncronization problems in case
 *                of multiple connections/disconnections of sensors
 * 
 ******************************************************************************
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be 
 *    reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************
 ******************************************************************************
 * This application is based on Nordic Semiconductor Examples, developed with 
 * and for the following components:
 *      - Nordic firware version: SDK v.17.1.0
 *      - Components: BMD300/BMD350
 *
 ******************************************************************************
 */

// Include ____________________________________________________________________
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h> 
#include <math.h>

#include "nordic_common.h"

#include "app_error.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util.h"

#include "ble.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "ble_hci.h"
#include "bsp_btn_ble.h"

#include "ble_mic_c.h"
   
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_log_default_backends.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "nrf_drv_clock.h"

// Definitions ________________________________________________________________
#define DEVICE_NAME               "Central_Device"
#define APP_BLE_CONN_CFG_TAG      1                                             // Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. 
                                                                                // The default tag is @ref APP_BLE_CONN_CFG_TAG.
#define APP_BLE_OBSERVER_PRIO     3                                             // BLE observer priority of the application. There is no need to modify this value.

#define UART_TX_BUF_SIZE          4096                                           // UART TX buffer size  4096
#define UART_RX_BUF_SIZE          4096                                           // UART RX buffer size  4096

#define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1
#define LEDBUTTON_LED             BSP_BOARD_LED_2                               // LED to indicate a change of state of the Button characteristic on the peer.

#define LEDBUTTON_BUTTON          BSP_BUTTON_0                                  // Button that writes to the LED characteristic of the peer.
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                           // Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks).

#define NUS_SERVICE_UUID_TYPE     BLE_UUID_TYPE_VENDOR_BEGIN  
#define ECHOBACK_BLE_UART_DATA    0                                             // Do not echo the UART data that is received over the Nordic UART Service (NUS) back to the sender.

//BLE Communication ___________________________________________________________
NRF_BLE_GATT_DEF(m_gatt);
BLE_MIC_C_ARRAY_DEF(m_ble_mic_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);

NRF_BLE_SCAN_DEF(m_scan);

NRF_BLE_GQ_DEF(m_ble_gatt_queue, 
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_mic_max_data_len = BLE_GATT_ATT_MTU_DEFAULT -             // Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module.
                                         OPCODE_LENGTH - HANDLE_LENGTH;  
                                                                                                        
static char const m_target_periph_name_00[] = "Nordic_UART_00";                 // Name of the device to try to connect to. This name is searched for in the scanning report data.
static char const m_target_periph_name_01[] = "Nordic_UART_01";                 // To enable more devices, it is needed to modify the NRF_BLE_SCAN_NAME_CNT value in sdk_config.h
static char const m_target_periph_name_02[] = "Nordic_UART_02";                 // functions scan_init and conn_init need to be modified.        
static char const m_target_periph_name_03[] = "Nordic_UART_03";

// Multilink data streaming ___________________________________________________
// The BLE communication between central and peripherals will be handled with an
// unsynchronized approach. Each peripheral streams independently and its data 
// are collected in a buffer BUF_ before being sent to the serial port. 
// Data are sent to the serial port using a clock, so with a fixed frequency,
// not depending on how fast they are collected.

// data_length_H and data_lengh_F are the expected lengths of the SensHand and
// SensFoot data arrays, respectively. SensHand contains data from 3 IMUs 
// (acceleration and angular velocity) for a total of 18 readings at time. 
// SensFoot contains data from 1 IMU, so 6 reading at time.
// Since data are uin16 and need to be casted in uint8 arrays, lengths need to
// doubled
#define data_length_H   36                                                      
#define data_length_F   12                                                      

#define store_sz        2520 // Increased from 2500 to 2520, to store an integer number of hand/feet data vectors (respectively 70 and 210)
uint8_t    BUF_0[store_sz];                                                     // Internal buffers to store data from peripherals before streaming the whole package
uint8_t    BUF_1[store_sz];
uint8_t    BUF_2[store_sz];
uint8_t    BUF_3[store_sz];

bool C_0 = false;
bool C_1 = false;
bool C_2 = false;
bool C_3 = false;

// Connected sensors
bool isSensConnected[4] = {false, false, false, false};

// Connected sensors counter
int sens_n  = 0;

// Order of connected sensors (from 0 to 3)
uint16_t conn_0 = 9999;
uint16_t conn_1 = 9999;
uint16_t conn_2 = 9999;
uint16_t conn_3 = 9999;

// Counters for packets received from sensors and sent via UART
uint16_t packetsReceived [4];
uint16_t packetsSent [4];

// Iteration index to print upcoming into the buffers and to read them from the
// buffers and send to the serial port.
uint16_t I_0 = 0;
uint16_t I_1 = 0;
uint16_t I_2 = 0;
uint16_t I_3 = 0;

uint16_t K_0 = 0;
uint16_t K_1 = 0;
uint16_t K_2 = 0;
uint16_t K_3 = 0;

// Final array to be sent to the serial port. Length is due to handle two 
// SensHands, two SensFoots, a start character and an end character. 
// So 36 * 2 + 12 * 2 + 2 = 98
uint8_t PRINT[98];
//uint8_t PRINT[106]; // For debug

float acc_fr_x;
float acc_fr_y; 
float acc_fr_z;
float gyr_fr_x;
float gyr_fr_y;
float gyr_fr_z;

uint16_t counter1 = 0;


// TIMER ______________________________________________________________________
// Timer is set with a sampling period of 100ms (10 Hz)
// This defines the frequency under which the dongle looks at the input buffers
// of the connected sensors and sends the packets received (if any) via serial 
// port. Data are received by the sensors with their sampling frequency
#define TIM_INTERVAL    APP_TIMER_TICKS(100) // 10 Hz

APP_TIMER_DEF(m_app_timer_id);

// FUNCTIONS __________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________

// ____________________________________________________________________________
/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final 
 *          product. You need to analyze how your product is supposed to react 
 *          in case of an assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

// ____________________________________________________________________________
/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went 
 *                          wrong.
 */
static void mic_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

// ____________________________________________________________________________         
/**@brief Function for starting scanning. */                                                     
static void scan_start(void)                                                   
{
  ret_code_t ret;
  
  ret = nrf_ble_scan_start(&m_scan);
  APP_ERROR_CHECK(ret);
  
  // Turn on the LED to signal scanning.
  bsp_board_led_on(CENTRAL_SCANNING_LED);
}

// ____________________________________________________________________________
/**@brief Function for the LEDs initialization.
 *
 * @details Initilizes all LEDs used by this application. 
 */
static void leds_init(void)
{
  bsp_board_init(BSP_INIT_LEDS);
}

// ____________________________________________________________________________
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
  ret_code_t err_code;
  switch(p_scan_evt->scan_evt_id)
  {
  case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
    {
      err_code = p_scan_evt->params.connecting_err.err_code;
      APP_ERROR_CHECK(err_code);
    } break;

  case NRF_BLE_SCAN_EVT_CONNECTED:
    {
      ble_gap_evt_connected_t const * p_connected =
        p_scan_evt->params.connected.p_connected;
      
      // Scan is automatically stopped by the connection.
      if(p_connected->peer_addr.addr[0]) {}
    } break;
    
//  case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
//    {
//      // NRF_LOG_INFO("Scan timed out.");
//      // scan_start();
//    } break;
    
  default:
    break;
  }
}

// ____________________________________________________________________________
/**@brief Function for initializing the scanning and setting the filters.
 *
 * @details Tis function initialize scanning for peripherals to be connected. 
 *          Peripherals are addressed by name. Each peripheral is scanned 
 *          independently if its associated condition (C_0, C_1, C_2, C_3) is
 *          toggled true by the user. These conditions are activated through
 *          specific commands as shown in uart_event_handle
 */
static void scan_init(void)
{
  ret_code_t          err_code;
  nrf_ble_scan_init_t init_scan;
  
  memset(&init_scan, 0, sizeof(init_scan));
  
  init_scan.connect_if_match = false;   // This allows to find peripherals but
                                        // not automatically connect to them.
  init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
  
  err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
  APP_ERROR_CHECK(err_code);
  
  // Nordic_UART_00
  if (C_0) {
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER,
                                       m_target_periph_name_00);
    APP_ERROR_CHECK(err_code);
  }
  
  // Nordic_UART_01
  if (C_1) {
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, 
                                       m_target_periph_name_01);
    APP_ERROR_CHECK(err_code);
  }

  // Nordic_UART_02
  if (C_2) {
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER,
                                       m_target_periph_name_02);
    APP_ERROR_CHECK(err_code);
  }
  
  // Nordic_UART_03
  if (C_3) {
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER,
                                       m_target_periph_name_03);
    APP_ERROR_CHECK(err_code);
  }
  err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER,
                                         false);
  APP_ERROR_CHECK(err_code);
}

// ____________________________________________________________________________
/**@brief Function for initializing the connection.
 *
 * @details Tis function initialize connection for peripherals to be connected. 
 *          Peripherals are addressed by name. Each peripheral is scanned 
 *          independently if its associated condition (C_0, C_1, C_2, C_3) is
 *          toggled true by the user. These conditions are activated through
 *          specific commands as shown in uart_event_handle
 */
static void conn_init(void)
{
  ret_code_t          err_code;
  nrf_ble_scan_init_t init_scan;
  
  memset(&init_scan, 0, sizeof(init_scan));
  
  init_scan.connect_if_match = true;    // If a match is found, enstablish 
                                        // connection.
  init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
  
  err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
  APP_ERROR_CHECK(err_code);
  
  // Nordic_UART_00
  if (C_0) {
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER,
                                       m_target_periph_name_00);
    APP_ERROR_CHECK(err_code);
  }

  // Nordic_UART_01
  if (C_1) {
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, 
                                       m_target_periph_name_01);
    APP_ERROR_CHECK(err_code);
  }
  
  // Nordic_UART_02
  if (C_2) {
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, 
                                       m_target_periph_name_02);
    APP_ERROR_CHECK(err_code);
  }
  
  // Nordic_UART_03
  if (C_3) {
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER,
                                       m_target_periph_name_03);
    APP_ERROR_CHECK(err_code);
  }
  //
  err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER,
                                         false);
  APP_ERROR_CHECK(err_code);
}

// ____________________________________________________________________________
/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the 
 *          database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards
 *          the events to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
//  NRF_LOG_DEBUG("call to ble_mic_on_db_disc_evt for instance %d and link 0x%x!",
//                p_evt->conn_handle, p_evt->conn_handle);  
  ble_mic_c_on_db_disc_evt(&m_ble_mic_c[p_evt->conn_handle], p_evt);
}

// ____________________________________________________________________________
/**@brief Function for handling characters received by the Nordic UART Service
 *        (NUS).
 *
 * @details This function takes a list of characters of length data_len and 
 *          prints the characters out on UART. If @ref ECHOBACK_BLE_UART_DATA is 
 *          set, the data is sent back to sender.
 */
static void ble_mic_chars_received_uart_print(uint8_t * p_data, 
                                              uint16_t data_len)
{
  ret_code_t ret_val;  
  for (uint32_t i = 0; i < data_len; i++)
  {
    do {
      ret_val = app_uart_put(p_data[i]);
      if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
      {
        // NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
        APP_ERROR_CHECK(ret_val);
      }
    } while (ret_val == NRF_ERROR_BUSY);
  }
}

// ____________________________________________________________________________ 
/**@brief   Function for handling app_uart events.
 *
 * @details This function receives data from the UART. More in detail, this is 
 *          used to send commands such as enstablish connection or disconnect
 *          the peripherals. Available commands are:
 *              - SCAN to assess the available peripherals.
 *              - CONNECT all the available peripherals.
 *              - CONNECT one or more specific peripherals.
 *              - DISCONNECT all the available peripherals.
 *              - DISCONNECT one or more specific peripherals.
 *              - SEND 1 byte to all available peripherals.
 *              - SEND 1 byte to one or more specific peripherals.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
  static uint8_t data_array[BLE_MIC_MAX_DATA_LEN];
  static uint16_t index = 0;
  ret_code_t ret_val;
  uint8_t byte[1];
  byte[0] = (uint8_t) 1;
  
  switch (p_event->evt_type)
  {
  /**@snippet [Handling data from UART] */
  case APP_UART_DATA_READY:
    UNUSED_VARIABLE(app_uart_get(&data_array[index]));
    // ------------------------------------------------------------------------
    // SCAN FOR THE AVAILABLE PERIPHERALS: at$scan
    if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
        (data_array[index -2] == 'n')&&(data_array[index -3] == 'a')&&
        (data_array[index -4] == 'c')&&(data_array[index -5] == 's')&&
        (data_array[index -6] == '$')&&(data_array[index -7] == 't')&&
        (data_array[index -8] == 'a'))
    {
      scan_init();
    }    
    // ------------------------------------------------------------------------
    // CONNECT ALL PERIPHERALS: at$cntn
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == 'n')&&(data_array[index -3] == 't')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 'c')&&
             (data_array[index -6] == '$')&&(data_array[index -7] == 't')&&
             (data_array[index -8] == 'a'))
    {
      C_0 = true;
      C_1 = true;
      C_2 = true;
      C_3 = true;
      conn_init();
      scan_start();
    }
    // ------------------------------------------------------------------------
    // CONNECT TO PERIPHERAL_0: at$cntn_0
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '0')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 't')&&
             (data_array[index -6] == 'n')&&(data_array[index -7] == 'c')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      C_0 = true;
      conn_init();
      scan_start();    
    }
    // ------------------------------------------------------------------------
    // CONNECT TO PERIPHERAL_1: at$cntn_1
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '1')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 't')&&
             (data_array[index -6] == 'n')&&(data_array[index -7] == 'c')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      C_1 = true;
      conn_init();
      scan_start();    
    }
    // ------------------------------------------------------------------------
    // CONNECT TO PERIPHERAL_2: at$cntn_2
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '2')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 't')&&
             (data_array[index -6] == 'n')&&(data_array[index -7] == 'c')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      C_2 = true;
      conn_init();
      scan_start();    
    }
    // ------------------------------------------------------------------------
    // CONNECT TO PERIPHERAL_3: at$cntn_3
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '3')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 't')&&
             (data_array[index -6] == 'n')&&(data_array[index -7] == 'c')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      C_3 = true;
      conn_init();
      scan_start();    
    }
    // ------------------------------------------------------------------------
    // DISCONNECT TO ALL: at$dntn
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == 'n')&&(data_array[index -3] == 't')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 'd')&&
             (data_array[index -6] == '$')&&(data_array[index -7] == 't')&&
             (data_array[index -8] == 'a'))
    {
      // First, scan is disabled in order to avoid unwanted re-connection, then 
      // connected peripherals are disconnected.
      nrf_ble_scan_stop();
      uint32_t err_code;
      for (int c = 0; c < sens_n; c++) 
      {
        do
        {
          err_code = sd_ble_gap_disconnect(m_ble_mic_c[c].conn_handle,
                                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
          APP_ERROR_CHECK(err_code);
        } while (err_code != NRF_SUCCESS);
      }
      
      // Turn off SCANNING AND CONNECTING LEDS
      bsp_board_led_off(CENTRAL_CONNECTED_LED);
      bsp_board_led_off(CENTRAL_SCANNING_LED);
      
      if (err_code != NRF_ERROR_INVALID_STATE)
      {
        APP_ERROR_CHECK(err_code);
      }
      index = 0;
    }
    // ------------------------------------------------------------------------
    // DISCONNECT TO SENSOR_0: at$dntn_0
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '0')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 't')&&
             (data_array[index -6] == 'n')&&(data_array[index -7] == 'd')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      nrf_ble_scan_stop();    
      uint32_t err_code;
      err_code = sd_ble_gap_disconnect(conn_0,
                            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      
      if (sens_n == 1) 
      {
        // Turn off SCANNING AND CONNECTING LEDS
        bsp_board_led_off(CENTRAL_CONNECTED_LED);
        bsp_board_led_off(CENTRAL_SCANNING_LED);
      }
      index = 0;
    }
    // ------------------------------------------------------------------------
    // DISCONNECT TO SENSOR_1: at$dntn_1
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '1')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 't')&&
             (data_array[index -6] == 'n')&&(data_array[index -7] == 'd')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      nrf_ble_scan_stop();    
      uint32_t err_code;
      err_code = sd_ble_gap_disconnect(conn_1,
                            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      
      if (sens_n == 1) 
      {
        // Turn off SCANNING AND CONNECTING LEDS
        bsp_board_led_off(CENTRAL_CONNECTED_LED);
        bsp_board_led_off(CENTRAL_SCANNING_LED);
      }
      index = 0;
    }
    // ------------------------------------------------------------------------
    // DISCONNECT TO SENSOR_2: at$dntn_2
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '2')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 't')&&
             (data_array[index -6] == 'n')&&(data_array[index -7] == 'd')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      nrf_ble_scan_stop();    
      uint32_t err_code;
      err_code = sd_ble_gap_disconnect(conn_2,
                            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      
      if (sens_n == 1) 
      {
        // Turn off SCANNING AND CONNECTING LEDS
        bsp_board_led_off(CENTRAL_CONNECTED_LED);
        bsp_board_led_off(CENTRAL_SCANNING_LED);
      }
      index = 0;
    }
    // ------------------------------------------------------------------------
    // DISCONNECT TO SENSOR_3: at$dntn_3
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '3')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 'n')&&(data_array[index -5] == 't')&&
             (data_array[index -6] == 'n')&&(data_array[index -7] == 'd')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      nrf_ble_scan_stop();    
      uint32_t err_code;
      err_code = sd_ble_gap_disconnect(conn_3,
                            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      
      if (sens_n == 1) 
      {
        // Turn off SCANNING AND CONNECTING LEDS
        bsp_board_led_off(CENTRAL_CONNECTED_LED);
        bsp_board_led_off(CENTRAL_SCANNING_LED);
      }
      index = 0;
    }
    // ------------------------------------------------------------------------
    // SEND BYTE TO ALL: at$sdbt
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == 't')&&(data_array[index -3] == 'b')&&
             (data_array[index -4] == 'd')&&(data_array[index -5] == 's')&&
             (data_array[index -6] == '$')&&(data_array[index -7] == 't')&&
             (data_array[index -8] == 'a'))
    {
      for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++) 
      {
        // Send data to the peripheral 
        do
        {
          ret_val = ble_mic_c_string_send(&m_ble_mic_c[c], byte, 1);
          if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) &&
              (ret_val != NRF_ERROR_INVALID_STATE))
          {
            APP_ERROR_CHECK(ret_val);
          }
        } while (ret_val == NRF_ERROR_BUSY);
      }
    }
    // ------------------------------------------------------------------------
    // SEND BYTE TO PERIPH_0: at$sdbt_0
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '0')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 't')&&(data_array[index -5] == 'b')&&
             (data_array[index -6] == 'd')&&(data_array[index -7] == 's')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      do
      {
        ret_val = ble_mic_c_string_send(&m_ble_mic_c[conn_0], byte, 1);
        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) &&
            (ret_val != NRF_ERROR_INVALID_STATE))
        {
          APP_ERROR_CHECK(ret_val);
        }
      } while (ret_val == NRF_ERROR_BUSY);
    }
    // ------------------------------------------------------------------------
    // SEND BYTE TO PERIPH_1: at$sdbt_1
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '1')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 't')&&(data_array[index -5] == 'b')&&
             (data_array[index -6] == 'd')&&(data_array[index -7] == 's')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      do
      {
        ret_val = ble_mic_c_string_send(&m_ble_mic_c[conn_1], byte, 1);
        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) &&
            (ret_val != NRF_ERROR_INVALID_STATE))
        {
          APP_ERROR_CHECK(ret_val);
        }
      } while (ret_val == NRF_ERROR_BUSY);
    }
    // ------------------------------------------------------------------------
    // SEND BYTE TO PERIPH_2: at$sdbt_2
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '2')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 't')&&(data_array[index -5] == 'b')&&
             (data_array[index -6] == 'd')&&(data_array[index -7] == 's')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      do
      {
        ret_val = ble_mic_c_string_send(&m_ble_mic_c[conn_2], byte, 1);
        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) &&
            (ret_val != NRF_ERROR_INVALID_STATE))
        {
          APP_ERROR_CHECK(ret_val);
        }
      } while (ret_val == NRF_ERROR_BUSY);
    }
    // ------------------------------------------------------------------------
    // SEND BYTE TO PERIPH_3: at$sdbt_3
    else if ((data_array[index] == 0x0A)&& (data_array[index -1] == 0x0D)&&
             (data_array[index -2] == '3')&&(data_array[index -3] == '_')&&
             (data_array[index -4] == 't')&&(data_array[index -5] == 'b')&&
             (data_array[index -6] == 'd')&&(data_array[index -7] == 's')&&
             (data_array[index -8] == '$')&&(data_array[index -9] == 't')&&
             (data_array[index -10] == 'a'))
    {
      do
      {
        ret_val = ble_mic_c_string_send(&m_ble_mic_c[conn_3], byte, 1);
        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) &&
            (ret_val != NRF_ERROR_INVALID_STATE))
        {
          APP_ERROR_CHECK(ret_val);
        }
      } while (ret_val == NRF_ERROR_BUSY);
    }
    else {
      index++;
    }
    break;
    
  case APP_UART_COMMUNICATION_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_communication);
    break;
  
  case APP_UART_FIFO_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_code);
    break;
  
  default:
    break;
  }
}

// ____________________________________________________________________________ 
/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client 
 *          events. More precisely this application is what it's needed to 
 *          handle upcoming data over BLE module. This application checks which
 *          sensor is sending data and stores them in the right buffer. For 
 *          development reasons NUS libraries were changed into "ble_mic" files.
 *          accordingly, renamed functions are used here but the logic is the
 *          same.
 *
 * @param[in]   p_ble_mic_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_mic_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_mic_c module] */
static void ble_mic_c_evt_handler(ble_mic_c_t * p_ble_mic_c, 
                                  ble_mic_c_evt_t const * p_ble_mic_evt)
{
  ret_code_t err_code; 
  switch (p_ble_mic_evt->evt_type)
  {
  case BLE_MIC_C_EVT_DISCOVERY_COMPLETE:
    err_code = ble_mic_c_handles_assign(p_ble_mic_c, 
                                        p_ble_mic_evt->conn_handle,
                                        &p_ble_mic_evt->handles);
    APP_ERROR_CHECK(err_code);
    err_code = ble_mic_c_tx_notif_enable(p_ble_mic_c);
    APP_ERROR_CHECK(err_code);
    break;
  case BLE_MIC_C_EVT_MIC_TX_EVT:      
    // If data are received, they are stored in a buffer. To assess in which 
    // buffer it is needed to store data, the algorithm checks the first element
    // that may be 0, 1, 2 or 3.
    //
    // After storing information into the right buffer, the associated iteration
    // index is then updated.
    //
    // SensHand, Left
    // Sensor 0, Sensor 1, Sensor 2 -------------------------------------------
    if (p_ble_mic_evt->p_data[0] == '0') {
      conn_0 = p_ble_mic_evt->conn_handle;
      // Acc [x, y, z], Gyr [x, y, z] 
      for (int i = 0; i < data_length_H; i++) {
        BUF_0[I_0 + i] = p_ble_mic_evt->p_data[i + 1];
      }

      I_0 = I_0 + data_length_H;
      if (I_0 > store_sz - data_length_H) {
        I_0 = 0;
      }
      if (isSensConnected[0] == false)
      {
        // Initialize the reading index, since after a new connection the data 
        // will be stored at initial positions of the input buffer
        K_0 = 0;
        isSensConnected[0] = true;
      }
      packetsReceived[0]++;
    }
    
    // SensHand, Right
    // Sensor 3, Sensor 4, Sensor 5 -------------------------------------------
    else if (p_ble_mic_evt->p_data[0] == '1') {
      conn_1 = p_ble_mic_evt->conn_handle;
      // Acc [x, y, z], Gyr [x, y, z]
      for (int i = 0; i < data_length_H; i++) {
        BUF_1[I_1 + i] = p_ble_mic_evt->p_data[i + 1];
      }
      
      I_1 = I_1 + data_length_H;
      if (I_1 > store_sz - data_length_H) {
        I_1 = 0;
      }
      if (isSensConnected[1] == false)
      {
        // Initialize the reading index, since after a new connection the data 
        // will be stored at initial positions of the input buffer
        K_1 = 0;
        isSensConnected[1] = true;
      }
      packetsReceived[1]++;
    }
    
    // SensFoot, Left
    // Sensor 6 ---------------------------------------------------------------
    else if (p_ble_mic_evt->p_data[0] == '2') {
      conn_2 = p_ble_mic_evt->conn_handle;
      // Acc [x, y, z], Gyr [x, y, z]
      for (int i = 0; i < data_length_F; i++) {
        BUF_2[I_2 + i] = p_ble_mic_evt->p_data[i + 1];
      }
      
      I_2 = I_2 + data_length_F;
      if (I_2 > store_sz - data_length_F) {
        I_2 = 0;
      }
      if (isSensConnected[2] == false)
      {
        // Initialize the reading index, since after a new connection the data 
        // will be stored at initial positions of the input buffer
        K_2 = 0;
        isSensConnected[2] = true;
      }
      packetsReceived[2]++;
    }
    
    // SensFoot, Right
    // Sensor 7 ---------------------------------------------------------------
    else if (p_ble_mic_evt->p_data[0] == '3') {
      conn_3 = p_ble_mic_evt->conn_handle;
      // Acc [x, y, z], Gyr [x, y, z]
      for (int i = 0; i < data_length_F; i++) {
        BUF_3[I_3 + i] = p_ble_mic_evt->p_data[i + 1];
      }
      
      I_3 = I_3 + data_length_F;
      if (I_3 > store_sz - data_length_F) {
        I_3 = 0;
      }
      if (isSensConnected[3] == false)
      {
        // Initialize the reading index, since after a new connection the data 
        // will be stored at initial positions of the input buffer
        K_3 = 0;
        isSensConnected[3] = true;
      }
      packetsReceived[3]++;
    }
    
    // ------------------------------------------------------------------------
    break;
  case BLE_MIC_C_EVT_DISCONNECTED:
    break;
  }
}

// ____________________________________________________________________________
/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
  ret_code_t err_code;
  
  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);
  
  switch (event)
  {
  case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
    break;
  
  default:
    break;
  }
  return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

// ____________________________________________________________________________
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  ret_code_t err_code;
  
  // For readability.
  ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;  
  
  switch (p_ble_evt->header.evt_id)
  {
  // Upon connection, check which peripheral is connected, initiate DB
  // discovery, update LEDs status, and resume scanning, if necessary.
  case BLE_GAP_EVT_CONNECTED:
    {
      // If a device is connected, the counter is updated
      sens_n++;          
      
      APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);
      err_code = ble_mic_c_handles_assign(&m_ble_mic_c[p_gap_evt->conn_handle],
                                          p_gap_evt->conn_handle,
                                          NULL);
      APP_ERROR_CHECK(err_code);
      
      err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],
                                        p_gap_evt->conn_handle);
      APP_ERROR_CHECK(err_code);
      
      // Update LEDs status and check whether it is needed to look for more
      // peripherals to connect to.
      bsp_board_led_on(CENTRAL_CONNECTED_LED);
      
      if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
      {
        bsp_board_led_off(CENTRAL_SCANNING_LED);
      }
      else
      {
        // Resume scanning.
        bsp_board_led_on(CENTRAL_SCANNING_LED);
        scan_start();
      }
    } break; // BLE_GAP_EVT_CONNECTED
    
    // Upon disconnection, reset the connection handle of the peer that
    // disconnected, update the LEDs status and start scanning again.
  case BLE_GAP_EVT_DISCONNECTED:
    {
      sens_n--;
      if (p_gap_evt->conn_handle == conn_0) {
        memset(BUF_0,0,store_sz);
        
        C_0 = false;
        isSensConnected[0] = false;
        I_0 = 0;
      }
      else if (p_gap_evt->conn_handle == conn_1) {
        memset(BUF_1,0,store_sz);
        
        C_1 = false;
        isSensConnected[1] = false;
        I_1 = 0;
      }
      else if (p_gap_evt->conn_handle == conn_2) {
        memset(BUF_2,0,store_sz);
                
        C_2 = false;
        isSensConnected[2] = false;
        I_2 = 0;
      }
      else if (p_gap_evt->conn_handle == conn_3) {
        memset(BUF_3,0,store_sz);
        
        C_3 = false;
        isSensConnected[3] = false;
        I_3 = 0;
      }
      
      if (ble_conn_state_central_conn_count() == 0)
      {
        err_code = app_button_disable();
        APP_ERROR_CHECK(err_code);
        
        // Turn off the LED that indicates the connection and turn off the LED 
        // that indicates the scanning/streaming.
        bsp_board_led_off(CENTRAL_CONNECTED_LED);
        bsp_board_led_off(CENTRAL_SCANNING_LED);
      }
    } break;
  
  case BLE_GAP_EVT_TIMEOUT:
    {
      // Timeout for scanning is not specified, so only the connection requests 
      // can time out.
      if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
      {
      }
    } break;
    
  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    {
      // Pairing not supported.
      err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, 
                                             BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, 
                                             NULL, NULL);
      APP_ERROR_CHECK(err_code);
    } break;
  
  case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
    {
      // Accept parameters requested by peer.
      err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                              &p_gap_evt->params.conn_param_update_request.conn_params);
      APP_ERROR_CHECK(err_code);
    } break;
  
  case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
      ble_gap_phys_t const phys =
      {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, 
                                       &phys);
      APP_ERROR_CHECK(err_code);
    } break;
  
  case BLE_GATTC_EVT_TIMEOUT:
    {
      // Disconnect on GATT client timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
    } break;
  
  case BLE_GATTS_EVT_TIMEOUT:
    {
      // Disconnect on GATT server timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
    } break;

  case BLE_GAP_EVT_ADV_REPORT:
    {
      // Here I should print devices names.
    } break;
    
  default:
    // No implementation needed.
    break;
  }
}

// ____________________________________________________________________________
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
  ret_code_t err_code;
  
  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);
  
  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);
  
  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);
  
  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, 
                       NULL);
}

// ____________________________________________________________________________
/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
  if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
  {
    m_ble_mic_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - 
                             HANDLE_LENGTH;
    printf("Ble NUS max data length set to 0x%X(%d)", 
           m_ble_mic_max_data_len, m_ble_mic_max_data_len);
  }
}

// ____________________________________________________________________________
/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
  APP_ERROR_CHECK(err_code);
  
  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, 
                                              NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

// ____________________________________________________________________________
/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
  ret_code_t err_code;
  
  switch (event)
  {
  case BSP_EVENT_SLEEP:
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    break;
  
  case BSP_EVENT_DISCONNECT:
    for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
    {
      err_code = sd_ble_gap_disconnect(m_ble_mic_c[c].conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_ERROR_INVALID_STATE)
      {
        APP_ERROR_CHECK(err_code);
      }
    }
    break;
  
  default:
    break;
  }
}

// ____________________________________________________________________________
/**@brief Function for initializing the UART. */
static void uart_init(void)
{
  ret_code_t err_code;
  app_uart_comm_params_t const comm_params =
  {
    .rx_pin_no    = RX_PIN_NUMBER,
    .tx_pin_no    = TX_PIN_NUMBER,
    .rts_pin_no   = RTS_PIN_NUMBER,
    .cts_pin_no   = CTS_PIN_NUMBER,
    .flow_control = APP_UART_FLOW_CONTROL_ENABLED,
    .use_parity   = false,
    .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud460800

  };
  APP_UART_FIFO_INIT(&comm_params, 
                     UART_RX_BUF_SIZE, 
                     UART_TX_BUF_SIZE,
                     uart_event_handle, 
                     APP_IRQ_PRIORITY_LOWEST, 
                     err_code);
  APP_ERROR_CHECK(err_code);
}

// ____________________________________________________________________________
/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void mic_c_init(void)
{
  ret_code_t       err_code;
  ble_mic_c_init_t init;
  
  init.evt_handler   = ble_mic_c_evt_handler;
  init.error_handler = mic_error_handler;
  init.p_gatt_queue  = &m_ble_gatt_queue;
  
  for(int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
  {
    err_code = ble_mic_c_init(&m_ble_mic_c[c], &init);
    APP_ERROR_CHECK(err_code);
  }
}

// ____________________________________________________________________________
/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

// ____________________________________________________________________________
/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
  ble_db_discovery_init_t db_init;
  memset(&db_init, 0, sizeof(ble_db_discovery_init_t));
  
  db_init.evt_handler  = db_disc_handler;
  db_init.p_gatt_queue = &m_ble_gatt_queue;
  
  ret_code_t err_code = ble_db_discovery_init(&db_init);
  APP_ERROR_CHECK(err_code);
}

// ____________________________________________________________________________
/**@brief Function for handling the idle state (main loop).
 *
 * @details This function handles any pending log operations, then sleeps until 
 *          the next event occurs.
 */
static void idle_state_handle(void)
{ 
  if (NRF_LOG_PROCESS() == false)
  {
    nrf_pwr_mgmt_run();
  }
}

// ____________________________________________________________________________
/**@brief Function to initialize the clock.
 *
 */
static void lfclk_config(void)
{
  ret_code_t err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);
  
  nrf_drv_clock_lfclk_request(NULL);            // No interrupt on ticks.
  
  // We are going to use the events with an interrupt handler.
}

void sendDataOverUart()
{
  // Print ! 
        PRINT[0] = (uint8_t) 'D';
        
        for (int i = 0; i < 2; i++) {
          // SensHand, Left _______________________________________________________
          // Sensor_0 : Wrist _____________________________________________________
          PRINT[i + 1]  = BUF_0[K_0 + i];               // Acc
          PRINT[i + 3]  = BUF_0[K_0 + 2  + i];     
          PRINT[i + 5]  = BUF_0[K_0 + 4  + i];
          
          PRINT[i + 7]  = BUF_0[K_0 + 6  + i];          // Gyr
          PRINT[i + 9]  = BUF_0[K_0 + 8  + i];     
          PRINT[i + 11] = BUF_0[K_0 + 10 + i];     
          
          // Sensor_1 : Thumb _____________________________________________________
          PRINT[i + 13] = BUF_0[K_0 + 12 + i];          // Acc
          PRINT[i + 15] = BUF_0[K_0 + 14 + i];     
          PRINT[i + 17] = BUF_0[K_0 + 16 + i];
          
          PRINT[i + 19] = BUF_0[K_0 + 18 + i];          // Gyr
          PRINT[i + 21] = BUF_0[K_0 + 20 + i];     
          PRINT[i + 23] = BUF_0[K_0 + 22 + i];
          
          // Sensor_2 : Index _____________________________________________________
          PRINT[i + 25] = BUF_0[K_0 + 24 + i];          // Acc
          PRINT[i + 27] = BUF_0[K_0 + 26 + i];     
          PRINT[i + 29] = BUF_0[K_0 + 28 + i];
          
          PRINT[i + 31] = BUF_0[K_0 + 30 + i];          // Gyr
          PRINT[i + 33] = BUF_0[K_0 + 32 + i];     
          PRINT[i + 35] = BUF_0[K_0 + 34 + i];
          
          // SensHand, Right ______________________________________________________
          // Sensor_3 : Wrist _____________________________________________________
          PRINT[i + 37] = BUF_1[K_1 + i];               // Acc
          PRINT[i + 39] = BUF_1[K_1 + 2  + i];     
          PRINT[i + 41] = BUF_1[K_1 + 4  + i];
          
          PRINT[i + 43] = BUF_1[K_1 + 6  + i];          // Gyr
          PRINT[i + 45] = BUF_1[K_1 + 8  + i];     
          PRINT[i + 47] = BUF_1[K_1 + 10 + i];
          
          // Sensor_4 : Thumb _____________________________________________________
          PRINT[i + 49] = BUF_1[K_1 + 12 + i];         // Acc
          PRINT[i + 51] = BUF_1[K_1 + 14 + i];     
          PRINT[i + 53] = BUF_1[K_1 + 16 + i];
          
          PRINT[i + 55] = BUF_1[K_1 + 18 + i];         // Gyr
          PRINT[i + 57] = BUF_1[K_1 + 20 + i];     
          PRINT[i + 59] = BUF_1[K_1 + 22 + i];
          
          // Sensor_5 : Index _____________________________________________________
          PRINT[i + 61] = BUF_1[K_1 + 24 + i];         // Acc
          PRINT[i + 63] = BUF_1[K_1 + 26 + i];     
          PRINT[i + 65] = BUF_1[K_1 + 28 + i];
          
          PRINT[i + 67] = BUF_1[K_1 + 30 + i];         // Gyr
          PRINT[i + 69] = BUF_1[K_1 + 32 + i];     
          PRINT[i + 71] = BUF_1[K_1 + 34 + i];
          
          // SensFoot, Left _______________________________________________________
          // Sensor_6 _____________________________________________________________
          PRINT[i + 73] = BUF_2[K_2 + i];              // Acc
          PRINT[i + 75] = BUF_2[K_2 + 2  + i];     
          PRINT[i + 77] = BUF_2[K_2 + 4  + i];
          
          PRINT[i + 79] = BUF_2[K_2 + 6  + i];         // Gyr
          PRINT[i + 81] = BUF_2[K_2 + 8  + i];     
          PRINT[i + 83] = BUF_2[K_2 + 10 + i];
          
          // SensFoot, Right ______________________________________________________
          // Sensor_7 _____________________________________________________________
          PRINT[i + 85] = BUF_3[K_3 + i];              // Acc
          PRINT[i + 87] = BUF_3[K_3 + 2  + i];     
          PRINT[i + 89] = BUF_3[K_3 + 4  + i];
          
          PRINT[i + 91] = BUF_3[K_3 + 6  + i];         // Gyr
          PRINT[i + 93] = BUF_3[K_3 + 8  + i];     
          PRINT[i + 95] = BUF_3[K_3 + 10 + i];
          //______________________________________________________________________
        }
        PRINT[97] = (uint8_t) 'L';
        
//        ///// Prova di corruzione dei pacchetti inviati al PC, per verificare il corretto
//        ///// funzionamento del meccanismo di ripristino del flusso di acquisizione dati
//        counter1++;
//        if (counter1 % 1000 == 1)
//        {
//          // Simulating an increasing number of lost bytes, not including start and stop bytes
//          int nLostBytes = counter1 / 1000;
//          for (int i = 20 ; i < 98 - nLostBytes; i++)
//          {
//            PRINT[i] = PRINT[i + nLostBytes];
//          }
//          PRINT[98 - nLostBytes - 1] = (uint8_t) 'L';
//          ble_mic_chars_received_uart_print(PRINT, (98 - nLostBytes));  
//        }
//        else if (counter1 % 1000 == 500)
//        {
//          // Simulating losing the stop byte
//          ble_mic_chars_received_uart_print(PRINT, 97);
//        }
//        else
//          /////
        
          // Serial print
          ble_mic_chars_received_uart_print(PRINT, 98);  
}

// ____________________________________________________________________________
/**@brief Time event handler
 * @details: This function is called at any tick of the clock. So it is called 
 *           every 9ms for the current settings. This function is UNRELATED to
 *           receiving data via Bluetooth. As soon as at least one device is
 *           connected it starts streaming via serial port. If data are 
 *           available in the buffers, they are sent, otherwise it sends zeros.
 */
static void app_timer_handler(void * p_context)
{
  if (sens_n >= 1) {
    // If data have been read and stored from all peripherals, then print.
    
    // Count the maximum number of packets that can be sent over uart, depending on the packets arrived from each connected sensor
    uint16_t nPackets = 0;
    uint16_t nPacketsTemp = 0;
    bool flag = true;
    
    for (int i = 0; i < 4; i++)
    {
      if (isSensConnected[i])
      {
        nPacketsTemp = packetsReceived[i] - packetsSent[i];
        if (flag)
        {
          // First of the connected sensors
          nPackets = nPacketsTemp;
          flag = false;
        }
        else if (nPacketsTemp < nPackets)
          nPackets = nPacketsTemp;
      }
    }
    
    if (nPackets > 20)
      nPackets = 20;
    
    if (nPackets > 0)
    {
      for (int j = 0; j < nPackets; j++)
      {
        // Send one data packet
        sendDataOverUart();
        
        // Updated index for printed values
        K_0 = K_0 + data_length_H;
        if (K_0 > store_sz - data_length_H) {
          K_0 = 0;
        }
        
        K_1 = K_1 + data_length_H;
        if (K_1 > store_sz - data_length_H) {
          K_1 = 0;
        }
        K_2 = K_2 + data_length_F;
        if (K_2 > store_sz - data_length_F) {
          K_2 = 0;
        }
        K_3 = K_3 + data_length_F;
        if (K_3 > store_sz - data_length_F) {
          K_3 = 0;
        }
        
        // Increase the counters packetsSent
        if (isSensConnected[0])
          packetsSent[0]++;
        if (isSensConnected[1])
          packetsSent[1]++;
        if (isSensConnected[2])
          packetsSent[2]++;
        if (isSensConnected[3])
          packetsSent[3]++;
      }
      
    }
    
    // If disconnected, stop blinking
    if (ble_conn_state_central_conn_count() == 0) bsp_board_led_off(CENTRAL_CONNECTED_LED);
  }
}
  


// ____________________________________________________________________________
/** @brief Function for initializing the timer.
 *  Function to initialize the timer. 
 */
static void timer_init(void)
{
  ret_code_t err_code;
  
  err_code = app_timer_init();                                       
  APP_ERROR_CHECK(err_code);
  
  err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED,
                              app_timer_handler);
  APP_ERROR_CHECK(err_code);
}

// ____________________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________
int main(void)
{
  // Initialize  
  lfclk_config();
  timer_init();
  leds_init();
  uart_init();
  db_discovery_init();
  power_management_init();
  ble_stack_init();
  gatt_init();
  mic_c_init();
  ble_conn_state_init();
  scan_init();
  
  // Start execution.  
  uint32_t err_code = app_timer_start(m_app_timer_id, TIM_INTERVAL, NULL);
  
  

  while (1) {
    idle_state_handle();
  }
}

// ____________________________________________________________________________
// ____________________________________________________________________________
// ____________________________________________________________________________

