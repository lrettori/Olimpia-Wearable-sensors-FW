 /**
 * Copyright (c) 2012 - 2021, Nordic Semiconductor ASA
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
 *
 * ____________________________________________________________________________
 */
/**@file
 *
 * @defgroup ble_mic Nordic UART Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Nordic UART Service implementation.
 *
 * @details The Nordic UART Service is a simple GATT-based service with TX and 
 *          RX characteristics.
 *          Data received from the peer is passed to the application, and the 
 *          data received from the application of this service is sent to the 
 *          peer as Handle Value Notifications. This module demonstrates how to 
 *          implement a custom GATT-based service and characteristics using the 
 *          SoftDevice. The service is used by the application to send and 
 *          receive ASCII text strings to and from the peer.
 *
 * @note    The application must register this module as BLE event observer 
 *          using the NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_mic_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_MIC_BLE_OBSERVER_PRIO,
 *                                   ble_mic_on_ble_evt, &instance);
 *          @endcode
 */
#ifndef BLE_MIC_H__
#define BLE_MIC_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

// ____________________________________________________________________________
/**@brief   Macro for defining a ble_mic instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _mic_max_clients Maximum number of MIC clients connected at a 
 *            time.
 * @hideinitializer
 */
#define BLE_MIC_DEF(_name, _mic_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_mic_max_clients),                  \
                             sizeof(ble_mic_client_context_t));   \
    static ble_mic_t _name =                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_MIC_BLE_OBSERVER_PRIO,               \
                         ble_mic_on_ble_evt,                      \
                         &_name)

#define BLE_UUID_MIC_SERVICE 0x0001 /**< The UUID of the Nordic UART Service. */

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

#define BLE_MIC_MAX_BUFFERED_DATA               600
                           
// ____________________________________________________________________________
/**@brief   Maximum length of data (in bytes) that can be transmitted to the 
 *          peer by the Nordic UART service module. 
 */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_MIC_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_MIC_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

// ____________________________________________________________________________
/**@brief   Nordic UART Service event types. */
typedef enum
{
  BLE_MIC_EVT_RX_DATA,                                                          /**< Data received. */
  BLE_MIC_EVT_TX_RDY,                                                           /**< Service is ready to accept new data to be transmitted. */
  BLE_MIC_EVT_COMM_STARTED,                                                     /**< Notification has been enabled. */
  BLE_MIC_EVT_COMM_STOPPED,                                                     /**< Notification has been disabled. */
} ble_mic_evt_type_t;

// ____________________________________________________________________________
/* Forward declaration of the ble_mic_t type. */
typedef struct ble_mic_s ble_mic_t;

// ____________________________________________________________________________
/**@brief   Nordic UART Service @ref BLE_MIC_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when 
 *          @ref BLE_MIC_EVT_RX_DATA occurs.
 */
typedef struct
{
  uint8_t const * p_data;                                                       /**< A pointer to the buffer with received data. */
  uint16_t        length;                                                       /**< Length of received data. */
} ble_mic_evt_rx_data_t;

// ____________________________________________________________________________
/**@brief Nordic UART Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
  bool is_notification_enabled;                                                 /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_mic_client_context_t;

// ____________________________________________________________________________
/**@brief   Nordic UART Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
  ble_mic_evt_type_t         type;                                              /**< Event type. */
  ble_mic_t                * p_mic;                                             /**< A pointer to the instance. */
  uint16_t                   conn_handle;                                       /**< Connection handle. */
  ble_mic_client_context_t * p_link_ctx;                                        /**< A pointer to the link context. */
  union
  {
    ble_mic_evt_rx_data_t rx_data;                                              /**< @ref BLE_MIC_EVT_RX_DATA event data. */
  } params;
} ble_mic_evt_t;

// ____________________________________________________________________________
/**@brief Nordic UART Service event handler type. */
typedef void (* ble_mic_data_handler_t) (ble_mic_evt_t * p_evt);

// ____________________________________________________________________________
/**@brief   Nordic UART Service initialization structure.
 *
 * @details This structure contains the initialization information for the 
 *          service. The application must fill this structure and pass it to the 
 *          service using the @ref ble_mic_init function.
 */
typedef struct
{
  ble_mic_data_handler_t       data_handler;                                    /**< Event handler to be called for handling received data. */
  bool                         is_sensor_contact_supported;
  security_req_t               mic_cccd_wr_sec;                                 /**< Security requirement for writing the HRM characteristic CCCD. */
  security_req_t               bsl_rd_sec;  
} ble_mic_init_t;

// ____________________________________________________________________________
/**@brief   Nordic UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_mic_s
{
  bool                            is_expended_energy_supported;
  bool                            is_sensor_contact_supported;
  uint8_t                         uuid_type;                                    /**< UUID type for Nordic UART Service Base UUID. */
  uint16_t                        service_handle;                               /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
  ble_gatts_char_handles_t        tx_handles;                                   /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
  ble_gatts_char_handles_t        rx_handles;                                   /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
  uint16_t                        conn_handle;                                  
  bool                            is_sensor_contact_detected;
  uint16_t                        data_buf[BLE_MIC_MAX_BUFFERED_DATA];
  uint16_t                        data_buf_count;
  blcm_link_ctx_storage_t * const p_link_ctx_storage;                           /**< Pointer to link context storage with handles of all current connections and its context. */
  ble_mic_data_handler_t          data_handler;                                 /**< Event handler to be called for handling received data. */
};

// ____________________________________________________________________________
/**@brief   Function for initializing the Nordic UART Service.
 *
 * @param[out] p_mic      Nordic UART Service structure. This structure must be 
 *                        supplied by the application. It is initialized by this 
 *                        function and will later be used to identify this 
 *                        particular service instance.
 * @param[in] p_mic_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, 
 *         an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_mic or p_mic_init is NULL.
 */
uint32_t ble_mic_init(ble_mic_t * p_mic, ble_mic_init_t const * p_mic_init);

// ____________________________________________________________________________
/**@brief   Function for handling the Nordic UART Service's BLE events.
 *
 * @details The Nordic UART Service expects the application to call this 
 *          function each time an event is received from the SoftDevice. 
 *          This function processes the event if it is relevant and calls the 
 *          Nordic UART Service event handler of the application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     Nordic UART Service structure.
 */
void ble_mic_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

// ____________________________________________________________________________
/**@brief   Function for sending a data to the peer.
 *
 * @details This function sends the input string as an RX characteristic
 *          notification to the peer.
 *
 * @param[in]     p_mic       Pointer to the Nordic UART Service structure.
 * @param[in]     p_data      String to be sent.
 * @param[in,out] p_length    Pointer Length of the string. Amount of sent 
 *                            bytes.
 * @param[in]     conn_handle Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error 
 *                     code is returned.
 */
uint32_t ble_mic_data_send(ble_mic_t * p_mic,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle);


#ifdef __cplusplus
}
#endif

#endif // BLE_MIC_H__

/** @} */
