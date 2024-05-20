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

// ____________________________________________________________________________
#include <stdio.h>                              

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_MIC)
#include "ble.h"
#include "ble_mic.h"
#include "ble_srv_common.h"

#define NRF_LOG_MODULE_NAME ble_mic
#if BLE_MIC_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_MIC_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_MIC_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_MIC_CONFIG_DEBUG_COLOR
#else // BLE_MIC_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_MIC_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#define BLE_UUID_MIC_TX_CHARACTERISTIC 0x0003                                   /**< The UUID of the TX Characteristic. */
#define BLE_UUID_MIC_RX_CHARACTERISTIC 0x0002                                   /**< The UUID of the RX Characteristic. */

#define BLE_MIC_MAX_RX_CHAR_LEN        BLE_MIC_MAX_DATA_LEN                     /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_MIC_MAX_TX_CHAR_LEN        BLE_MIC_MAX_DATA_LEN                     /**< Maximum length of the TX Characteristic (in bytes). */

#define MIC_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */

// ____________________________________________________________________________

// ____________________________________________________________________________
/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the 
 *        SoftDevice.
 *
 * @param[in] p_mic     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_mic_t * p_mic, ble_evt_t const * p_ble_evt)
{
  ret_code_t                 err_code;
  ble_mic_evt_t              evt;
  ble_gatts_value_t          gatts_val;
  uint8_t                    cccd_value[2];
  ble_mic_client_context_t * p_client = NULL;
  
  err_code = blcm_link_ctx_get(p_mic->p_link_ctx_storage,
                               p_ble_evt->evt.gap_evt.conn_handle,
                               (void *) &p_client);
  if (err_code != NRF_SUCCESS)
    {
      NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                    p_ble_evt->evt.gap_evt.conn_handle);
    }
  
  /* Check the hosts CCCD value to inform of readiness to send data using the RX 
     characteristic */
  memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
  gatts_val.p_value = cccd_value;
  gatts_val.len     = sizeof(cccd_value);
  gatts_val.offset  = 0;
  
  err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                    p_mic->tx_handles.cccd_handle,
                                    &gatts_val);
  
  if ((err_code == NRF_SUCCESS)     &&
      (p_mic->data_handler != NULL) &&
       ble_srv_is_notification_enabled(gatts_val.p_value))
  {
    if (p_client != NULL)
    {
      p_client->is_notification_enabled = true;
    }
    memset(&evt, 0, sizeof(ble_mic_evt_t));
    evt.type        = BLE_MIC_EVT_COMM_STARTED;
    evt.p_mic       = p_mic;
    evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    evt.p_link_ctx  = p_client;
    
    p_mic->data_handler(&evt);
  }
}

// ____________________________________________________________________________
/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the 
 *        SoftDevice.
 *
 * @param[in] p_mic     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_mic_t * p_mic, ble_evt_t const * p_ble_evt)
{
  ret_code_t                    err_code;
  ble_mic_evt_t                 evt;
  ble_mic_client_context_t    * p_client;
  ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  
  err_code = blcm_link_ctx_get(p_mic->p_link_ctx_storage,
                               p_ble_evt->evt.gatts_evt.conn_handle,
                               (void *) &p_client);
  
  if (err_code != NRF_SUCCESS)
  {
    NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                  p_ble_evt->evt.gatts_evt.conn_handle);
  }
  
  memset(&evt, 0, sizeof(ble_mic_evt_t));
  evt.p_mic       = p_mic;
  evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
  evt.p_link_ctx  = p_client;
  
  if ((p_evt_write->handle == p_mic->tx_handles.cccd_handle) &&
      (p_evt_write->len == 2))
  {
    if (p_client != NULL)
    {
      if (ble_srv_is_notification_enabled(p_evt_write->data))
      {
        p_client->is_notification_enabled = true;
        evt.type                          = BLE_MIC_EVT_COMM_STARTED;
      }
      else
      {
        p_client->is_notification_enabled = false;
        evt.type                          = BLE_MIC_EVT_COMM_STOPPED;
      }
      if (p_mic->data_handler != NULL)
      {
        p_mic->data_handler(&evt);
      }
    }
  }
  else if ((p_evt_write->handle == p_mic->rx_handles.value_handle) &&
           (p_mic->data_handler != NULL))
  {
    evt.type                  = BLE_MIC_EVT_RX_DATA;
    evt.params.rx_data.p_data = p_evt_write->data;
    evt.params.rx_data.length = p_evt_write->len;
    
    p_mic->data_handler(&evt);
  }
  else
  {
    // Do Nothing. This event is not relevant for this service.
  }
}

// ____________________________________________________________________________
/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event 
 *        from the SoftDevice.
 *
 * @param[in] p_mic     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_hvx_tx_complete(ble_mic_t * p_mic, ble_evt_t const * p_ble_evt)
{
  ret_code_t                 err_code;
  ble_mic_evt_t              evt;
  ble_mic_client_context_t * p_client;
  
  err_code = blcm_link_ctx_get(p_mic->p_link_ctx_storage,
                               p_ble_evt->evt.gatts_evt.conn_handle,
                               (void *) &p_client);
  if (err_code != NRF_SUCCESS)
  {
    NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                  p_ble_evt->evt.gatts_evt.conn_handle);
    return;
  }
  if ((p_client->is_notification_enabled) && (p_mic->data_handler != NULL))
  {
    memset(&evt, 0, sizeof(ble_mic_evt_t));
    evt.type        = BLE_MIC_EVT_TX_RDY;
    evt.p_mic       = p_mic;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    evt.p_link_ctx  = p_client;
    
    p_mic->data_handler(&evt);
  }
}

// ____________________________________________________________________________
void ble_mic_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
  if ((p_context == NULL) || (p_ble_evt == NULL))
  {
    return;
  }
  ble_mic_t * p_mic = (ble_mic_t *)p_context;
  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
    on_connect(p_mic, p_ble_evt);
    break;
  
  case BLE_GATTS_EVT_WRITE:
    on_write(p_mic, p_ble_evt);
    break;
  
  case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    on_hvx_tx_complete(p_mic, p_ble_evt);
    break;
  
  default:
    // No implementation needed.
    break;
  }
}

// ____________________________________________________________________________
uint32_t ble_mic_init(ble_mic_t * p_mic, ble_mic_init_t const * p_mic_init)
{
  ret_code_t            err_code;
  ble_uuid_t            ble_uuid;
  ble_uuid128_t         mic_base_uuid = MIC_BASE_UUID;
  ble_add_char_params_t add_char_params;
    
  VERIFY_PARAM_NOT_NULL(p_mic);
  VERIFY_PARAM_NOT_NULL(p_mic_init);
  
  // Initialize the service structure.
  p_mic->data_handler                = p_mic_init->data_handler;
  p_mic->is_sensor_contact_supported = p_mic_init->is_sensor_contact_supported;
  p_mic->conn_handle                 = BLE_CONN_HANDLE_INVALID;
  p_mic->is_sensor_contact_detected  = false;
  
  // Add a custom base UUID.
  err_code = sd_ble_uuid_vs_add(&mic_base_uuid, &p_mic->uuid_type);
  VERIFY_SUCCESS(err_code);
  
  ble_uuid.type = p_mic->uuid_type;
  ble_uuid.uuid = BLE_UUID_MIC_SERVICE;
  
  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                      &ble_uuid,
                                      &p_mic->service_handle);
  VERIFY_SUCCESS(err_code);
  
  // Add the RX Characteristic.
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid                     = BLE_UUID_MIC_RX_CHARACTERISTIC;
  add_char_params.uuid_type                = p_mic->uuid_type;
  add_char_params.max_len                  = BLE_MIC_MAX_RX_CHAR_LEN;
  add_char_params.init_len                 = sizeof(uint8_t);
  add_char_params.is_var_len               = true;
  add_char_params.char_props.write         = 1;
  add_char_params.char_props.write_wo_resp = 1;
  
  add_char_params.cccd_write_access        = p_mic_init->mic_cccd_wr_sec;
  
  add_char_params.read_access  = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;
  
  err_code = characteristic_add(p_mic->service_handle, &add_char_params, 
                                &p_mic->rx_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
  
  // Add the TX Characteristic.
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid              = BLE_UUID_MIC_TX_CHARACTERISTIC;
  add_char_params.uuid_type         = p_mic->uuid_type;
  add_char_params.max_len           = BLE_MIC_MAX_TX_CHAR_LEN;
  add_char_params.init_len          = sizeof(uint8_t);
  add_char_params.is_var_len        = true;
  add_char_params.char_props.notify = 1;

  add_char_params.read_access       = SEC_OPEN;
  add_char_params.write_access      = SEC_OPEN;
  add_char_params.cccd_write_access = SEC_OPEN;
  
  return characteristic_add(p_mic->service_handle, &add_char_params, 
                            &p_mic->tx_handles);
  /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}

// ____________________________________________________________________________
uint32_t ble_mic_data_send(ble_mic_t * p_mic,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle)
{
  ret_code_t                 err_code;
  ble_gatts_hvx_params_t     hvx_params;
  ble_mic_client_context_t * p_client;
  
  VERIFY_PARAM_NOT_NULL(p_mic);
  err_code = blcm_link_ctx_get(p_mic->p_link_ctx_storage, conn_handle, 
                               (void *) &p_client);
  VERIFY_SUCCESS(err_code);
  
  // Send value if connected and notifying 
  if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
  {
    return NRF_ERROR_NOT_FOUND;
  }
  if (!p_client->is_notification_enabled)
  {
    return NRF_ERROR_INVALID_STATE;
  }
  if (*p_length > BLE_MIC_MAX_DATA_LEN)
  {
    return NRF_ERROR_INVALID_PARAM;
  }
  
  memset(&hvx_params, 0, sizeof(hvx_params));
  
  hvx_params.handle = p_mic->tx_handles.value_handle;
  hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
  hvx_params.p_data = p_data;
  hvx_params.p_len  = p_length;    
  return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}

// ____________________________________________________________________________
/*
 *
 */
void ble_mic_data_buf_add(ble_mic_t * p_mic, uint16_t data_buf)
{
  if (p_mic->data_buf_count == BLE_MIC_MAX_BUFFERED_DATA)
  {
    // The data buffere is full, delete the oldest value
    memmove(&p_mic->data_buf[0], &p_mic->data_buf[1],
            (BLE_MIC_MAX_BUFFERED_DATA - 1) * sizeof(uint16_t));
    p_mic->data_buf_count--;
  }
  // Add new value
  p_mic->data_buf[p_mic->data_buf_count++] = data_buf;
}

// ____________________________________________________________________________
/*
 *
 */
bool ble_mic_data_buffer_is_full(ble_mic_t * p_mic)
{
  return (p_mic->data_buf_count == BLE_MIC_MAX_BUFFERED_DATA);
}

// ____________________________________________________________________________

#endif // NRF_MODULE_ENABLED(BLE_MIC)
