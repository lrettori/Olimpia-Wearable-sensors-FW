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

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_MIC_C)
#include <stdlib.h>

#include "ble.h"
#include "ble_mic_c.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"

#define NRF_LOG_MODULE_NAME ble_mic_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

// ____________________________________________________________________________
/**@brief Function for intercepting the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
  ble_mic_c_t * p_ble_mic_c = (ble_mic_c_t *)p_ctx;
  NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", 
                conn_handle);
  if (p_ble_mic_c->error_handler != NULL)
  {
    p_ble_mic_c->error_handler(nrf_error);
  }
}

// ____________________________________________________________________________
void ble_mic_c_on_db_disc_evt(ble_mic_c_t * p_ble_mic_c, 
                              ble_db_discovery_evt_t * p_evt)
{
  ble_mic_c_evt_t mic_c_evt;
  memset(&mic_c_evt,0,sizeof(ble_mic_c_evt_t));
  ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;
  
  // Check if the MIC was discovered.
  if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
      &&  (p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_MIC_SERVICE)
      &&  (p_evt->params.discovered_db.srv_uuid.type == p_ble_mic_c->uuid_type))
  {
    for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
    {
      switch (p_chars[i].characteristic.uuid.uuid)
      {
      case BLE_UUID_MIC_RX_CHARACTERISTIC:
        mic_c_evt.handles.mic_rx_handle = p_chars[i].characteristic.handle_value;
        break;
      
      case BLE_UUID_MIC_TX_CHARACTERISTIC:
        mic_c_evt.handles.mic_tx_handle = p_chars[i].characteristic.handle_value;
        mic_c_evt.handles.mic_tx_cccd_handle = p_chars[i].cccd_handle;
        break;
      
      default:
        break;
      }
    }
    if (p_ble_mic_c->evt_handler != NULL)
    {
      mic_c_evt.conn_handle = p_evt->conn_handle;
      mic_c_evt.evt_type    = BLE_MIC_C_EVT_DISCOVERY_COMPLETE;
      p_ble_mic_c->evt_handler(p_ble_mic_c, &mic_c_evt);
    }
  }
}

// ____________________________________________________________________________
/**@brief     Function for handling Handle Value Notification received from the 
 *            SoftDevice.
 *
 * @details   This function uses the Handle Value Notification received from the 
 *            SoftDevice and checks if it is a notification of the MIC TX 
 *            characteristic from the peer. If it is, this function decodes the 
 *            data and sends it to the application.
 *            
 * @param[in] p_ble_mic_c Pointer to the MIC Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_mic_c_t * p_ble_mic_c, ble_evt_t const * p_ble_evt)
{
  // HVX can only occur from client sending.
  if (   (p_ble_mic_c->handles.mic_tx_handle != BLE_GATT_HANDLE_INVALID)
      && (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_mic_c->handles.mic_tx_handle)
      && (p_ble_mic_c->evt_handler != NULL))
  {
    ble_mic_c_evt_t ble_mic_c_evt;
    
    ble_mic_c_evt.evt_type = BLE_MIC_C_EVT_MIC_TX_EVT;
    ble_mic_c_evt.p_data   = (uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    ble_mic_c_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;
    ble_mic_c_evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle; 
    
    p_ble_mic_c->evt_handler(p_ble_mic_c, &ble_mic_c_evt);
//    NRF_LOG_DEBUG("Client sending data.");
  }
}

// ____________________________________________________________________________
uint32_t ble_mic_c_init(ble_mic_c_t * p_ble_mic_c, 
                        ble_mic_c_init_t * p_ble_mic_c_init)
{
  uint32_t      err_code;
  ble_uuid_t    ble_mic_uuid;
  ble_uuid128_t mic_base_uuid = MIC_BASE_UUID;
  
  VERIFY_PARAM_NOT_NULL(p_ble_mic_c);
  VERIFY_PARAM_NOT_NULL(p_ble_mic_c_init);
  VERIFY_PARAM_NOT_NULL(p_ble_mic_c_init->p_gatt_queue);
  
  err_code = sd_ble_uuid_vs_add(&mic_base_uuid, &p_ble_mic_c->uuid_type);
  VERIFY_SUCCESS(err_code);
  
  ble_mic_uuid.type = p_ble_mic_c->uuid_type;
  ble_mic_uuid.uuid = BLE_UUID_MIC_SERVICE;
  
  p_ble_mic_c->conn_handle           = BLE_CONN_HANDLE_INVALID;
  p_ble_mic_c->evt_handler           = p_ble_mic_c_init->evt_handler;
  p_ble_mic_c->error_handler         = p_ble_mic_c_init->error_handler;
  p_ble_mic_c->handles.mic_tx_handle = BLE_GATT_HANDLE_INVALID;
  p_ble_mic_c->handles.mic_rx_handle = BLE_GATT_HANDLE_INVALID;
  p_ble_mic_c->p_gatt_queue          = p_ble_mic_c_init->p_gatt_queue;
  
  return ble_db_discovery_evt_register(&ble_mic_uuid);
}

// ____________________________________________________________________________
void ble_mic_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
  ble_mic_c_t * p_ble_mic_c = (ble_mic_c_t *)p_context;
  
  if ((p_ble_mic_c == NULL) || (p_ble_evt == NULL))
  {
    return;
  }
  
  if (  (p_ble_mic_c->conn_handle == BLE_CONN_HANDLE_INVALID)
      ||(p_ble_mic_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle))
  {
    return;
  }
  
  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GATTC_EVT_HVX:
    on_hvx(p_ble_mic_c, p_ble_evt);
    break;
  
  case BLE_GAP_EVT_DISCONNECTED:
    if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_mic_c->conn_handle
        && p_ble_mic_c->evt_handler != NULL)
    {
      ble_mic_c_evt_t mic_c_evt;
      mic_c_evt.evt_type = BLE_MIC_C_EVT_DISCONNECTED;
      
      p_ble_mic_c->conn_handle = BLE_CONN_HANDLE_INVALID;
      p_ble_mic_c->evt_handler(p_ble_mic_c, &mic_c_evt);
    }
    break;
  
  default:
    // No implementation needed.
    break;
  }
}

// ____________________________________________________________________________
/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_configure(ble_mic_c_t * p_ble_mic_c, bool notification_enable)
{
  nrf_ble_gq_req_t cccd_req;
  uint8_t          cccd[BLE_CCCD_VALUE_LEN];
  uint16_t         cccd_val = notification_enable ? BLE_GATT_HVX_NOTIFICATION : 0;
  
  memset(&cccd_req, 0, sizeof(nrf_ble_gq_req_t));

  cccd[0] = LSB_16(cccd_val);
  cccd[1] = MSB_16(cccd_val);

  cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
  cccd_req.error_handler.cb            = gatt_error_handler;
  cccd_req.error_handler.p_ctx         = p_ble_mic_c;
  cccd_req.params.gattc_write.handle   = p_ble_mic_c->handles.mic_tx_cccd_handle;
  cccd_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
  cccd_req.params.gattc_write.offset   = 0;
  cccd_req.params.gattc_write.p_value  = cccd;
  cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
  cccd_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
  
  return nrf_ble_gq_item_add(p_ble_mic_c->p_gatt_queue, &cccd_req, 
                             p_ble_mic_c->conn_handle);
}

// ____________________________________________________________________________
uint32_t ble_mic_c_tx_notif_enable(ble_mic_c_t * p_ble_mic_c)
{
  VERIFY_PARAM_NOT_NULL(p_ble_mic_c);
  
  if (  (p_ble_mic_c->conn_handle == BLE_CONN_HANDLE_INVALID)
      ||(p_ble_mic_c->handles.mic_tx_cccd_handle == BLE_GATT_HANDLE_INVALID))
  {
    return NRF_ERROR_INVALID_STATE;
  }
  return cccd_configure(p_ble_mic_c, true);
}

// ____________________________________________________________________________
uint32_t ble_mic_c_string_send(ble_mic_c_t * p_ble_mic_c, 
                               uint8_t * p_string, uint16_t length)
{
  VERIFY_PARAM_NOT_NULL(p_ble_mic_c);
  nrf_ble_gq_req_t write_req;
  
  memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));
  
  if (length > BLE_MIC_MAX_DATA_LEN)
  {
//    NRF_LOG_WARNING("Content too long.");
    return NRF_ERROR_INVALID_PARAM;
  }
  if (p_ble_mic_c->conn_handle == BLE_CONN_HANDLE_INVALID)
  {
//    NRF_LOG_WARNING("Connection handle invalid.");
    return NRF_ERROR_INVALID_STATE;
  }
  
  write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
  write_req.error_handler.cb            = gatt_error_handler;
  write_req.error_handler.p_ctx         = p_ble_mic_c;
  write_req.params.gattc_write.handle   = p_ble_mic_c->handles.mic_rx_handle;
  write_req.params.gattc_write.len      = length;
  write_req.params.gattc_write.offset   = 0;
  write_req.params.gattc_write.p_value  = p_string;
  write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_CMD;
  write_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
  
  return nrf_ble_gq_item_add(p_ble_mic_c->p_gatt_queue, &write_req, 
                             p_ble_mic_c->conn_handle);
}

// ____________________________________________________________________________
uint32_t ble_mic_c_handles_assign(ble_mic_c_t               * p_ble_mic,
                                  uint16_t                    conn_handle,
                                  ble_mic_c_handles_t const * p_peer_handles)
{
  VERIFY_PARAM_NOT_NULL(p_ble_mic);
  p_ble_mic->conn_handle = conn_handle;
  if (p_peer_handles != NULL)
  {
    p_ble_mic->handles.mic_tx_cccd_handle = p_peer_handles->mic_tx_cccd_handle;
    p_ble_mic->handles.mic_tx_handle      = p_peer_handles->mic_tx_handle;
    p_ble_mic->handles.mic_rx_handle      = p_peer_handles->mic_rx_handle;
  }
  return nrf_ble_gq_conn_handle_register(p_ble_mic->p_gatt_queue, conn_handle);
}

#endif // NRF_MODULE_ENABLED(BLE_MIC_C)
