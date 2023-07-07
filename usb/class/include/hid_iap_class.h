/**
  **************************************************************************
  * @file     hid_iap_class.h
  * @brief    usb hid iap header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

 /* define to prevent recursive inclusion -------------------------------------*/
#ifndef __HID_IAP_CLASS_H
#define __HID_IAP_CLASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_std.h"
#include "usbd_core.h"


/** @addtogroup AT32F425_middlewares_usbd_class
  * @{
  */

/** @addtogroup USB_hid_iap_class
  * @{
  */

/** @defgroup USB_hid_iap_class_definition
  * @{
  */


#define USBD_HIDIAP_IN_EPT                  0x81
#define USBD_HIDIAP_OUT_EPT                 0x01

#define USBD_HIDIAP_IN_MAXPACKET_SIZE       0x40
#define USBD_HIDIAP_OUT_MAXPACKET_SIZE      0x40

#define FLASH_SIZE_REG()                 ((*(uint32_t *)0x1FFFF7E0) & 0xFFFF) /*Get Flash size*/
#define KB_TO_B(kb)                      ((kb) << 10)

#define SECTOR_SIZE_1K                   0x400

/**
  * @brief iap command
  */
#define IAP_CMD_INFO      0x01
#define IAP_CMD_FW_START  0x02
#define IAP_CMD_START_APP 0x03

#define HID_IAP_BUFFER_LEN               8192
#define IAP_UPGRADE_COMPLETE_FLAG        0x41544B38
#define CONVERT_ENDIAN(dwValue)          ((dwValue >> 24) | ((dwValue >> 8) & 0xFF00) | \
                                         ((dwValue << 8) & 0xFF0000) | (dwValue << 24) )

#define IAP_FAIL                         0x00
#define IAP_ERASE_OK                     0x01
#define IAP_CRC_OK                       0x02
#define IAP_CRC_FAIL                     0x03
#define IAP_TIMEOUT                      0x04

typedef enum
{
  IAP_SUCCESS,
  IAP_WAIT,
  IAP_FAILED
}iap_result_type;

typedef enum
{
  IAP_STS_IDLE,
  IAP_STS_FW_UPDATE,
  IAP_STS_JMP_WAIT,
  IAP_STS_JMP,
}iap_machine_state_type;

typedef struct
{
  uint8_t iap_rx[USBD_HIDIAP_OUT_MAXPACKET_SIZE];
  uint8_t iap_tx[USBD_HIDIAP_IN_MAXPACKET_SIZE];

  uint32_t tx_length;

  uint32_t app_address;
  uint32_t iap_address;
  uint32_t flash_flag_address;
  uint32_t ram_flag_address;

  uint32_t flash_start_address;
  uint32_t flash_end_address;

  uint32_t sector_size;
  uint32_t flash_size;

  uint32_t respond_flag;

  uint8_t g_rxhid_buff[USBD_HIDIAP_OUT_MAXPACKET_SIZE];
  uint32_t hid_protocol;
  uint32_t hid_set_idle;
  uint32_t alt_setting;
  uint32_t hid_state;
  uint8_t hid_set_report[64];

  uint32_t fw_crc32;
  uint16_t fw_pack_count;
  uint16_t recv_pack_count;

  usbd_core_type* device;
  uint8_t timeout;
  uint8_t tmr_started;

  iap_machine_state_type state;
}iap_info_type;

extern usbd_class_handler hid_iap_class_handler;
extern iap_info_type iap_info;
usb_sts_type usb_iap_class_send_report(void *udev, uint8_t *report, uint16_t len);
iap_result_type usbd_hid_iap_process(void *udev, uint8_t *report, uint16_t len);
void usbd_hid_iap_in_complete(void *udev);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif


