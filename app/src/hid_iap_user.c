/**
  **************************************************************************
  * @file     hid_iap_user.c
  * @brief    usb hid iap user file
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

#include "hid_iap_user.h"
#include "hid_iap_class.h"
#include "hid_iap_desc.h"
#include "string.h"

void (*pftarget)(void);

/**
  * @brief  jump to app
  * @param  none
  * @retval none
  */
void jump_to_app(uint32_t address)
{
  uint32_t stkptr, jumpaddr;
  stkptr = *(uint32_t *)address;
  jumpaddr = *(uint32_t *)(address + sizeof(uint32_t));

  /* disable nvic irq and periph clock, clear pending */
  nvic_irq_disable(OTGFS1_IRQn);

  __NVIC_ClearPendingIRQ(OTGFS1_IRQn);

  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, FALSE);

  crm_periph_reset(CRM_OTGFS1_PERIPH_RESET, TRUE);
  crm_periph_reset(CRM_OTGFS1_PERIPH_RESET, FALSE);

  __set_MSP(stkptr);
  pftarget = (void (*) (void))jumpaddr;
  pftarget();
}

/**
  * @brief  clear iap upgrade flag
  * @param  none
  * @retval none
  */
void iap_clear_upgrade_flag(void)
{
  *((uint32_t*)iap_info.ram_flag_address) = 0;

  flash_unlock();
  flash_sector_erase(iap_info.flash_flag_address);
  flash_lock();
}

/**
  * @brief  set iap upgrade complete flag
  * @param  none
  * @retval none
  */
void iap_set_upgrade_flag(void)
{
  *((uint32_t*)iap_info.ram_flag_address) = IAP_UPGRADE_COMPLETE_FLAG;

  flash_unlock();
  flash_word_program(iap_info.flash_flag_address, IAP_UPGRADE_COMPLETE_FLAG);
  flash_lock();
}

/**
  * @brief  get iap upgrade complete flag
  * @param  none
  * @retval the status of the iap flag
  */
iap_result_type iap_get_upgrade_flag(void)
{
    uint32_t ram_flag = iap_info.ram_flag_address;
    uint32_t flash_flag = iap_info.flash_flag_address;

    if((*((uint32_t *)flash_flag) ||
	*((uint32_t *)ram_flag))
       == IAP_UPGRADE_COMPLETE_FLAG)
  {
    return IAP_SUCCESS;
  }
  else
  {
    return IAP_FAILED;
  }
}

/**
  * @brief  set iap upgrade complete flag
  * @param  none
  * @retval none
  */
static void iap_erase_sector(uint32_t address)
{
  flash_unlock();
  flash_sector_erase(address);
  flash_lock();
}

/**
  * @brief  crc cal
  * @param  addr: start address
  * @param  block_count: number of iap data blocks
  * @retval crc value
  */
static uint32_t crc32_cal(uint32_t addr, uint16_t pack_count)
{
    uint32_t *paddr = (uint32_t *)addr;
    uint32_t wlen = pack_count * IAP_PACKET_LENGTH / sizeof(uint32_t);
    uint32_t value, i_index = 0;
    crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
    crc_data_reset();

    for(i_index = 0; i_index < wlen; i_index ++)
    {
	value = *paddr;
	crc_one_word_calculate(CONVERT_ENDIAN(value));

	paddr ++;
    }
    return crc_data_get();
}

/**
  * @brief  iap init
  * @param  none
  * @retval none
  */
void iap_init(void)
{
  iap_info.sector_size = SECTOR_SIZE_1K;
  iap_info.flash_size = KB_TO_B(FLASH_SIZE_REG());

  iap_info.flash_start_address = FLASH_BASE;
  iap_info.flash_end_address = iap_info.flash_start_address + iap_info.flash_size;

  iap_info.app_address = FLASH_APP_ADDRESS;
  iap_info.flash_flag_address = FLASH_UPGRADE_FLAG_ADDRESS;
  iap_info.ram_flag_address = RAM_UPGRADE_FLAG_ADDRESS;

  iap_info.state = IAP_STS_IDLE;
  iap_info.fifo_length = 0;
  iap_info.fw_crc32 = 0;
  iap_info.fw_pack_count = 0;
  iap_info.recv_pack_count = 0;
}

/*
  * @brief  iap respond
  * @param  res_buf: data buffer pointer.
  * @param  iap_cmd: iap command
  * @param  result: iap result
  * @retval none
  */
static void iap_respond(uint8_t *res_buf, uint8_t iap_cmd, uint8_t result)
{
    uint8_t fill_pos = 0;

    res_buf[0] = 0x01;
    res_buf[1] = iap_cmd;
    fill_pos += 2;

    if (iap_cmd == IAP_CMD_INFO) {
	fill_pos += INFO_CMD_RESPONSE_LENGTH;
    }
    else {
	if (iap_cmd != IAP_CMD_START_APP)
	    fill_pos += sizeof(result);
    }

  while (fill_pos != IAP_PACKET_LENGTH) {
      res_buf[fill_pos] = 0xFF;
      fill_pos++;
  }

  iap_info.respond_flag = 1;
}

/**
  * @brief  iap info function
  * @param  none
  * @retval none
  */
static void iap_inform() {
    uint32_t cpu_id_0, cpu_id_1, cpu_id_2;
    uint32_t app_ver_high, app_ver_low;

    if ( iap_info.state == IAP_STS_FW_UPDATE)
	return;

    cpu_id_0 = *(uint32_t*)MCU_ID1;
    cpu_id_1 = *(uint32_t*)MCU_ID2;
    cpu_id_2 = *(uint32_t*)MCU_ID3;
    app_ver_high = *(uint32_t*)APP_VERSION_HIGH_ADDRESS;
    app_ver_low = *(uint32_t*)APP_VERSION_LOW_ADDRESS;

    iap_respond(iap_info.iap_tx, IAP_CMD_INFO, 0);
    iap_info.iap_tx[2] = (uint8_t)((cpu_id_0 >> 24) & 0xFF);
    iap_info.iap_tx[3] = (uint8_t)((cpu_id_0 >> 16) & 0xFF);
    iap_info.iap_tx[4] = (uint8_t)((cpu_id_0 >> 8) & 0xFF);
    iap_info.iap_tx[5] = (uint8_t)((cpu_id_0) & 0xFF);
    iap_info.iap_tx[6] = (uint8_t)((cpu_id_1 >> 24) & 0xFF);
    iap_info.iap_tx[7] = (uint8_t)((cpu_id_1 >> 16) & 0xFF);
    iap_info.iap_tx[8] = (uint8_t)((cpu_id_1 >> 8) & 0xFF);
    iap_info.iap_tx[9] = (uint8_t)((cpu_id_1) & 0xFF);
    iap_info.iap_tx[10] = (uint8_t)((cpu_id_2 >> 24) & 0xFF);
    iap_info.iap_tx[11] = (uint8_t)((cpu_id_2 >> 16) & 0xFF);
    iap_info.iap_tx[12] = (uint8_t)((cpu_id_2 >> 8) & 0xFF);
    iap_info.iap_tx[13] = (uint8_t)((cpu_id_2) & 0xFF);
    iap_info.iap_tx[14] = (uint8_t)((app_ver_high >> 8) & 0xFF);
    iap_info.iap_tx[15] = (uint8_t)((app_ver_high) & 0xFF);
    iap_info.iap_tx[16] = (uint8_t)((app_ver_low >> 24) & 0xFF);
    iap_info.iap_tx[17] = (uint8_t)((app_ver_low >> 16) & 0xFF);
    iap_info.iap_tx[18] = (uint8_t)((app_ver_low >> 8) & 0xFF);
    iap_info.iap_tx[19] = (uint8_t)((app_ver_low) & 0xFF);
}

/**
  * @brief  iap start function
  * @param  none
  * @retval none
  */
static iap_result_type iap_start(uint8_t *pdata)
{
    flash_status_type status = FLASH_OPERATE_DONE;
    uint32_t address = iap_info.app_address;

    iap_info.state = IAP_STS_FW_UPDATE;

    while (address < iap_info.flash_end_address) {
	iap_erase_sector(address);

	if (status != FLASH_OPERATE_DONE) {
	    iap_respond(iap_info.iap_tx, IAP_CMD_FW_START, IAP_FAIL);
	    return IAP_FAILED;
	}
	address += iap_info.sector_size;
    }

    iap_init();
    iap_info.fw_pack_count = (pdata[2] << 8 | pdata[3]);
    iap_info.fw_crc32 = (pdata[4] << 24) | (pdata[5] << 16) |
	(pdata[6] << 8) | pdata[7];

    iap_respond(iap_info.iap_tx, IAP_CMD_FW_START, IAP_ERASE_OK);
    return IAP_SUCCESS;
}

/*
  * @brief  iap finish
  * @param  none
  * @retval none
  */
static void iap_finish()
{
    uint32_t crc32_value;

    crc32_value = crc32_cal(iap_info.app_address, iap_info.fw_pack_count);

    if (crc32_value != iap_info.fw_crc32) {
	iap_respond(iap_info.iap_tx, IAP_CMD_FW_START, IAP_FAIL);
	return;
    }

    iap_set_upgrade_flag();
    iap_respond(iap_info.iap_tx, IAP_CMD_FW_START, IAP_CRC_OK);
}

/*
  * @brief  iap data write
  * @param  pdata: data buffer pointer.
  * @param  len: buffer length
  * @retval the result of the address parse
  */
static iap_result_type iap_data_write(uint8_t *pdata, uint32_t len)
{
    uint32_t *pbuf;
    uint32_t i_index = 0;
    uint8_t  fw_complete = 0;

    iap_info.recv_pack_count++;

    if(len + iap_info.fifo_length <= HID_IAP_BUFFER_LEN)
    {
	for(i_index = 0; i_index < len; i_index++)
	{
	    iap_info.iap_fifo[iap_info.fifo_length++] = pdata[i_index];
	}
    }

    if (iap_info.recv_pack_count == iap_info.fw_pack_count) {
	while (iap_info.fifo_length != HID_IAP_BUFFER_LEN) {
	    iap_info.iap_fifo[iap_info.fifo_length++] = 0xFF;
	}
	fw_complete = 1;
    }

    /* buffer full */
    if(iap_info.fifo_length == HID_IAP_BUFFER_LEN)
    {
	flash_unlock();
	pbuf = (uint32_t *)iap_info.iap_fifo;
	for(i_index = 0; i_index < iap_info.fifo_length / sizeof(uint32_t); i_index ++)
	{
	    flash_word_program(iap_info.app_address, *pbuf++);
	    iap_info.app_address += 4;
	}
	flash_lock();

	iap_info.fifo_length = 0;
    }

    if (fw_complete) {
	iap_finish();
    }

    return IAP_SUCCESS;
}

/*
  * @brief  iap jump
  * @param  none
  * @retval none
  */
static void iap_jump()
{
    if ( iap_info.state == IAP_STS_FW_UPDATE)
	return;

    iap_info.state = IAP_STS_JMP_WAIT;
    iap_respond(iap_info.iap_tx, IAP_CMD_START_APP, 0);
}

/**
  * @brief  usb device iap function
  * @param  udev: to the structure of usbd_core_type
  * @param  pdata: data buffer point
  * @param  len: data length
  * @retval iap_result_type
  */
iap_result_type usbd_hid_iap_process(void *udev, uint8_t *pdata, uint16_t len)
{
  iap_result_type status = IAP_SUCCESS;
  uint16_t iap_cmd;
  uint8_t iap_sign;

  if(len != IAP_PACKET_LENGTH + 1)
  {
      return IAP_FAILED;
  }
  iap_info.respond_flag = 0;
  iap_sign = pdata[0];

  if(iap_sign != SIGN_CMD && iap_sign != SIGN_FW_DATA)
  {
      return IAP_FAILED;
  }

  if (iap_sign == SIGN_FW_DATA)
  {
      status = iap_data_write(pdata + 1, len - 1);
      return status;
  }

  iap_cmd = pdata[1];

  switch(iap_cmd)
  {
  case IAP_CMD_INFO:
      iap_inform();
      break;
  case IAP_CMD_FW_START:
      status = iap_start(pdata);
      break;
  case IAP_CMD_START_APP:
      iap_jump();
      break;
  default:
      status = IAP_FAILED;
      break;
  }

  if(iap_info.respond_flag)
  {
    usb_iap_class_send_report(udev, iap_info.iap_tx, IAP_PACKET_LENGTH + 1);
  }

  return status;
}

/**
  * @brief  usb device in transfer complete function
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_hid_iap_in_complete(void *udev)
{
  if(iap_info.state == IAP_STS_JMP_WAIT)
  {
    iap_info.state = IAP_STS_JMP;
  }
}

/**
  * @brief  iap loop
  * @param  none
  * @retval none
  */
void iap_loop(void)
{
  if(iap_info.state == IAP_STS_JMP)
  {
  	delay_ms(100);
    jump_to_app(iap_info.app_address);
  }
}

/**
  * @}
  */

/**
  * @}
  */
