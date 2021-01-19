/**
 ******************************************************************************
 * @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
 * @author  MCD Application Team
 * @brief   Source file for USBD CDC interface
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------ */
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "circular_buf.h"

static uint8_t usb_cir_buf[2048];

static CIR_BUFFER cir_buf;

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */

/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
USBD_CDC_LineCodingTypeDef LineCoding = { 115200, /* baud rate */
0x00, /* stop bits-1 */
0x00, /* parity - none */
0x08 /* nb. of bits 8 */
};

extern USBD_HandleTypeDef USBD_Device;

static uint8_t UserRxBuffer[512];

/* Private function prototypes ----------------------------------------------- */
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t *pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_fops = {
		CDC_Itf_Init,
		CDC_Itf_DeInit,
		CDC_Itf_Control,
		CDC_Itf_Receive
};

static int8_t CDC_Itf_Transmit(uint8_t *Buf, uint32_t Len);

/* Private functions --------------------------------------------------------- */

/**
 * @brief  Initializes the CDC media low layer
 * @param  None
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Itf_Init(void) {
	/* ##-5- Set Application Buffers ############################################ */
	USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);
    CirBufInit(&cir_buf, sizeof(usb_cir_buf), usb_cir_buf);

    return (USBD_OK);
}

/**
 * @brief  CDC_Itf_DeInit
 *         DeInitializes the CDC media low layer
 * @param  None
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Itf_DeInit(void) {
	return (USBD_OK);
}

/**
 * @brief  CDC_Itf_Control
 *         Manage the CDC class requests
 * @param  Cmd: Command code
 * @param  Buf: Buffer containing command data (request parameters)
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
	uint32_t speed = 115200;
	uint8_t databits = 8;
	switch (cmd) {
	case CDC_SEND_ENCAPSULATED_COMMAND:
		/* Add your code here */
		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:
		/* Add your code here */
		break;

	case CDC_SET_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_GET_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_CLEAR_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_SET_LINE_CODING:
		break;

	case CDC_GET_LINE_CODING:
		length = 7;
		memcpy(pbuf,&speed,4);
		pbuf[4] = 0;
		pbuf[5] = 0;
		pbuf[6] = 8;
		break;

	case CDC_SET_CONTROL_LINE_STATE:
		/* Add your code here */
		break;

	case CDC_SEND_BREAK:
		/* Add your code here */
		break;

	default:
		break;
	}

	return (USBD_OK);
}

/**
 * @brief  CDC_Itf_DataRx
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 * @param  Buf: Buffer of data to be transmitted
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Itf_Receive(uint8_t *Buf, uint32_t *Len) {
	SCB_CleanDCache_by_Addr((uint32_t*) Buf, *Len);

	if (getCirBufAvailableSize(&cir_buf) > *Len) {
		pushToCirBuf(&cir_buf, Buf, *Len);
	}
	USBD_CDC_ReceivePacket(&USBD_Device);	//prepare next data receive buffer
	return (USBD_OK);
}

int8_t usb_send(uint8_t *Buf, uint32_t Len) {
	CDC_Itf_Transmit(Buf, Len);
}

static int8_t CDC_Itf_Transmit(uint8_t *Buf, uint32_t Len) {
	USBD_CDC_SetTxBuffer(&USBD_Device, Buf, Len);
	USBD_CDC_TransmitPacket(&USBD_Device);
	return (0);
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
