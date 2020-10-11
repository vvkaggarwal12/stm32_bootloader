/*
 * usb_bootloader.h
 *
 *  Created on: Sep 29, 2020
 *      Author: ngx-vivek
 */

#ifndef INC_USB_BOOTLOADER_H_
#define INC_USB_BOOTLOADER_H_

typedef enum {
	UPDATE_SUCCESS,
	FLASH_OPERATION_FAILED,
	FILE_OPERATION_FAILED,
} bootloader_retval_t;

uint8_t IsApplicationFileExist(void);
uint8_t UpdateApplicationFromUsb(void);
uint8_t IsValidApplicationPresent(void);
bootloader_retval_t testFlashOperations(void);

#endif /* INC_USB_BOOTLOADER_H_ */
