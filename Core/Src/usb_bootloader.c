/*
 * usb_bootloader.c
 *
 *  Created on: Sep 29, 2020
 *      Author: ngx-vivek
 */
#include "main.h"
#include "ff.h"
#include "qspi.h"
#include "usb_bootloader.h"

static FIL fp;

#define APPLICATION_FILEPATH		"0:App.bin"

uint8_t IsApplicationFileExist(void) {
	if (f_open(&fp, APPLICATION_FILEPATH, FA_READ) != FR_OK) {
		return 0;
	}
	return 1;
}
uint8_t buffer[QSPI_PAGE_SIZE];
bootloader_retval_t UpdateApplicationFromUsb(void) {
	FRESULT res = FR_DENIED;
	uint32_t br = 0x00, address = 0x00;
	uint32_t retval = UPDATE_SUCCESS;

	do {
		res = f_open(&fp, APPLICATION_FILEPATH, FA_READ);
		if (FR_OK != res) {
			retval = FILE_OPERATION_FAILED;
			break;
		}
		f_lseek(&fp, 0x00);
		address = 0x00;
		while (0 == f_eof(&fp)) {
			memset(buffer, 0x00, sizeof(buffer));
			res = f_read(&fp, buffer, sizeof(buffer), &br);
			if (FR_OK != res) {
				retval = FILE_OPERATION_FAILED;
				break;
			}
			if (br != sizeof(buffer)) {

			}
			if (0 == (address % 0x10000)) {
				if (MEMORY_OK != ExternalFlashErase(address)) {
					retval = FLASH_OPERATION_FAILED;
					break;
				}
			}
			if (MEMORY_OK != ExternalFlashProgram(address, buffer, sizeof(buffer))) {
				retval = FLASH_OPERATION_FAILED;
				break;
			}
			address += sizeof(buffer);
		}
		if (UPDATE_SUCCESS != retval) {
			break;
		}
	} while(0);
	f_close(&fp);

	return retval;
}

bootloader_retval_t testFlashOperations(void) {
	uint8_t buffer[QSPI_PAGE_SIZE];
	uint32_t i = 0, address = 0x00;
	uint32_t retval = UPDATE_SUCCESS;

	do {
		address = 0x00;
		memset(buffer, 0x0A, sizeof(buffer));
		for (i = 0; i < 10; i++) {
			if (MEMORY_OK != ExternalFlashErase(address)) {
				retval = FLASH_OPERATION_FAILED;
				break;
			}
			if (MEMORY_OK != ExternalFlashProgram(address, buffer, sizeof(buffer))) {
				retval = FLASH_OPERATION_FAILED;
				break;
			}
			address += sizeof(buffer);
		}
	} while(0);

	return retval;
}

uint8_t IsValidApplicationPresent(void) {
	return 1;
}
