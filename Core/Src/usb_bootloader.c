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
#include "string.h"

typedef struct {
	uint32_t updateDataFlag;
	uint8_t dataConf[1024];
} system_configuration_t;

static FIL fp;
static uint8_t  __attribute__ ((aligned (0x10))) buffer[QSPI_PAGE_SIZE];
static __attribute__ ((section(".conf_section"))) system_configuration_t conf;

#define CONFIGURATION_DATA_SECTOR_NUM		(QSPI_TOTAL_SECTORS - 1)
#define CONFIGURATION_DATA_ADDRESS			(CONFIGURATION_DATA_SECTOR_NUM * QSPI_SECTOR_SIZE)

#define APPLICATION_FILEPATH		"0:App.bin"

uint8_t IsApplicationFileExist(void) {
	if (f_open(&fp, APPLICATION_FILEPATH, FA_READ) != FR_OK) {
		return 0;
	}
	return 1;
}

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

uint8_t isConfDataUpdateRequired(void) {
	uint8_t retval = 0;

	if (1 == conf.updateDataFlag) {
		retval = 1;
	}

	return retval;
}

void getConfDataFromFlashToRAM(void) {
	uint8_t *address = (uint8_t *)(QSPI_START_ADDRESS + CONFIGURATION_DATA_ADDRESS);
	memcpy(&conf.dataConf[2], address, sizeof(conf.dataConf) - 2);
	SCB_CleanDCache_by_Addr((uint32_t *) &conf, sizeof(conf));
}

bootloader_retval_t updateConfDataToExternalFlash(void) {
	uint32_t retval = UPDATE_SUCCESS, i = 0;

	do {
		if (MEMORY_OK != ExternalFlashErase(CONFIGURATION_DATA_ADDRESS)) {
			retval = FLASH_OPERATION_FAILED;
			break;
		}
		if (0 != sizeof(conf.dataConf) % sizeof(buffer)) {
			retval = FLASH_OPERATION_FAILED;
			break;
		}
		for (i = 0; i < (sizeof(conf.dataConf) / sizeof(buffer)); i++) {
			memset(buffer, 0x00, sizeof(buffer));
			memcpy(buffer, &conf.dataConf[i * sizeof(buffer)], sizeof(buffer));
			if (MEMORY_OK != ExternalFlashProgram((CONFIGURATION_DATA_ADDRESS + (i * sizeof(buffer))), buffer, sizeof(buffer))) {
				retval = FLASH_OPERATION_FAILED;
				break;
			}
		}
		conf.updateDataFlag = 0x00;
		SCB_CleanDCache_by_Addr((uint32_t *) &conf, sizeof(conf));
	} while(0);

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
