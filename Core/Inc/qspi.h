/*
 * qspi.h
 *
 *  Created on: Sep 29, 2020
 *      Author: ngx-vivek
 */

#ifndef INC_QSPI_H_
#define INC_QSPI_H_

#define APPLICATION_ADDRESS           	QSPI_BASE
#define MEMORY_OK          				((uint32_t)0x00)
#define MEMORY_ERROR       				((uint32_t)0x01)
#define QSPI_PAGE_SIZE					0x100
#define QSPI_SECTOR_SIZE           		0x10000   /* 2 * 1024 sectors of 64KBytes */

uint32_t InitializeExternalFlash(void);
uint32_t ExternalFlashProgram(uint32_t address, uint8_t *buffer, uint32_t size);
uint32_t ExternalFlashErase(uint32_t address);
uint32_t ExternalFlashRead(uint32_t address, uint8_t *buffer, uint32_t size);
uint32_t ExternalFlashMemoryMapped(void);

#endif /* INC_QSPI_H_ */
