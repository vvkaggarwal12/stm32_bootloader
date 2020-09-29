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


uint32_t InitializeExternalFlash(void);
uint32_t ExternalFlashProgram(uint32_t address, uint8_t *buffer, uint32_t size);

#endif /* INC_QSPI_H_ */
