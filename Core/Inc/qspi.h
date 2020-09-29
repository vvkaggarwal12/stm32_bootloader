/*
 * qspi.h
 *
 *  Created on: Sep 29, 2020
 *      Author: ngx-vivek
 */

#ifndef INC_QSPI_H_
#define INC_QSPI_H_

#define APPLICATION_ADDRESS            QSPI_BASE
#define MEMORY_OK          ((uint32_t)0x00)
#define MEMORY_ERROR       ((uint32_t)0x01)


uint32_t InitializeExternalFlash(void);

#endif /* INC_QSPI_H_ */
