/*
 * qspi.c
 *
 *  Created on: Sep 29, 2020
 *      Author: ngx-vivek
 */

/**
 ******************************************************************************
 * @file    qspi.c
 * @author  MCD Application Team
 * @brief   This file includes a driver for QSPI flashes support mounted on
 *          STM32H750B_DISCO evaluation boards.
 @verbatim
 PartNumber supported by the file:
 -----------------------
 - MT25QL512    :  QSPI Flash memory mounted on STM32H750B Discovery board.
 @endverbatim
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "memory.h"
#include "qspi.h"

/**
 * @brief  MT25TL01G Configuration
 */
#define MT25TL01G_FLASH_SIZE                  0x1000000 /* 2 * 512 MBits => 2 * 64MBytes => 128MBytes*/
#define MT25TL01G_SUBSECTOR_SIZE              0x1000    /* 2 * 16384 subsectors of 4kBytes */
#define MT25TL01G_PAGE_SIZE                   0x100     /* 2 * 262144 pages of 256 bytes */

#define MT25TL01G_DUMMY_CYCLES_READ_QUAD      8
#define MT25TL01G_DUMMY_CYCLES_READ           8
#define MT25TL01G_DUMMY_CYCLES_READ_DTR       6
#define MT25TL01G_DUMMY_CYCLES_READ_QUAD_DTR  6

#define MT25TL01G_DIE_ERASE_MAX_TIME          460000
#define MT25TL01G_SECTOR_ERASE_MAX_TIME       1000
#define MT25TL01G_SUBSECTOR_ERASE_MAX_TIME    400

/**
 * @brief  MT25TL01G Commands
 */
/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

/* Identification Operations */
#define READ_ID_CMD                          0x9E
#define READ_ID_CMD2                         0x9F
#define MULTIPLE_IO_READ_ID_CMD              0xAF
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

/* Read Operations */
#define READ_CMD                             0x03
#define READ_4_BYTE_ADDR_CMD                 0x13

#define FAST_READ_CMD                        0x0B
#define FAST_READ_DTR_CMD                    0x0D
#define FAST_READ_4_BYTE_ADDR_CMD            0x0C

#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_OUT_FAST_READ_DTR_CMD           0x3D
#define DUAL_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x3C

#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define DUAL_INOUT_FAST_READ_DTR_CMD         0xBD
#define DUAL_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xBC

#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_OUT_FAST_READ_DTR_CMD           0x6D
#define QUAD_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x6C

#define QUAD_INOUT_FAST_READ_CMD             0xEB
#define QUAD_INOUT_FAST_READ_DTR_CMD         0xED
#define QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xEC

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define WRITE_STATUS_REG_CMD                 0x01

#define READ_LOCK_REG_CMD                    0xE8
#define WRITE_LOCK_REG_CMD                   0xE5

#define READ_FLAG_STATUS_REG_CMD             0x70
#define CLEAR_FLAG_STATUS_REG_CMD            0x50

#define READ_NONVOL_CFG_REG_CMD              0xB5
#define WRITE_NONVOL_CFG_REG_CMD             0xB1

#define READ_VOL_CFG_REG_CMD                 0x85
#define WRITE_VOL_CFG_REG_CMD                0x81

#define READ_ENHANCED_VOL_CFG_REG_CMD        0x65
#define WRITE_ENHANCED_VOL_CFG_REG_CMD       0x61

#define READ_EXT_ADDR_REG_CMD                0xC8
#define WRITE_EXT_ADDR_REG_CMD               0xC5

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define PAGE_PROG_4_BYTE_ADDR_CMD            0x12

#define DUAL_IN_FAST_PROG_CMD                0xA2
#define EXT_DUAL_IN_FAST_PROG_CMD            0xD2

#define QUAD_IN_FAST_PROG_CMD                0x32
#define EXT_QUAD_IN_FAST_PROG_CMD            0x38
#define QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD    0x34

/* Erase Operations */
#define SUBSECTOR_ERASE_CMD                  0x20
#define SUBSECTOR_ERASE_4_BYTE_ADDR_CMD      0x21

#define SECTOR_ERASE_CMD                     0xD8
#define SECTOR_ERASE_4_BYTE_ADDR_CMD         0xDC

#define DIE_ERASE_CMD                        0xC4

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* One-Time Programmable Operations */
#define READ_OTP_ARRAY_CMD                   0x4B
#define PROG_OTP_ARRAY_CMD                   0x42

/* 4-byte Address Mode Operations */
#define ENTER_4_BYTE_ADDR_MODE_CMD           0xB7
#define EXIT_4_BYTE_ADDR_MODE_CMD            0xE9

/* Quad Operations */
#define ENTER_QUAD_CMD                       0x35
#define EXIT_QUAD_CMD                        0xF5

/**
 * @brief  MT25TL01G Registers
 */
/* Status Register */
#define MT25TL01G_SR_WIP                      ((uint8_t)0x01)    /*!< Write in progress */
#define MT25TL01G_SR_WREN                     ((uint8_t)0x02)    /*!< Write enable latch */
#define MT25TL01G_SR_BLOCKPR                  ((uint8_t)0x5C)    /*!< Block protected against program and erase operations */
#define MT25TL01G_SR_PRBOTTOM                 ((uint8_t)0x20)    /*!< Protected memory area defined by BLOCKPR starts from top or bottom */
#define MT25TL01G_SR_SRWREN                   ((uint8_t)0x80)    /*!< Status register write enable/disable */

/* Non volatile Configuration Register */
#define MT25TL01G_NVCR_NBADDR                 ((uint16_t)0x0001) /*!< 3-bytes or 4-bytes addressing */
#define MT25TL01G_NVCR_SEGMENT                ((uint16_t)0x0002) /*!< Upper or lower 128Mb segment selected by default */
#define MT25TL01G_NVCR_DUAL                   ((uint16_t)0x0004) /*!< Dual I/O protocol */
#define MT25TL01G_NVCR_QUAB                   ((uint16_t)0x0008) /*!< Quad I/O protocol */
#define MT25TL01G_NVCR_RH                     ((uint16_t)0x0010) /*!< Reset/hold */
#define MT25TL01G_NVCR_DTRP                   ((uint16_t)0x0020) /*!< Double transfer rate protocol */
#define MT25TL01G_NVCR_ODS                    ((uint16_t)0x01C0) /*!< Output driver strength */
#define MT25TL01G_NVCR_XIP                    ((uint16_t)0x0E00) /*!< XIP mode at power-on reset */
#define MT25TL01G_NVCR_NB_DUMMY               ((uint16_t)0xF000) /*!< Number of dummy clock cycles */

/* Volatile Configuration Register */
#define MT25TL01G_VCR_WRAP                    ((uint8_t)0x03)    /*!< Wrap */
#define MT25TL01G_VCR_XIP                     ((uint8_t)0x08)    /*!< XIP */
#define MT25TL01G_VCR_NB_DUMMY                ((uint8_t)0xF0)    /*!< Number of dummy clock cycles */

/* Extended Address Register */
#define MT25TL01G_EAR_HIGHEST_SE              ((uint8_t)0x03)    /*!< Select the Highest 128Mb segment */
#define MT25TL01G_EAR_THIRD_SEG               ((uint8_t)0x02)    /*!< Select the Third 128Mb segment */
#define MT25TL01G_EAR_SECOND_SEG              ((uint8_t)0x01)    /*!< Select the Second 128Mb segment */
#define MT25TL01G_EAR_LOWEST_SEG              ((uint8_t)0x00)    /*!< Select the Lowest 128Mb segment (default) */

/* Enhanced Volatile Configuration Register */
#define MT25TL01G_EVCR_ODS                    ((uint8_t)0x07)    /*!< Output driver strength */
#define MT25TL01G_EVCR_RH                     ((uint8_t)0x10)    /*!< Reset/hold */
#define MT25TL01G_EVCR_DTRP                   ((uint8_t)0x20)    /*!< Double transfer rate protocol */
#define MT25TL01G_EVCR_DUAL                   ((uint8_t)0x40)    /*!< Dual I/O protocol */
#define MT25TL01G_EVCR_QUAD                   ((uint8_t)0x80)    /*!< Quad I/O protocol */

/* Flag Status Register */
#define MT25TL01G_FSR_NBADDR                  ((uint8_t)0x01)    /*!< 3-bytes or 4-bytes addressing */
#define MT25TL01G_FSR_PRERR                   ((uint8_t)0x02)    /*!< Protection error */
#define MT25TL01G_FSR_PGSUS                   ((uint8_t)0x04)    /*!< Program operation suspended */
#define MT25TL01G_FSR_PGERR                   ((uint8_t)0x10)    /*!< Program error */
#define MT25TL01G_FSR_ERERR                   ((uint8_t)0x20)    /*!< Erase error */
#define MT25TL01G_FSR_ERSUS                   ((uint8_t)0x40)    /*!< Erase operation suspended */
#define MT25TL01G_FSR_READY                   ((uint8_t)0x80)    /*!< Ready or command in progress */

#define QSPI_CLK_ENABLE()              __HAL_RCC_QSPI_CLK_ENABLE()
#define QSPI_CLK_DISABLE()             __HAL_RCC_QSPI_CLK_DISABLE()
#define QSPI_CLK_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define QSPI_BK1_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define QSPI_BK1_D0_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define QSPI_BK1_D1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define QSPI_BK1_D2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define QSPI_BK1_D3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define QSPI_BK2_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define QSPI_BK2_D0_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOH_CLK_ENABLE()
#define QSPI_BK2_D1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOH_CLK_ENABLE()
#define QSPI_BK2_D2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define QSPI_BK2_D3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define QSPI_MDMA_CLK_ENABLE()         __HAL_RCC_MDMA_CLK_ENABLE()
#define QSPI_FORCE_RESET()         __HAL_RCC_QSPI_FORCE_RESET()
#define QSPI_RELEASE_RESET()       __HAL_RCC_QSPI_RELEASE_RESET()

/* Definition for QSPI Pins */
#define QSPI_CLK_PIN               GPIO_PIN_10
#define QSPI_CLK_GPIO_PORT         GPIOF
/* Bank 1 */
#define QSPI_BK1_CS_PIN            GPIO_PIN_6
#define QSPI_BK1_CS_GPIO_PORT      GPIOG
#define QSPI_BK1_D0_PIN            GPIO_PIN_11
#define QSPI_BK1_D0_GPIO_PORT      GPIOD
#define QSPI_BK1_D1_PIN            GPIO_PIN_9
#define QSPI_BK1_D1_GPIO_PORT      GPIOF
#define QSPI_BK1_D2_PIN            GPIO_PIN_7
#define QSPI_BK1_D2_GPIO_PORT      GPIOF
#define QSPI_BK1_D3_PIN            GPIO_PIN_6
#define QSPI_BK1_D3_GPIO_PORT      GPIOF

/* Bank 2 */
#define QSPI_BK2_CS_PIN            GPIO_PIN_6
#define QSPI_BK2_CS_GPIO_PORT      GPIOG
#define QSPI_BK2_D0_PIN            GPIO_PIN_2
#define QSPI_BK2_D0_GPIO_PORT      GPIOH
#define QSPI_BK2_D1_PIN            GPIO_PIN_3
#define QSPI_BK2_D1_GPIO_PORT      GPIOH
#define QSPI_BK2_D2_PIN            GPIO_PIN_9
#define QSPI_BK2_D2_GPIO_PORT      GPIOG
#define QSPI_BK2_D3_PIN            GPIO_PIN_14
#define QSPI_BK2_D3_GPIO_PORT      GPIOG
#define QSPI_MODE          			((uint32_t)0x00)    /* Init in quad-spi mode for XiP mode */

static QSPI_HandleTypeDef QSPIHandle;
static __IO uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch;

/** @defgroup QSPI_Private_Functions QSPI Private Functions
 * @{
 */
static uint32_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi);
static uint32_t QSPI_EnterFourBytesAddress(QSPI_HandleTypeDef *hqspi);
static uint32_t QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi);
static uint32_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
static uint32_t QSPI_EnableMemoryMappedMode(QSPI_HandleTypeDef *hqspi);
static uint32_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi,
		uint32_t Timeout);
static uint8_t QSPI_EnterQPI(QSPI_HandleTypeDef *hqspi);
static uint32_t QSPI_Startup(uint32_t Mode);
static void QSPI_MspInit(void);

/** @defgroup QSPI_Exported_Functions QSPI Exported Functions
 * @{
 */
uint32_t InitializeExternalFlash(void) {
	if (QSPI_Startup(QSPI_MODE) != MEMORY_OK) {
		return MEMORY_ERROR;
	}
	StatusMatch = 0x00;
	if (QSPI_AutoPollingMemReady(&QSPIHandle, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != MEMORY_OK) {
		return MEMORY_ERROR;
	}
	while(StatusMatch == 0);
	return MEMORY_OK;
}

uint32_t ExternalFlashMemoryMapped(void) {
	/* Configure automatic polling mode to wait the memory is ready */
	if (QSPI_AutoPollingMemReady(&QSPIHandle, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != MEMORY_OK) {
		return MEMORY_ERROR;
	}

	/* Configuration of the dummy cycles on QSPI memory side */
	if (QSPI_DummyCyclesCfg(&QSPIHandle) != MEMORY_OK) {
		return MEMORY_ERROR;
	}

	if (QSPI_EnableMemoryMappedMode(&QSPIHandle) != MEMORY_OK) {
		return MEMORY_ERROR;
	}
	return MEMORY_OK;
}

uint32_t ExternalFlashProgram(uint32_t address, uint8_t *buffer, uint32_t size) {
	QSPI_CommandTypeDef sCommand;

	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize       = QSPI_ADDRESS_32_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    sCommand.Address     		= address;
    sCommand.DummyCycles 		= 0;

	/* Enable write operations ----------------------------------------- */
	QSPI_WriteEnable(&QSPIHandle);

	TxCplt = 0;
	/* Writing Sequence ------------------------------------------------ */
	sCommand.Instruction = QUAD_IN_FAST_PROG_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
	sCommand.DataMode = QSPI_DATA_4_LINES;
	sCommand.NbData = size;

	if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	if (HAL_QSPI_Transmit_IT(&QSPIHandle, buffer) != HAL_OK) {
		return MEMORY_ERROR;
	}
	while(TxCplt == 0);
	StatusMatch = 0x00;
	if (QSPI_AutoPollingMemReady(&QSPIHandle,
			HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != MEMORY_OK) {
		return MEMORY_ERROR;
	}
	while(StatusMatch == 0);
	return MEMORY_OK;
}

uint32_t ExternalFlashErase(uint32_t address) {
	QSPI_CommandTypeDef sCommand;

	CmdCplt = 0;
	/* Enable write operations ------------------------------------------- */
	QSPI_WriteEnable(&QSPIHandle);

	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize       = QSPI_ADDRESS_32_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	/* Erasing Sequence -------------------------------------------------- */
	sCommand.Instruction = SECTOR_ERASE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
	sCommand.Address = address;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;

	if (HAL_QSPI_Command_IT(&QSPIHandle, &sCommand) != HAL_OK) {
		return MEMORY_ERROR;
	}
	while(CmdCplt == 0);
	StatusMatch = 0x00;
	if (QSPI_AutoPollingMemReady(&QSPIHandle,
			HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != MEMORY_OK) {
		return MEMORY_ERROR;
	}
	while(StatusMatch == 0);
	return MEMORY_OK;
}

uint32_t ExternalFlashRead(uint32_t address, uint8_t *buffer, uint32_t size) {
	QSPI_CommandTypeDef sCommand;
	QSPI_MemoryMappedTypeDef sMemMappedCfg;
	uint16_t index;
	uint8_t *tempAddress = NULL;

	/* Enable write operations ----------------------------------------- */
	/* Configure Volatile Configuration register (with new dummy cycles) */
	QSPI_DummyCyclesCfg(&QSPIHandle);

	/* Reading Sequence ------------------------------------------------ */
	sCommand.Instruction = QUAD_OUT_FAST_READ_CMD;
	sCommand.DummyCycles = 10;

	sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

	if (HAL_QSPI_MemoryMapped(&QSPIHandle, &sCommand, &sMemMappedCfg)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	tempAddress = (uint8_t*) (APPLICATION_ADDRESS + address);
	for (index = 0; index < size; index++) {
		buffer[index] = tempAddress[index];
	}

	return MEMORY_OK;
}

/**
 * @brief  Initializes and configure the QSPI interface.
 * @retval QSPI memory status
 */
static uint32_t QSPI_Startup(uint32_t Mode) {
	QSPIHandle.Instance = QUADSPI;

	/* QSPI initialization */
	/* ClockPrescaler set to 1, so QSPI clock = 200MHz / (1+3) = 50MHz */
	QSPIHandle.Init.ClockPrescaler = 1;
	QSPIHandle.Init.FifoThreshold = 1;
	QSPIHandle.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
	QSPIHandle.Init.FlashSize = 26;
	QSPIHandle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_3_CYCLE;
	QSPIHandle.Init.ClockMode = QSPI_CLOCK_MODE_0;
	QSPIHandle.Init.FlashID = QSPI_FLASH_ID_2;
	QSPIHandle.Init.DualFlash = QSPI_DUALFLASH_ENABLE;

    HAL_QSPI_DeInit(&QSPIHandle);
    if (HAL_QSPI_Init(&QSPIHandle) != HAL_OK)
    {
    	return MEMORY_ERROR;
    }

	/* Set the QSPI memory in 4-bytes address mode */
	if (QSPI_EnterFourBytesAddress(&QSPIHandle) != MEMORY_OK) {
		return MEMORY_ERROR;
	}

	return MEMORY_OK;
}

#if (BINARY_AREA == USE_SPI_NOR)
/**
 * @brief  De-Initializes and the QSPI interface.
 * @retval None
 */
static uint32_t QSPI_Shutdown(void) {
	/* Call the DeInit function to reset the driver */
	if (HAL_QSPI_DeInit(&QSPIHandle) != HAL_OK) {
		return MEMORY_ERROR;
	}

	/* System level De-initialization */
	QSPI_MspDeInit();

	return MEMORY_OK;
}

/**
 * @brief  Copy an amount of data from the QSPI memory to destination memory.
 * @param  WriteAddr: Pointer to data to be read
 * @param  ReadAddr: Read start address
 * @param  Size: Size of data to read
 * @retval QSPI memory status
 */
static uint32_t QSPI_Copy(uint32_t WriteAddr, uint32_t ReadAddr, uint32_t Size) {
	QSPI_CommandTypeDef s_command;

	/* Initialize the read command */
	s_command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
	s_command.Instruction = QUAD_INOUT_FAST_READ_DTR_CMD; /* DTR QUAD INPUT/OUTPUT FAST READ and 4-BYTE DTR FAST READ commands */
	s_command.AddressMode = QSPI_ADDRESS_4_LINES;
	s_command.AddressSize = QSPI_ADDRESS_32_BITS;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_4_LINES;
	s_command.DummyCycles = MT25TL01G_DUMMY_CYCLES_READ_QUAD_DTR - 1;
	s_command.DdrMode = QSPI_DDR_MODE_ENABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_HALF_CLK_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	while (Size) {
		s_command.NbData = (Size < 256) ? Size : 256;
		s_command.Address = ReadAddr;

		/* Configure the command */
		if (HAL_QSPI_Command(&QSPIHandle, &s_command,
				HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
			return MEMORY_ERROR;
		}

		/* Reception of the data */
		if (HAL_QSPI_Receive(&QSPIHandle, (uint8_t*) WriteAddr,
				HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
			return MEMORY_ERROR;
		}

		if (Size > 256) {
			Size -= 256;
			WriteAddr += 256;
			ReadAddr += 256;
		} else {
			Size = 0;
		}
	}

	return MEMORY_OK;
}
#endif /* (BINARY_AREA == USE_SPI_NOR) */

/**
 * @}
 */

/** @addtogroup QSPI_Private_Functions
 * @{
 */

#if (CODE_AREA == USE_QSPI)
/**
 * @brief  Configure the QSPI in memory-mapped mode
 * @retval QSPI memory status
 */
static uint32_t QSPI_EnableMemoryMappedMode(QSPI_HandleTypeDef *hqspi) {
	QSPI_CommandTypeDef s_command;
	QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

	/* Configure the command for the read instruction */
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = QUAD_OUT_FAST_READ_CMD; /* DTR QUAD INPUT/OUTPUT FAST READ and 4-BYTE DTR FAST READ commands */
	s_command.AddressMode = QSPI_ADDRESS_1_LINE;
	s_command.AddressSize = QSPI_ADDRESS_32_BITS;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_4_LINES;
	s_command.DummyCycles = 10;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	/* Configure the memory mapped mode */
	s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
	s_mem_mapped_cfg.TimeOutPeriod = 0;

	if (HAL_QSPI_MemoryMapped(&QSPIHandle, &s_command, &s_mem_mapped_cfg)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	return MEMORY_OK;
}
#endif /* (BINARY_AREA == USE_SPI_NOR) */

/**
 * @brief  This function reset the QSPI memory.
 * @param  hqspi: QSPI handle
 * @retval None
 */
static uint32_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi) {
	QSPI_CommandTypeDef s_command;

	/* Initialize the reset enable command */
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = RESET_ENABLE_CMD;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_NONE;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	/* Send the command */
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	/* Send the reset memory command */
	s_command.Instruction = RESET_MEMORY_CMD;
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	s_command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
	s_command.Instruction = RESET_ENABLE_CMD;
	/* Send the command */
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	/* Send the reset memory command */
	s_command.Instruction = RESET_MEMORY_CMD;
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	/* Enter QSPI memory in QPI mode */
	if (QSPI_EnterQPI(hqspi) != MEMORY_OK) {
		return MEMORY_ERROR;
	}

	/* Configure automatic polling mode to wait the memory is ready */
	if (QSPI_AutoPollingMemReady(hqspi,
			HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != MEMORY_OK) {
		return MEMORY_ERROR;
	}

	return MEMORY_OK;
}

/**
 * @brief  This function set the QSPI memory in 4-byte address mode
 * @param  hqspi: QSPI handle
 * @retval None
 */
static uint32_t QSPI_EnterFourBytesAddress(QSPI_HandleTypeDef *hqspi) {
	QSPI_CommandTypeDef s_command;

	/* Initialize the command */
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = ENTER_4_BYTE_ADDR_MODE_CMD;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_NONE;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	/* Enable write operations */
	if (QSPI_WriteEnable(hqspi) != MEMORY_OK) {
		return MEMORY_ERROR;
	}

	/* Send the command */
	if (HAL_QSPI_Command(hqspi, &s_command,
			HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != MEMORY_OK) {
		return MEMORY_ERROR;
	}

	/* Configure automatic polling mode to wait the memory is ready */
	if (QSPI_AutoPollingMemReady(hqspi,
			HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != MEMORY_OK) {
		return MEMORY_ERROR;
	}

	return MEMORY_OK;
}

/**
 * @brief  This function configure the dummy cycles on memory side.
 * @param  hqspi: QSPI handle
 * @retval None
 */
static uint32_t QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi) {
	QSPI_CommandTypeDef s_command;
	uint16_t reg = 0;

	/* Initialize the read volatile configuration register command */
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = READ_VOL_CFG_REG_CMD;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.DummyCycles = 0;
	s_command.NbData = 2;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	/* Configure the command */
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	/* Reception of the data */
	if (HAL_QSPI_Receive(hqspi, (uint8_t*) (&reg),
			HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return MEMORY_ERROR;
	}

	/* Enable write operations */
	if (QSPI_WriteEnable(hqspi) != MEMORY_OK) {
		return MEMORY_ERROR;
	}

	/* Update volatile configuration register (with new dummy cycles) */
	s_command.Instruction = WRITE_VOL_CFG_REG_CMD;
	MODIFY_REG(reg, 0xF0F0,
			((10 << 4) | (10 << 12)));

	/* Configure the write volatile configuration register command */
	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	/* Transmission of the data */
	if (HAL_QSPI_Transmit(hqspi, (uint8_t*) (&reg),
			HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return MEMORY_ERROR;
	}

	return MEMORY_OK;
}

/**
 * @brief  This function send a Write Enable and wait it is effective.
 * @param  hqspi: QSPI handle
 * @retval None
 */
static uint32_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi) {
	QSPI_CommandTypeDef s_command;
	QSPI_AutoPollingTypeDef s_config;

	/* Enable write operations */
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = WRITE_ENABLE_CMD;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_NONE;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	/* Configure automatic polling mode to wait for write enabling */
	s_config.Match = MT25TL01G_SR_WREN | (MT25TL01G_SR_WREN << 8);
	s_config.Mask = MT25TL01G_SR_WREN | (MT25TL01G_SR_WREN << 8);
	s_config.MatchMode = QSPI_MATCH_MODE_AND;
	s_config.StatusBytesSize = 2;
	s_config.Interval = 0x10;
	s_config.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	s_command.Instruction = READ_STATUS_REG_CMD;
	s_command.DataMode = QSPI_DATA_1_LINE;

	if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config,
			HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return MEMORY_ERROR;
	}

	return MEMORY_OK;
}

/**
 * @brief  This function read the SR of the memory and wait the EOP.
 * @param  hqspi: QSPI handle
 * @param  Timeout
 * @retval None
 */
static uint32_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi,
		uint32_t Timeout) {
	QSPI_CommandTypeDef s_command;
	QSPI_AutoPollingTypeDef s_config;

	/* Configure automatic polling mode to wait for memory ready */
	memset(&s_command, 0x00, sizeof(s_command));
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = READ_STATUS_REG_CMD;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	s_config.Match = 0;
	s_config.Mask = 0x0101;
	s_config.MatchMode = QSPI_MATCH_MODE_AND;
	s_config.StatusBytesSize = 2;
	s_config.Interval = 0x10;
	s_config.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling_IT(hqspi, &s_command, &s_config) != HAL_OK) {
		return MEMORY_ERROR;
	}

	return MEMORY_OK;
}

/**
 * @brief  This function enter the QPSI memory in QPI mode
 * @param  hqspi QSPI handle
 * @retval QSPI status
 */
static uint8_t QSPI_EnterQPI(QSPI_HandleTypeDef *hqspi) {
	QSPI_CommandTypeDef s_command;

	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = ENTER_QUAD_CMD;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_NONE;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return MEMORY_ERROR;
	}

	return MEMORY_OK;
}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
  GPIO_InitTypeDef gpio_init_structure;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable the QuadSPI memory interface clock */
  QSPI_CLK_ENABLE();
  /* Reset the QuadSPI memory interface */
  QSPI_FORCE_RESET();
  QSPI_RELEASE_RESET();
  /* Enable GPIO clocks */
  QSPI_CLK_GPIO_CLK_ENABLE();
  QSPI_BK1_D0_GPIO_CLK_ENABLE();
  QSPI_BK1_D1_GPIO_CLK_ENABLE();
  QSPI_BK1_D2_GPIO_CLK_ENABLE();
  QSPI_BK1_D3_GPIO_CLK_ENABLE();

  QSPI_BK2_D0_GPIO_CLK_ENABLE();
  QSPI_BK2_D1_GPIO_CLK_ENABLE();
  QSPI_BK2_D2_GPIO_CLK_ENABLE();
  QSPI_BK2_D3_GPIO_CLK_ENABLE();
  /* Enable DMA clock */
  QSPI_MDMA_CLK_ENABLE();

/*##-2- Configure peripheral GPIO ##########################################*/
  /* QSPI CLK GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_CLK_PIN;
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_CLK_GPIO_PORT, &gpio_init_structure);

  /* QSPI CS GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_BK1_CS_PIN;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_BK1_CS_GPIO_PORT, &gpio_init_structure);

  /* QSPI D0 GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_BK1_D0_PIN;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_BK1_D0_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin       = QSPI_BK2_D0_PIN;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_BK2_D0_GPIO_PORT, &gpio_init_structure);

  /* QSPI D1 GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_BK1_D1_PIN;
  gpio_init_structure.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_BK1_D1_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin       = QSPI_BK2_D1_PIN;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_BK2_D1_GPIO_PORT, &gpio_init_structure);

  /* QSPI D2 GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_BK1_D2_PIN;
  gpio_init_structure.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_BK1_D2_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin       = QSPI_BK2_D2_PIN;
  HAL_GPIO_Init(QSPI_BK2_D2_GPIO_PORT, &gpio_init_structure);

  /* QSPI D3 GPIO pin configuration  */
  gpio_init_structure.Pin       = QSPI_BK1_D3_PIN;
  HAL_GPIO_Init(QSPI_BK1_D3_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin       = QSPI_BK2_D3_PIN;
  HAL_GPIO_Init(QSPI_BK2_D3_GPIO_PORT, &gpio_init_structure);

  /*##-3- Configure the NVIC for QSPI #########################################*/
  /* NVIC configuration for QSPI interrupt */
  HAL_NVIC_SetPriority(QUADSPI_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(QUADSPI_IRQn);

}

/**
  * @brief QSPI MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param hqspi: QSPI handle pointer
  * @retval None
  */
void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi)
{
  /*##-1- Disable the NVIC for QSPI ###########################################*/
  HAL_NVIC_DisableIRQ(QUADSPI_IRQn);

  /*##-1- Disable peripherals ################################################*/
  /* De-Configure QSPI pins */
  HAL_GPIO_DeInit(QSPI_CLK_GPIO_PORT, QSPI_CLK_PIN);
  HAL_GPIO_DeInit(QSPI_BK1_CS_GPIO_PORT, QSPI_BK1_CS_PIN);
  HAL_GPIO_DeInit(QSPI_BK1_D0_GPIO_PORT, QSPI_BK1_D0_PIN);
  HAL_GPIO_DeInit(QSPI_BK1_D1_GPIO_PORT, QSPI_BK1_D1_PIN);
  HAL_GPIO_DeInit(QSPI_BK1_D2_GPIO_PORT, QSPI_BK1_D2_PIN);
  HAL_GPIO_DeInit(QSPI_BK1_D3_GPIO_PORT, QSPI_BK1_D3_PIN);

  HAL_GPIO_DeInit(QSPI_BK2_CS_GPIO_PORT, QSPI_BK2_CS_PIN);
  HAL_GPIO_DeInit(QSPI_BK2_D0_GPIO_PORT, QSPI_BK2_D0_PIN);
  HAL_GPIO_DeInit(QSPI_BK2_D1_GPIO_PORT, QSPI_BK2_D1_PIN);
  HAL_GPIO_DeInit(QSPI_BK2_D2_GPIO_PORT, QSPI_BK2_D2_PIN);
  HAL_GPIO_DeInit(QSPI_BK2_D3_GPIO_PORT, QSPI_BK2_D3_PIN);

  /*##-3- Reset peripherals ##################################################*/
  /* Reset the QuadSPI memory interface */
  QSPI_FORCE_RESET();
  QSPI_RELEASE_RESET();

  /* Disable the QuadSPI memory interface clock */
  QSPI_CLK_DISABLE();
}

/**
 * @brief  Command completed callbacks.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi) {
	CmdCplt++;
}

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi) {
	RxCplt++;
}

/**
 * @brief  Tx Transfer completed callbacks.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi) {
	TxCplt++;
}

/**
 * @brief  Status Match callbacks
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi) {
	StatusMatch++;
}

/**
 * @brief  This function handles QUADSPI interrupt request.
 * @param  None
 * @retval None
 */
void QUADSPI_IRQHandler(void)
{
	HAL_QSPI_IRQHandler(&QSPIHandle);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

