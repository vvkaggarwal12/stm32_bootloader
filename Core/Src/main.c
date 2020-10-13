/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_init.h"
#include "qspi.h"
#include "usb_bootloader.h"

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void CPU_CACHE_Disable(void);
static void jumpToAddress(uint32_t address);
static void startApplication(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
typedef  void (*pFunction)(void);
static pFunction JumpToApplication;
extern MSC_ApplicationTypeDef Appli_state;

int main(void) {
	uint32_t step = 0;

	CPU_CACHE_Enable();
	HAL_Init();
	SystemClock_Config();

	usb_initialize();

	/* Infinite loop */
	while (1) {
		usbhost_process();
		/*check if usb stick enumeration is complete*/
//		if ((500 < step) && !IsUSBHostConnected()) {
//			if (1 == IsValidApplicationPresent()) {
//				InitializeExternalFlash();
//				startApplication();
//			} else {
//				continue;
//				//Invalid Application in external flash
//				//Continue to stay in bootloader
//			}
//		}
		if (Appli_state == APPLICATION_READY) {
			if (1 == IsApplicationFileExist()) {
				if (MEMORY_OK != InitializeExternalFlash()) {
					//external flash initialization issue, check external flash
				}
				if (UPDATE_SUCCESS == UpdateApplicationFromUsb()) {
					startApplication();
				} else {
					//update application failed, try again
				}
			} else {
				//application update file not exist.
				if (1 == IsValidApplicationPresent()) {
					InitializeExternalFlash();
					startApplication();
				} else {
					continue;
					//Invalid Application in external flash
					//Continue to stay in bootloader
				}
			}
		} else {
			HAL_Delay(1);
			step++;
		}
	}
}

static void startApplication(void) {
	ExternalFlashMemoryMapped();
	/* Disable CPU L1 cache before jumping to the QSPI code execution */
	CPU_CACHE_Disable();
	/* Disable Systick interrupt */
	SysTick->CTRL = 0;
	jumpToAddress(APPLICATION_ADDRESS);
}

static void jumpToAddress(uint32_t address) {
	/* Initialize user application's Stack Pointer & Jump to user application */
	JumpToApplication = (pFunction)(*(__IO uint32_t*) (address + 4));
	__set_MSP(*(__IO uint32_t*) address);
	JumpToApplication();
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/*!< Supply configuration update enable */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY); /* PWR set to LDO for the STM32H750B-DISCO board */

	/* The voltage scaling allows optimizing the power consumption when the device is
	 clocked below the maximum system frequency, to update the voltage scaling value
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	/* PLL1 for System Clock */
	RCC_OscInitStruct.PLL.PLLM = 3;
	RCC_OscInitStruct.PLL.PLLN = 200;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;

	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* PLL3 for USB Clock */

	PeriphClkInitStruct.PLL3.PLL3M = 3;
	PeriphClkInitStruct.PLL3.PLL3N = 120;
	PeriphClkInitStruct.PLL3.PLL3P = 2;
	PeriphClkInitStruct.PLL3.PLL3Q = 10;
	PeriphClkInitStruct.PLL3.PLL3R = 18;

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	/* Select PLL as system clock source and configure  bus clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 |
			RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1);

	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

	/*activate CSI clock mondatory for I/O Compensation Cell*/
	__HAL_RCC_CSI_ENABLE();

	/* Enable SYSCFG clock mondatory for I/O Compensation Cell */
	__HAL_RCC_SYSCFG_CLK_ENABLE()
	;

	/* Enables the I/O Compensation Cell */
	HAL_EnableCompensationCell();
}

/**
 * @brief  CPU L1-Cache enable.
 * @param  None
 * @retval None
 */
static void CPU_CACHE_Enable(void) {
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

/**
  * @brief  CPU L1-Cache disable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Disable(void)
{
  /* Disable I-Cache */
  SCB_DisableICache();

  /* Disable D-Cache */
  SCB_DisableDCache();
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
