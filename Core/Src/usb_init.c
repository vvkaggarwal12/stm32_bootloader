/*
 * usb_init.c
 *
 *  Created on: Sep 29, 2020
 *      Author: ngx-vivek
 */
#include "usbh_core.h"
#include "usbh_msc.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "usbh_diskio.h"
#include "main.h"

#define LCD_DbgLog(x)
#define LCD_ErrLog(x)
#define LCD_UsrLog(x)


static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
extern uint8_t MSC_File_Operations(void);
char USBDISKPath[4]; /* USB Host logical drive path */
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;
static FATFS USBH_fatfs;
USBH_HandleTypeDef hUSB_Host;

void usb_initialize(void) {
	/* Init Host Library */
	USBH_Init(&hUSB_Host, USBH_UserProcess, 0);

	/* Add Supported Class */
	USBH_RegisterClass(&hUSB_Host, USBH_MSC_CLASS);

	/* Start Host Process */
	USBH_Start(&hUSB_Host);

	/* Enable the USB voltage level detector */

	HAL_PWREx_EnableUSBVoltageDetector();
}

void usbhost_process(void) {
	USBH_Process(&hUSB_Host);
}

uint8_t IsUSBHostConnected(void) {
	return hUSB_Host.device.is_connected;
}
/**
 * @brief  User Process
 * @param  phost: Host Handle
 * @param  id: Host Library user message ID
 * @retval None
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id) {
	switch (id) {
	case HOST_USER_SELECT_CONFIGURATION:
		break;

	case HOST_USER_DISCONNECTION:
		Appli_state = APPLICATION_DISCONNECT;
		if (f_mount(NULL, "", 0) != FR_OK) {
			LCD_ErrLog("ERROR : Cannot DeInitialize FatFs! \n");
		}
		if (FATFS_UnLinkDriver(USBDISKPath) != 0) {
			LCD_ErrLog("ERROR : Cannot UnLink FatFS Driver! \n");
		}
		break;

	case HOST_USER_CLASS_ACTIVE:
		Appli_state = APPLICATION_READY;
		break;

	case HOST_USER_CONNECTION:
		if (FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0) {
			if (f_mount(&USBH_fatfs, "", 0) != FR_OK) {
				LCD_ErrLog("ERROR : Cannot Initialize FatFs! \n");
			}
		}
		break;

	default:
		break;
	}
}

/**
 * @brief  Files operations: Read/Write and compare
 * @param  None
 * @retval None
 */
//uint8_t MSC_File_Operations(void) {
//	uint16_t bytesread;
//	uint8_t retvalue = 0;
//
//	LCD_UsrLog("INFO : FatFs Initialized \n");
//
//	if (f_open(&MyFile, "0:USBHost.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
//		LCD_ErrLog("Cannot Open 'USBHost.txt' file \n");
//		retvalue = 1;
//	} else {
//		LCD_UsrLog("INFO : 'USBHost.txt' opened for write  \n");
//		res = f_write(&MyFile, wtext, sizeof(wtext), (void*) &bytesWritten);
//		f_close(&MyFile);
//
//		if ((bytesWritten == 0) || (res != FR_OK)) /*EOF or Error*/
//		{
//			LCD_ErrLog("Cannot Write on the  'USBHost.txt' file \n");
//			retvalue = 1;
//		} else {
//			if (f_open(&MyFile, "0:USBHost.txt", FA_READ) != FR_OK) {
//				LCD_ErrLog("Cannot Open 'USBHost.txt' file for read.\n");
//				retvalue = 1;
//			} else {
//				LCD_UsrLog("INFO : Text written on the 'USBHost.txt' file \n");
//
//				res = f_read(&MyFile, rtext, sizeof(rtext), (void*) &bytesread);
//
//				if ((bytesread == 0) || (res != FR_OK)) /*EOF or Error*/
//				{
//					LCD_ErrLog("Cannot Read from the  'USBHost.txt' file \n");
//					retvalue = 1;
//				} else {
//					LCD_UsrLog("Read Text : \n");
//					LCD_DbgLog((char*) rtext);
//					LCD_DbgLog("\n");
//				}
//				f_close(&MyFile);
//			}
//			/* Compare read data with the expected data */
//			if ((bytesread == bytesWritten)) {
//				LCD_UsrLog("INFO : FatFs data compare SUCCES");
//				LCD_UsrLog("\n");
//			} else {
//				LCD_ErrLog("FatFs data compare ERROR");
//				LCD_ErrLog("\n");
//				retvalue = 1;
//			}
//		}
//	}
//	return (retvalue);
//}

