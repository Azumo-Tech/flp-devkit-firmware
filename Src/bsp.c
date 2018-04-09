#include "bsp.h"
#include "main.h"
#include "stm32l1xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

static const uint32_t DFU_BOOTKEY = 0x157F32D4;
static uint32_t *bootkey_adr = (void *)(0x20013ffc); /* This is where the bootloader looks for the magic number */

void BSP_reset_to_bootloader() {
	*bootkey_adr = DFU_BOOTKEY;
	BSP_reset();
}

void BSP_reset() {
	HAL_GPIO_WritePin(USB_DISCONNECT_GPIO_Port, USB_DISCONNECT_Pin, 1);
	USBD_Stop(&hUsbDeviceFS);
	HAL_Delay(1000);
	NVIC_SystemReset();
}
