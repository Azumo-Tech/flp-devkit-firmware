#ifndef BSP_H_
#define BSP_H_

#define FIRMWARE_VERSION 20190826

void BSP_init();
int BSP_sleep();
void BSP_reset_to_bootloader();
void BSP_reset();

#endif /* BSP_H_ */
