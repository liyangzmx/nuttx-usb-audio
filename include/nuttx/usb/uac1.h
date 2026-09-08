/****************************************************************************
 * include/nuttx/usb/uac1.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_UAC1_H
#define __INCLUDE_NUTTX_USB_UAC1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UAC1IOC_GETSTATUS _USBCIOC(0x10)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct uac1_status_s
{
  int16_t volume;       /* Signed 8.8 dB value, 0 == 0 dB */
  bool mute;
  bool configured;
  bool streaming;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

FAR void *usbdev_uac1_initialize(void);
void usbdev_uac1_uninitialize(FAR void *handle);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_USB_UAC1_H */
