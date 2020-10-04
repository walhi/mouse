/**
  ******************************************************************************
  * @file    usbd_customhid.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_customhid.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_COMPOSITE_H
#define __USB_COMPOSITE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_COMPOSITE
  * @brief This file is the Header file for USBD_customhid.c
  * @{
  */


/** @defgroup USBD_COMPOSITE_Exported_Defines
  * @{
  */

#define USB_COMPOSITE_CONFIG_DESC_SIZ       (9+9+9+7+9+9+7+7)
#define USB_COMPOSITE_DESC_SIZ              9U


#ifndef HID_HS_BINTERVAL
#define HID_HS_BINTERVAL            0x07U
#endif /* HID_HS_BINTERVAL */

#ifndef HID_FS_BINTERVAL
#define HID_FS_BINTERVAL            0x0AU
#endif /* HID_FS_BINTERVAL */



#ifndef COMPOSITE_HS_BINTERVAL
#define COMPOSITE_HS_BINTERVAL            0x05U
#endif /* COMPOSITE_HS_BINTERVAL */

#ifndef COMPOSITE_FS_BINTERVAL
#define COMPOSITE_FS_BINTERVAL            0x05U
#endif /* COMPOSITE_FS_BINTERVAL */

#ifndef USBD_CUSTOMHID_OUTREPORT_BUF_SIZE
#define USBD_CUSTOMHID_OUTREPORT_BUF_SIZE  0x02U
#endif /* USBD_CUSTOMHID_OUTREPORT_BUF_SIZE */
#ifndef USBD_COMPOSITE_REPORT_DESC_SIZE
#define USBD_COMPOSITE_REPORT_DESC_SIZE   163U
#endif /* USBD_COMPOSITE_REPORT_DESC_SIZE */

#define COMPOSITE_DESCRIPTOR_TYPE           0x21U
#define COMPOSITE_REPORT_DESC               0x22U

#define COMPOSITE_REQ_SET_PROTOCOL          0x0BU
#define COMPOSITE_REQ_GET_PROTOCOL          0x03U

#define COMPOSITE_REQ_SET_IDLE              0x0AU
#define COMPOSITE_REQ_GET_IDLE              0x02U

#define COMPOSITE_REQ_SET_REPORT            0x09U
#define COMPOSITE_REQ_GET_REPORT            0x01U
/**
  * @}
  */


#define MOUSE_INTERFACE_IDX 0x0                            	// Index of mouse interface
#define COMPOSITE_INTERFACE_IDX 0x1                            	// Index of custom HID interface

// endpoints numbers
// endpoints numbers
#define MOUSE_EP_IDX                      0x01
#define COMPOSITE_CMD_EP_IDX                  0x02
#define COMPOSITE_EP_IDX                      0x03

#define IN_EP_DIR						0x80 // Adds a direction bit

#define MOUSE_OUT_EP                      MSC_EP_IDX                  /* EP1 for BULK OUT */
#define MOUSE_IN_EP                       MSC_EP_IDX | IN_EP_DIR      /* EP1 for BULK IN */
#define COMPOSITE_CMD_EP                      CDC_CMD_EP_IDX| IN_EP_DIR   /* EP2 for CDC commands */
#define COMPOSITE_OUT_EP                      CDC_EP_IDX                  /* EP3 for data OUT */
#define COMPOSITE_IN_EP                       CDC_EP_IDX | IN_EP_DIR      /* EP3 for data IN */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef enum
{
  COMPOSITE_IDLE = 0U,
  COMPOSITE_BUSY,
}
COMPOSITE_StateTypeDef;

typedef struct _USBD_COMPOSITE_Itf
{
  uint8_t                  *pReport;
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* OutEvent)(uint8_t event_idx, uint8_t state);

} USBD_COMPOSITE_ItfTypeDef;

typedef struct
{
  uint8_t              Report_buf[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];
  uint32_t             Protocol;
  uint32_t             IdleState;
  uint32_t             AltSetting;
  uint32_t             IsReportAvailable;
  COMPOSITE_StateTypeDef     state;
}
USBD_COMPOSITE_HandleTypeDef;
/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_COMPOSITE;
#define USBD_COMPOSITE_CLASS    &USBD_COMPOSITE
/**
  * @}
  */


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_CUSTOMHID_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
