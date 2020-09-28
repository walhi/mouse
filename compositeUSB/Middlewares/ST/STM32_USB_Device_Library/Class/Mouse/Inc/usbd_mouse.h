/**
  ******************************************************************************
  * @file    usbd_MOUSE.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_MOUSE_core.c file.
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
#ifndef __USB_MOUSE_H
#define __USB_MOUSE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_MOUSE
  * @brief This file is the Header file for usbd_MOUSE.c
  * @{
  */


/** @defgroup USBD_MOUSE_Exported_Defines
  * @{
  */
#define MOUSE_EPIN_ADDR                 0x81U
#define MOUSE_EPIN_SIZE                 0x04U

#define USB_MOUSE_CONFIG_DESC_SIZ       34U
#define USB_MOUSE_DESC_SIZ              9U
#define HID_MOUSE_REPORT_DESC_SIZE    74U

#define MOUSE_DESCRIPTOR_TYPE           0x21U
#define MOUSE_REPORT_DESC               0x22U

#ifndef MOUSE_HS_BINTERVAL
#define MOUSE_HS_BINTERVAL            0x07U
#endif /* HID_HS_BINTERVAL */

#ifndef MOUSE_FS_BINTERVAL
#define MOUSE_FS_BINTERVAL            0x0AU
#endif /* HID_FS_BINTERVAL */

#define MOUSE_REQ_SET_PROTOCOL          0x0BU
#define MOUSE_REQ_GET_PROTOCOL          0x03U

#define MOUSE_REQ_SET_IDLE              0x0AU
#define MOUSE_REQ_GET_IDLE              0x02U

#define MOUSE_REQ_SET_REPORT            0x09U
#define MOUSE_REQ_GET_REPORT            0x01U
/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef enum
{
	MOUSE_IDLE = 0,
	MOUSE_BUSY,
}
HID_StateTypeDef;


typedef struct
{
  uint32_t             Protocol;
  uint32_t             IdleState;
  uint32_t             AltSetting;
  HID_StateTypeDef     state;
}
USBD_MOUSE_HandleTypeDef;
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

extern USBD_ClassTypeDef  USBD_MOUSE;
#define USBD_MOUSE_CLASS    &USBD_MOUSE
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_MOUSE_SendReport(USBD_HandleTypeDef *pdev,
                            uint8_t *report,
                            uint16_t len);

uint32_t USBD_MOUSE_GetPollingInterval(USBD_HandleTypeDef *pdev);



uint8_t  USBD_MOUSE_Init(USBD_HandleTypeDef *pdev,
                              uint8_t cfgidx);

uint8_t  USBD_MOUSE_DeInit(USBD_HandleTypeDef *pdev,
                                uint8_t cfgidx);

uint8_t  USBD_MOUSE_Setup(USBD_HandleTypeDef *pdev,
                               USBD_SetupReqTypedef *req);

uint8_t  *USBD_MOUSE_GetFSCfgDesc(uint16_t *length);

uint8_t  *USBD_MOUSE_GetHSCfgDesc(uint16_t *length);

uint8_t  *USBD_MOUSE_GetOtherSpeedCfgDesc(uint16_t *length);

uint8_t  *USBD_MOUSE_GetDeviceQualifierDesc(uint16_t *length);

uint8_t  USBD_MOUSE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_MOUSE_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
