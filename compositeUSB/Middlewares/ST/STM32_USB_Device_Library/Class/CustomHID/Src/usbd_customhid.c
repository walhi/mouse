/**
 ******************************************************************************
 * @file    usbd_customhid.c
 * @author  MCD Application Team
 * @brief   This file provides the CUSTOM_HID core functions.
 *
 * @verbatim
 *
 *          ===================================================================
 *                                CUSTOM_HID Class  Description
 *          ===================================================================
 *           This module manages the CUSTOM_HID class V1.11 following the "Device Class Definition
 *           for Human Interface Devices (CUSTOM_HID) Version 1.11 Jun 27, 2001".
 *           This driver implements the following aspects of the specification:
 *             - The Boot Interface Subclass
 *             - Usage Page : Generic Desktop
 *             - Usage : Vendor
 *             - Collection : Application
 *
 * @note     In HS mode and when the DMA is used, all variables and data structures
 *           dealing with the DMA during the transaction process should be 32-bit aligned.
 *
 *
 *  @endverbatim
 *
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

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_customhid.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */


/** @defgroup USBD_CUSTOM_HID
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */


/** @defgroup USBD_CUSTOM_HID_Private_Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup USBD_CUSTOM_HID_Private_Macros
 * @{
 */
/**
 * @}
 */
/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes
 * @{
 */



/**
 * @}
 */

/** @defgroup USBD_CUSTOM_HID_Private_Variables
 * @{
 */

USBD_ClassTypeDef  USBD_CUSTOM_HID =
{
		USBD_CUSTOM_HID_Init,
		USBD_CUSTOM_HID_DeInit,
		USBD_CUSTOM_HID_Setup,
		NULL, /*EP0_TxSent*/
		USBD_CUSTOM_HID_EP0_RxReady, /*EP0_RxReady*/ /* STATUS STAGE IN */
		USBD_CUSTOM_HID_DataIn, /*DataIn*/
		USBD_CUSTOM_HID_DataOut,
		NULL, /*SOF */
		NULL,
		NULL,
		USBD_CUSTOM_HID_GetHSCfgDesc,
		USBD_CUSTOM_HID_GetFSCfgDesc,
		USBD_CUSTOM_HID_GetOtherSpeedCfgDesc,
		USBD_CUSTOM_HID_GetDeviceQualifierDesc,
};

/* USB CUSTOM_HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_CfgFSDesc[USB_CUSTOM_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
		9, //bLength
		USB_DESC_TYPE_CONFIGURATION, //bDescriptorType: config
		USB_CUSTOM_HID_CONFIG_DESC_SIZ, 0x00, // wTotalLength:00
		0x02,	// bNumInterfaces
		0x01, //bConfigurationValue
		0x03, //iConfiguration
		0xC0,         /*bmAttributes: bus powered */
		//USB_INITIAL_FEATURE, //bmAttributes
		50, //maxPower == 100mA


		// Interface0 - Custom hid
		/************** Descriptor of CUSTOM HID interface ****************/
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		0x00,         /*bInterfaceNumber: Number of Interface*/
		0x00,         /*bAlternateSetting: Alternate setting*/
		0x02,         /*bNumEndpoints*/
		0x03,         /*bInterfaceClass: CUSTOM_HID*/
		0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
		0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
		0,            /*iInterface: Index of string descriptor*/
		/******************** Descriptor of CUSTOM_HID *************************/
		0x09,         /*bLength: CUSTOM_HID Descriptor size*/
		CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
		0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
		0x00,
		/******************** Descriptor of Custom HID endpoints ********************/
		0x07,          /*bLength: Endpoint Descriptor size*/
		USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
		CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
		0x03,          /*bmAttributes: Interrupt endpoint*/
		CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
		0x00,
		CUSTOM_HID_HS_BINTERVAL,          /*bInterval: Polling Interval */

		0x07,          /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
		CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
		0x03, /* bmAttributes: Interrupt endpoint */
		CUSTOM_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
		0x00,
		CUSTOM_HID_FS_BINTERVAL,  /* bInterval: Polling Interval */

		// Interface1 - Custom hid
		/************** Descriptor of CUSTOM HID interface ****************/
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		0x01,         /*bInterfaceNumber: Number of Interface*/
		0x00,         /*bAlternateSetting: Alternate setting*/
		0x02,         /*bNumEndpoints*/
		0x03,         /*bInterfaceClass: CUSTOM_HID*/
		0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
		0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
		0,            /*iInterface: Index of string descriptor*/
		/******************** Descriptor of CUSTOM_HID *************************/
		0x09,         /*bLength: CUSTOM_HID Descriptor size*/
		CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
		0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
		0x00,
		/******************** Descriptor of Custom HID endpoints ********************/
		0x07,          /*bLength: Endpoint Descriptor size*/
		USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
		CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
		0x03,          /*bmAttributes: Interrupt endpoint*/
		CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
		0x00,
		CUSTOM_HID_HS_BINTERVAL,          /*bInterval: Polling Interval */

		0x07,          /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
		CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
		0x03, /* bmAttributes: Interrupt endpoint */
		CUSTOM_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
		0x00,
		CUSTOM_HID_FS_BINTERVAL,  /* bInterval: Polling Interval */

};

/* USB CUSTOM_HID device HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_CfgHSDesc[USB_CUSTOM_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
		9, //bLength
		USB_DESC_TYPE_CONFIGURATION, //bDescriptorType: config
		USB_CUSTOM_HID_CONFIG_DESC_SIZ, 0x00, // wTotalLength:00
		0x02,	// bNumInterfaces
		0x01, //bConfigurationValue
		0x03, //iConfiguration
		0xC0,         /*bmAttributes: bus powered */
		//USB_INITIAL_FEATURE, //bmAttributes
		50, //maxPower == 100mA

		// Interface0 - Mouse
		/************** Descriptor of Mouse interface ****************/
		/* 09 */
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		0x00,         /*bInterfaceNumber: Number of Interface*/
		0x00,         /*bAlternateSetting: Alternate setting*/
		0x01,         /*bNumEndpoints*/
		0x03,         /*bInterfaceClass: HID*/
		0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
		0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
		0,            /*iInterface: Index of string descriptor*/
		/******************** Descriptor of Joystick Mouse HID ********************/
		0x09,         /*bLength: HID Descriptor size*/
		HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
		0x11,         /*bcdHID: HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
		0x00,
		/******************** Descriptor of Mouse endpoint ********************/
		0x07,          /*bLength: Endpoint Descriptor size*/
		USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
		HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
		0x03,          /*bmAttributes: Interrupt endpoint*/
		HID_EPIN_SIZE, /*wMaxPacketSize: 4 Byte max */
		0x00,
		HID_HS_BINTERVAL,          /*bInterval: Polling Interval */

		// Interface1 - Custom hid
		/************** Descriptor of CUSTOM HID interface ****************/
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		0x01,         /*bInterfaceNumber: Number of Interface*/
		0x00,         /*bAlternateSetting: Alternate setting*/
		0x02,         /*bNumEndpoints*/
		0x03,         /*bInterfaceClass: CUSTOM_HID*/
		0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
		0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
		0,            /*iInterface: Index of string descriptor*/
		/******************** Descriptor of CUSTOM_HID *************************/
		0x09,         /*bLength: CUSTOM_HID Descriptor size*/
		CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
		0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
		0x00,
		/******************** Descriptor of Custom HID endpoints ********************/
		0x07,          /*bLength: Endpoint Descriptor size*/
		USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
		CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
		0x03,          /*bmAttributes: Interrupt endpoint*/
		CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
		0x00,
		CUSTOM_HID_HS_BINTERVAL,          /*bInterval: Polling Interval */

		0x07,          /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
		CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
		0x03, /* bmAttributes: Interrupt endpoint */
		CUSTOM_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
		0x00,
		CUSTOM_HID_HS_BINTERVAL,  /* bInterval: Polling Interval */
};

/* USB CUSTOM_HID device Other Speed Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_OtherSpeedCfgDesc[USB_CUSTOM_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
		9, //bLength
		USB_DESC_TYPE_CONFIGURATION, //bDescriptorType: config
		USB_CUSTOM_HID_CONFIG_DESC_SIZ, 0x00, // wTotalLength:00
		0x02,	// bNumInterfaces
		0x01, //bConfigurationValue
		0x03, //iConfiguration
		0xC0,         /*bmAttributes: bus powered */
		//USB_INITIAL_FEATURE, //bmAttributes
		50, //maxPower == 100mA

		// Interface0 - Mouse
		/************** Descriptor of Mouse interface ****************/
		/* 09 */
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		0x00,         /*bInterfaceNumber: Number of Interface*/
		0x00,         /*bAlternateSetting: Alternate setting*/
		0x01,         /*bNumEndpoints*/
		0x03,         /*bInterfaceClass: HID*/
		0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
		0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
		0,            /*iInterface: Index of string descriptor*/
		/******************** Descriptor of Joystick Mouse HID ********************/
		0x09,         /*bLength: HID Descriptor size*/
		HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
		0x11,         /*bcdHID: HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
		0x00,
		/******************** Descriptor of Mouse endpoint ********************/
		0x07,          /*bLength: Endpoint Descriptor size*/
		USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
		HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
		0x03,          /*bmAttributes: Interrupt endpoint*/
		HID_EPIN_SIZE, /*wMaxPacketSize: 4 Byte max */
		0x00,
		HID_FS_BINTERVAL,          /*bInterval: Polling Interval */

		// Interface1 - Custom hid
		/************** Descriptor of CUSTOM HID interface ****************/
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		0x01,         /*bInterfaceNumber: Number of Interface*/
		0x00,         /*bAlternateSetting: Alternate setting*/
		0x02,         /*bNumEndpoints*/
		0x03,         /*bInterfaceClass: CUSTOM_HID*/
		0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
		0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
		0,            /*iInterface: Index of string descriptor*/
		/******************** Descriptor of CUSTOM_HID *************************/
		0x09,         /*bLength: CUSTOM_HID Descriptor size*/
		CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
		0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
		0x00,
		/******************** Descriptor of Custom HID endpoints ********************/
		0x07,          /*bLength: Endpoint Descriptor size*/
		USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
		CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
		0x03,          /*bmAttributes: Interrupt endpoint*/
		CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
		0x00,
		CUSTOM_HID_HS_BINTERVAL,          /*bInterval: Polling Interval */

		0x07,          /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
		CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
		0x03, /* bmAttributes: Interrupt endpoint */
		CUSTOM_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
		0x00,
		CUSTOM_HID_FS_BINTERVAL,  /* bInterval: Polling Interval */
};

/* USB CUSTOM_HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_Desc[USB_CUSTOM_HID_DESC_SIZ] __ALIGN_END =
{
		/* 18 */
		0x09,         /*bLength: CUSTOM_HID Descriptor size*/
		CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
		0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
		0x00,
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
		USB_LEN_DEV_QUALIFIER_DESC,
		USB_DESC_TYPE_DEVICE_QUALIFIER,
		0x00,
		0x02,
		0x00,
		0x00,
		0x00,
		0x40,
		0x01,
		0x00,
};

/**
 * @}
 */

/** @defgroup USBD_CUSTOM_HID_Private_Functions
 * @{
 */

USBD_CUSTOM_HID_HandleTypeDef customHIDInstance;

/**
 * @brief  USBD_CUSTOM_HID_Init
 *         Initialize the CUSTOM_HID interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
uint8_t  USBD_CUSTOM_HID_Init(USBD_HandleTypeDef *pdev,
		uint8_t cfgidx)
{
	uint8_t ret = 0U;
	USBD_CUSTOM_HID_HandleTypeDef     *hhid;

	/* Open EP IN */
	USBD_LL_OpenEP(pdev, CUSTOM_HID_EPIN_ADDR, USBD_EP_TYPE_INTR,
			CUSTOM_HID_EPIN_SIZE);

	pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].is_used = 1U;

	/* Open EP OUT */
	USBD_LL_OpenEP(pdev, CUSTOM_HID_EPOUT_ADDR, USBD_EP_TYPE_INTR,
			CUSTOM_HID_EPOUT_SIZE);

	pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].is_used = 1U;

	//pdev->pClassDataCustomHID = USBD_malloc(sizeof(USBD_CUSTOM_HID_HandleTypeDef));
	pdev->pClassDataCustomHID = &customHIDInstance;

	if (pdev->pClassDataCustomHID == NULL)
	{
		ret = 1U;
	}
	else
	{
		hhid = (USBD_CUSTOM_HID_HandleTypeDef *) pdev->pClassDataCustomHID;

		hhid->state = CUSTOM_HID_IDLE;
		((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pClassSpecificInterfaceCustomHID)->Init();

		/* Prepare Out endpoint to receive 1st packet */
		USBD_LL_PrepareReceive(pdev, CUSTOM_HID_EPOUT_ADDR, hhid->Report_buf,
				USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);
	}

	return ret;
}

/**
 * @brief  USBD_CUSTOM_HID_Init
 *         DeInitialize the CUSTOM_HID layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
uint8_t  USBD_CUSTOM_HID_DeInit(USBD_HandleTypeDef *pdev,
		uint8_t cfgidx)
{
	/* Close CUSTOM_HID EP IN */
	USBD_LL_CloseEP(pdev, CUSTOM_HID_EPIN_ADDR);
	pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].is_used = 0U;

	/* Close CUSTOM_HID EP OUT */
	USBD_LL_CloseEP(pdev, CUSTOM_HID_EPOUT_ADDR);
	pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].is_used = 0U;

	/* FRee allocated memory */
	if (pdev->pClassDataCustomHID != NULL)
	{
		((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pClassSpecificInterfaceCustomHID)->DeInit();
		// USBD_free(pdev->pClassDataCustomHID); // Статическое выделение
		pdev->pClassDataCustomHID = NULL;
	}
	return USBD_OK;
}



__ALIGN_BEGIN static uint8_t HID_CUSTOM_ReportDesc[USBD_CUSTOM_HID_REPORT_DESC_SIZE]  __ALIGN_END =
{
		  /* USER CODE BEGIN 0 */
				0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
				    0x09, 0x01,                    // USAGE (Vendor Usage 1)
				    0xa1, 0x01,                    // COLLECTION (Application)
				    0x85, 0x01,                    //   REPORT_ID (1)
				    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
				    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
				    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
				    0x75, 0x08,                    //   REPORT_SIZE (8)
				    0x95, 0x01,                    //   REPORT_COUNT (1)
				    0xb1, 0x82,                    //   FEATURE (Data,Var,Abs,Vol)
				    0x85, 0x01,                    //   REPORT_ID (1)
				    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
				    0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)

				    0x85, 0x02,                    //   REPORT_ID (2)
				    0x09, 0x04,                    //   USAGE (Vendor Usage 4)
				    0x75, 0x08,                    //   REPORT_SIZE (8)
				    0x95, 4,                    //   REPORT_COUNT (N)
				    0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)
		  /* USER CODE END 0 */
		  0xC0    /*     END_COLLECTION	             */
};
/**
 * @brief  USBD_CUSTOM_HID_Setup
 *         Handle the CUSTOM_HID specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
uint8_t  USBD_CUSTOM_HID_Setup(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req)
{
	USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef *)pdev->pClassDataCustomHID;
	uint16_t len = 0U;
	uint8_t  *pbuf = NULL;
	uint16_t status_info = 0U;
	uint8_t ret = USBD_OK;

	switch (req->bmRequest & USB_REQ_TYPE_MASK)
	{
	case USB_REQ_TYPE_CLASS :
		switch (req->bRequest)
		{
		case CUSTOM_HID_REQ_SET_PROTOCOL:
			hhid->Protocol = (uint8_t)(req->wValue);
			break;

		case CUSTOM_HID_REQ_GET_PROTOCOL:
			USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->Protocol, 1U);
			break;

		case CUSTOM_HID_REQ_SET_IDLE:
			hhid->IdleState = (uint8_t)(req->wValue >> 8);
			break;

		case CUSTOM_HID_REQ_GET_IDLE:
			USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->IdleState, 1U);
			break;

		case CUSTOM_HID_REQ_SET_REPORT:
			hhid->IsReportAvailable = 1U;
			USBD_CtlPrepareRx(pdev, hhid->Report_buf, req->wLength);
			break;

		default:
			USBD_CtlError(pdev, req);
			ret = USBD_FAIL;
			break;
		}
		break;

		case USB_REQ_TYPE_STANDARD:
			switch (req->bRequest)
			{
			case USB_REQ_GET_STATUS:
				if (pdev->dev_state == USBD_STATE_CONFIGURED)
				{
					USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
				}
				else
				{
					USBD_CtlError(pdev, req);
					ret = USBD_FAIL;
				}
				break;

			case USB_REQ_GET_DESCRIPTOR:
				if (req->wValue >> 8 == CUSTOM_HID_REPORT_DESC)
				{
					len = MIN(USBD_CUSTOM_HID_REPORT_DESC_SIZE, req->wLength);
					pbuf = HID_CUSTOM_ReportDesc;
				}
				else
				{
					if (req->wValue >> 8 == CUSTOM_HID_DESCRIPTOR_TYPE)
					{
						pbuf = USBD_CUSTOM_HID_Desc;
						len = MIN(USB_CUSTOM_HID_DESC_SIZ, req->wLength);
					}
				}

				USBD_CtlSendData(pdev, pbuf, len);
				break;

			case USB_REQ_GET_INTERFACE :
				if (pdev->dev_state == USBD_STATE_CONFIGURED)
				{
					USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->AltSetting, 1U);
				}
				else
				{
					USBD_CtlError(pdev, req);
					ret = USBD_FAIL;
				}
				break;

			case USB_REQ_SET_INTERFACE :
				if (pdev->dev_state == USBD_STATE_CONFIGURED)
				{
					hhid->AltSetting = (uint8_t)(req->wValue);
				}
				else
				{
					USBD_CtlError(pdev, req);
					ret = USBD_FAIL;
				}
				break;

			default:
				USBD_CtlError(pdev, req);
				ret = USBD_FAIL;
				break;
			}
			break;

			default:
				USBD_CtlError(pdev, req);
				ret = USBD_FAIL;
				break;
	}
	return ret;
}

/**
 * @brief  USBD_CUSTOM_HID_SendReport
 *         Send CUSTOM_HID Report
 * @param  pdev: device instance
 * @param  buff: pointer to report
 * @retval status
 */
uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef  *pdev,
		uint8_t *report,
		uint16_t len)
{
	USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef *)pdev->pClassDataCustomHID;

	if (pdev->dev_state == USBD_STATE_CONFIGURED)
	{
		if (hhid->state == CUSTOM_HID_IDLE)
		{
			hhid->state = CUSTOM_HID_BUSY;
			USBD_LL_Transmit(pdev, CUSTOM_HID_EPIN_ADDR, report, len);
		}
		else
		{
			return USBD_BUSY;
		}
	}
	return USBD_OK;
}

/**
 * @brief  USBD_CUSTOM_HID_GetFSCfgDesc
 *         return FS configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t  *USBD_CUSTOM_HID_GetFSCfgDesc(uint16_t *length)
{
	*length = sizeof(USBD_CUSTOM_HID_CfgFSDesc);
	return USBD_CUSTOM_HID_CfgFSDesc;
}

/**
 * @brief  USBD_CUSTOM_HID_GetHSCfgDesc
 *         return HS configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t  *USBD_CUSTOM_HID_GetHSCfgDesc(uint16_t *length)
{
	*length = sizeof(USBD_CUSTOM_HID_CfgHSDesc);
	return USBD_CUSTOM_HID_CfgHSDesc;
}

/**
 * @brief  USBD_CUSTOM_HID_GetOtherSpeedCfgDesc
 *         return other speed configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t  *USBD_CUSTOM_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
	*length = sizeof(USBD_CUSTOM_HID_OtherSpeedCfgDesc);
	return USBD_CUSTOM_HID_OtherSpeedCfgDesc;
}

/**
 * @brief  USBD_CUSTOM_HID_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
uint8_t  USBD_CUSTOM_HID_DataIn(USBD_HandleTypeDef *pdev,
		uint8_t epnum)
{
	/* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
	((USBD_CUSTOM_HID_HandleTypeDef *)pdev->pClassDataCustomHID)->state = CUSTOM_HID_IDLE;

	return USBD_OK;
}

/**
 * @brief  USBD_CUSTOM_HID_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
uint8_t  USBD_CUSTOM_HID_DataOut(USBD_HandleTypeDef *pdev,
		uint8_t epnum)
{

	USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef *)pdev->pClassDataCustomHID;

	((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pClassSpecificInterfaceCustomHID)->OutEvent(hhid->Report_buf[0],
			hhid->Report_buf[1]);

	USBD_LL_PrepareReceive(pdev, CUSTOM_HID_EPOUT_ADDR, hhid->Report_buf,
			USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);

	return USBD_OK;
}

/**
 * @brief  USBD_CUSTOM_HID_EP0_RxReady
 *         Handles control request data.
 * @param  pdev: device instance
 * @retval status
 */
uint8_t USBD_CUSTOM_HID_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
	USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef *)pdev->pClassDataCustomHID;

	if (hhid->IsReportAvailable == 1U)
	{
		((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pClassSpecificInterfaceCustomHID)->OutEvent(hhid->Report_buf[0],
				hhid->Report_buf[1]);
		hhid->IsReportAvailable = 0U;
	}

	return USBD_OK;
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t  *USBD_CUSTOM_HID_GetDeviceQualifierDesc(uint16_t *length)
{
	*length = sizeof(USBD_CUSTOM_HID_DeviceQualifierDesc);
	return USBD_CUSTOM_HID_DeviceQualifierDesc;
}

/**
 * @brief  USBD_CUSTOM_HID_RegisterInterface
 * @param  pdev: device instance
 * @param  fops: CUSTOMHID Interface callback
 * @retval status
 */
uint8_t  USBD_CUSTOM_HID_RegisterInterface(USBD_HandleTypeDef   *pdev,
		USBD_CUSTOM_HID_ItfTypeDef *fops)
{
	uint8_t  ret = USBD_FAIL;

	if (fops != NULL)
	{
		pdev->pClassSpecificInterfaceCustomHID = fops;
		ret = USBD_OK;
	}

	return ret;
}
/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
