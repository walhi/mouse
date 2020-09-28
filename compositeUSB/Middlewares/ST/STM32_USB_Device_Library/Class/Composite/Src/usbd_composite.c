/**
 ******************************************************************************
 * @file    usbd_customhid.c
 * @author  MCD Application Team
 * @brief   This file provides the COMPOSITE core functions.
 *
 * @verbatim
 *
 *          ===================================================================
 *                                COMPOSITE Class  Description
 *          ===================================================================
 *           This module manages the COMPOSITE class V1.11 following the "Device Class Definition
 *           for Human Interface Devices (COMPOSITE) Version 1.11 Jun 27, 2001".
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


/* Includes ------------------------------------------------------------------*/
#include "../Inc/usbd_composite.h"
#include "usbd_mouse.h"
#include "usbd_customhid.h"
#include "usbd_ctlreq.h"

static uint8_t  USBD_COMPOSITE_Init(USBD_HandleTypeDef *pdev,
		uint8_t cfgidx);

static uint8_t  USBD_COMPOSITE_DeInit(USBD_HandleTypeDef *pdev,
		uint8_t cfgidx);

static uint8_t  USBD_COMPOSITE_Setup(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req);

static uint8_t  *USBD_COMPOSITE_GetFSCfgDesc(uint16_t *length);

static uint8_t  *USBD_COMPOSITE_GetHSCfgDesc(uint16_t *length);

static uint8_t  *USBD_COMPOSITE_GetOtherSpeedCfgDesc(uint16_t *length);

static uint8_t  *USBD_COMPOSITE_GetDeviceQualifierDesc(uint16_t *length);

static uint8_t  USBD_COMPOSITE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_COMPOSITE_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef  *pdev);

USBD_ClassTypeDef  USBD_COMPOSITE =
{
		USBD_COMPOSITE_Init,
		USBD_COMPOSITE_DeInit,
		USBD_COMPOSITE_Setup,
		NULL, /*EP0_TxSent*/
		USBD_COMPOSITE_EP0_RxReady, /*EP0_RxReady*/ /* STATUS STAGE IN */
		USBD_COMPOSITE_DataIn, /*DataIn*/
		USBD_COMPOSITE_DataOut,
		NULL, /*SOF */
		NULL,
		NULL,
		USBD_COMPOSITE_GetHSCfgDesc,
		USBD_COMPOSITE_GetFSCfgDesc,
		USBD_COMPOSITE_GetOtherSpeedCfgDesc,
		USBD_COMPOSITE_GetDeviceQualifierDesc,
};

/* USB COMPOSITE device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_CfgFSDesc[USB_COMPOSITE_CONFIG_DESC_SIZ] __ALIGN_END =
{
		9, //bLength
		USB_DESC_TYPE_CONFIGURATION, //bDescriptorType: config
		USB_COMPOSITE_CONFIG_DESC_SIZ, 0x00, // wTotalLength:00
		0x02,	// bNumInterfaces
		0x01, //bConfigurationValue
		0x03, //iConfiguration
		0xC0,         /*bmAttributes: bus powered */
		//USB_INITIAL_FEATURE, //bmAttributes
		50, //maxPower == 100mA


		// Interface0 - Mouse
		/************** Descriptor of Mouse interface ****************/
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		MOUSE_INTERFACE_IDX,/*bInterfaceNumber: Number of Interface*/
		0x00,         /*bAlternateSetting: Alternate setting*/
		0x01,         /*bNumEndpoints*/
		0x03,         /*bInterfaceClass: HID*/
		0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
		0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
		0,            /*iInterface: Index of string descriptor*/
		/******************** Descriptor of Mouse HID ********************/
		0x09,         /*bLength: HID Descriptor size*/
		MOUSE_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
		0x11,         /*bcdHID: HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
		0x00,
		/******************** Descriptor of Mouse endpoint ********************/
		0x07,          /*bLength: Endpoint Descriptor size*/
		USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
		HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
		0x03,          /*bmAttributes: Interrupt endpoint*/
		HID_EPIN_SIZE, /*wMaxPacketSize: 4 Byte max */
		0x00,
		MOUSE_FS_BINTERVAL,          /*bInterval: Polling Interval */

		// Interface1 - Custom hid
		/************** Descriptor of CUSTOM HID interface ****************/
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		COMPOSITE_INTERFACE_IDX,/*bInterfaceNumber: Number of Interface*/
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

/* USB COMPOSITE device HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_CfgHSDesc[USB_COMPOSITE_CONFIG_DESC_SIZ] __ALIGN_END =
{
		9, //bLength
		USB_DESC_TYPE_CONFIGURATION, //bDescriptorType: config
		USB_COMPOSITE_CONFIG_DESC_SIZ, 0x00, // wTotalLength:00
		0x02,	// bNumInterfaces
		0x01, //bConfigurationValue
		0x03, //iConfiguration
		0xC0,         /*bmAttributes: bus powered */
		//USB_INITIAL_FEATURE, //bmAttributes
		50, //maxPower == 100mA


		// Interface0 - Mouse
		/************** Descriptor of Mouse interface ****************/
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		MOUSE_INTERFACE_IDX,/*bInterfaceNumber: Number of Interface*/
		0x00,         /*bAlternateSetting: Alternate setting*/
		0x01,         /*bNumEndpoints*/
		0x03,         /*bInterfaceClass: HID*/
		0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
		0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
		0,            /*iInterface: Index of string descriptor*/
		/******************** Descriptor of Mouse HID ********************/
		0x09,         /*bLength: HID Descriptor size*/
		HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
		0x11,         /*bcdHID: HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
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
		COMPOSITE_INTERFACE_IDX,/*bInterfaceNumber: Number of Interface*/
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

/* USB COMPOSITE device Other Speed Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_OtherSpeedCfgDesc[USB_COMPOSITE_CONFIG_DESC_SIZ] __ALIGN_END =
{
		9, //bLength
		USB_DESC_TYPE_CONFIGURATION, //bDescriptorType: config
		USB_COMPOSITE_CONFIG_DESC_SIZ, 0x00, // wTotalLength:00
		0x02,	// bNumInterfaces
		0x01, //bConfigurationValue
		0x03, //iConfiguration
		0xC0,         /*bmAttributes: bus powered */
		//USB_INITIAL_FEATURE, //bmAttributes
		50, //maxPower == 100mA


		// Interface0 - Mouse
		/************** Descriptor of Mouse interface ****************/
		0x09,         /*bLength: Interface Descriptor size*/
		USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
		MOUSE_INTERFACE_IDX,/*bInterfaceNumber: Number of Interface*/
		0x00,         /*bAlternateSetting: Alternate setting*/
		0x01,         /*bNumEndpoints*/
		0x03,         /*bInterfaceClass: HID*/
		0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
		0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
		0,            /*iInterface: Index of string descriptor*/
		/******************** Descriptor of Mouse HID ********************/
		0x09,         /*bLength: HID Descriptor size*/
		HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
		0x11,         /*bcdHID: HID Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
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
		COMPOSITE_INTERFACE_IDX,/*bInterfaceNumber: Number of Interface*/
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

/* USB COMPOSITE device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_Desc[USB_COMPOSITE_DESC_SIZ] __ALIGN_END =
{
		/* 18 */
		0x09,         /*bLength: COMPOSITE Descriptor size*/
		COMPOSITE_DESCRIPTOR_TYPE, /*bDescriptorType: COMPOSITE*/
		0x11,         /*bCOMPOSITEUSTOM_HID: COMPOSITE Class Spec release number*/
		0x01,
		0x00,         /*bCountryCode: Hardware target country*/
		0x01,         /*bNumDescriptors: Number of COMPOSITE class descriptors to follow*/
		0x22,         /*bDescriptorType*/
		USBD_COMPOSITE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
		0x00,
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

static uint8_t  USBD_COMPOSITE_Init(USBD_HandleTypeDef *pdev,
		uint8_t cfgidx)
{
	/* Mouse initialization */
	uint8_t ret = USBD_MOUSE_Init (pdev, cfgidx);
	if(ret != USBD_OK)
		return ret;

	/* Custom HID initialization */
	ret = USBD_CUSTOM_HID_Init (pdev, cfgidx);
	if(ret != USBD_OK)
		return ret;

	return USBD_OK;
}

/**
 * @brief  USBD_COMPOSITE_Init
 *         DeInitialize the COMPOSITE layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_DeInit(USBD_HandleTypeDef *pdev,
		uint8_t cfgidx)
{
	/* Mouse initialization */
	uint8_t ret = USBD_MOUSE_DeInit (pdev, cfgidx);
	if(ret != USBD_OK)
		return ret;

	/* Custom HID initialization */
	ret = USBD_CUSTOM_HID_DeInit (pdev, cfgidx);
	if(ret != USBD_OK)
		return ret;

	return USBD_OK;
}

/**
 * @brief  USBD_COMPOSITE_Setup
 *         Handle the COMPOSITE specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_Setup(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req)
{
	// Route requests to MSC interface or its endpoints to MSC class implementaion
	if(((req->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_INTERFACE && req->wIndex == MOUSE_INTERFACE_IDX) ||
			((req->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_ENDPOINT && ((req->wIndex & 0x7F) == MOUSE_EP_IDX)))
	{
		return USBD_MOUSE_Setup(pdev, req);
	}

	return USBD_CUSTOM_HID_Setup(pdev, req);
}

/**
 * @brief  USBD_COMPOSITE_SendReport
 *         Send COMPOSITE Report
 * @param  pdev: device instance
 * @param  buff: pointer to report
 * @retval status
 */
uint8_t USBD_COMPOSITE_SendReport(USBD_HandleTypeDef  *pdev,
		uint8_t *report,
		uint16_t len)
{
	return USBD_OK;
}

/**
 * @brief  USBD_COMPOSITE_GetFSCfgDesc
 *         return FS configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_COMPOSITE_GetFSCfgDesc(uint16_t *length)
{
	*length = sizeof(USBD_COMPOSITE_CfgFSDesc);
	return USBD_COMPOSITE_CfgFSDesc;
}

/**
 * @brief  USBD_COMPOSITE_GetHSCfgDesc
 *         return HS configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_COMPOSITE_GetHSCfgDesc(uint16_t *length)
{
	*length = sizeof(USBD_COMPOSITE_CfgHSDesc);
	return USBD_COMPOSITE_CfgHSDesc;
}

/**
 * @brief  USBD_COMPOSITE_GetOtherSpeedCfgDesc
 *         return other speed configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_COMPOSITE_GetOtherSpeedCfgDesc(uint16_t *length)
{
	*length = sizeof(USBD_COMPOSITE_OtherSpeedCfgDesc);
	return USBD_COMPOSITE_OtherSpeedCfgDesc;
}

/**
 * @brief  USBD_COMPOSITE_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_DataIn(USBD_HandleTypeDef *pdev,
		uint8_t epnum)
{
	if(epnum == MOUSE_EP_IDX)
		return USBD_MOUSE_DataIn(pdev, epnum);

	return USBD_CUSTOM_HID_DataIn(pdev, epnum);
}

/**
 * @brief  USBD_COMPOSITE_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_DataOut(USBD_HandleTypeDef *pdev,
		uint8_t epnum)
{
	if(epnum == MOUSE_EP_IDX)
		return USBD_OK; // USBD_MOUSE_DataOut(pdev, epnum); // не нужна для мыши

	return USBD_CUSTOM_HID_DataOut(pdev, epnum);
}

/**
 * @brief  USBD_COMPOSITE_EP0_RxReady
 *         Handles control request data.
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
	return USBD_CUSTOM_HID_EP0_RxReady(pdev);
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_COMPOSITE_GetDeviceQualifierDesc(uint16_t *length)
{
	*length = sizeof(USBD_COMPOSITE_DeviceQualifierDesc);
	return USBD_COMPOSITE_DeviceQualifierDesc;
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
