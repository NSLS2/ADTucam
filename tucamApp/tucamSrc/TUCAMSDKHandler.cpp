/**
 * TUCAMSDKHandler.cpp
 *
 * Implementation of TUCAMSDKHandler class.
 *
 * Author: Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */

#include "TUCAMSDKHandler.h"

// API lifecycle operations
TUCAMRET TUCAMSDKHandler::initializeAPI(TUCAM_INIT* apiHandle) {
  return TUCAM_Api_Init(apiHandle);
}

TUCAMRET TUCAMSDKHandler::uninitializeAPI() { return TUCAM_Api_Uninit(); }

// Device operations
TUCAMRET TUCAMSDKHandler::openDevice(TUCAM_OPEN* camHandle) {
  return TUCAM_Dev_Open(camHandle);
}

TUCAMRET TUCAMSDKHandler::closeDevice(void* deviceHandle) {
  return TUCAM_Dev_Close(static_cast<HDTUCAM>(deviceHandle));
}

TUCAMRET TUCAMSDKHandler::getDeviceInfo(void* deviceHandle,
                                        TUCAM_VALUE_INFO* valueInfo) {
  return TUCAM_Dev_GetInfo(static_cast<HDTUCAM>(deviceHandle), valueInfo);
}

// Buffer operations
TUCAMRET TUCAMSDKHandler::allocateBuffer(void* deviceHandle,
                                         TUCAM_FRAME* frameHandle) {
  return TUCAM_Buf_Alloc(static_cast<HDTUCAM>(deviceHandle), frameHandle);
}

TUCAMRET TUCAMSDKHandler::releaseBuffer(void* deviceHandle) {
  return TUCAM_Buf_Release(static_cast<HDTUCAM>(deviceHandle));
}

TUCAMRET TUCAMSDKHandler::waitForFrame(void* deviceHandle,
                                       TUCAM_FRAME* frameHandle,
                                       double timeout) {
  return TUCAM_Buf_WaitForFrame(static_cast<HDTUCAM>(deviceHandle), frameHandle,
                                timeout);
}

TUCAMRET TUCAMSDKHandler::abortWait(void* deviceHandle) {
  return TUCAM_Buf_AbortWait(static_cast<HDTUCAM>(deviceHandle));
}

// Capture operations
TUCAMRET TUCAMSDKHandler::startCapture(void* deviceHandle, int triggerMode) {
  return TUCAM_Cap_Start(static_cast<HDTUCAM>(deviceHandle), triggerMode);
}

TUCAMRET TUCAMSDKHandler::stopCapture(void* deviceHandle) {
  return TUCAM_Cap_Stop(static_cast<HDTUCAM>(deviceHandle));
}

TUCAMRET TUCAMSDKHandler::doSoftwareTrigger(void* deviceHandle) {
  return TUCAM_Cap_DoSoftwareTrigger(static_cast<HDTUCAM>(deviceHandle));
}

// Trigger operations
TUCAMRET TUCAMSDKHandler::getTrigger(void* deviceHandle,
                                     TUCAM_TRIGGER_ATTR* triggerAttr) {
  return TUCAM_Cap_GetTrigger(static_cast<HDTUCAM>(deviceHandle), triggerAttr);
}

TUCAMRET TUCAMSDKHandler::setTrigger(void* deviceHandle,
                                     TUCAM_TRIGGER_ATTR triggerAttr) {
  return TUCAM_Cap_SetTrigger(static_cast<HDTUCAM>(deviceHandle), triggerAttr);
}

TUCAMRET TUCAMSDKHandler::getTriggerOut(void* deviceHandle,
                                        TUCAM_TRGOUT_ATTR* triggerOutAttr) {
  return TUCAM_Cap_GetTriggerOut(static_cast<HDTUCAM>(deviceHandle),
                                 triggerOutAttr);
}

TUCAMRET TUCAMSDKHandler::setTriggerOut(void* deviceHandle,
                                        TUCAM_TRGOUT_ATTR triggerOutAttr) {
  return TUCAM_Cap_SetTriggerOut(static_cast<HDTUCAM>(deviceHandle),
                                 triggerOutAttr);
}

// ROI operations
TUCAMRET TUCAMSDKHandler::getROI(void* deviceHandle, TUCAM_ROI_ATTR* roiAttr) {
  return TUCAM_Cap_GetROI(static_cast<HDTUCAM>(deviceHandle), roiAttr);
}

TUCAMRET TUCAMSDKHandler::setROI(void* deviceHandle, TUCAM_ROI_ATTR roiAttr) {
  return TUCAM_Cap_SetROI(static_cast<HDTUCAM>(deviceHandle), roiAttr);
}

// Property operations
TUCAMRET TUCAMSDKHandler::getPropertyValue(void* deviceHandle, int property,
                                           double* value) {
  return TUCAM_Prop_GetValue(static_cast<HDTUCAM>(deviceHandle), property,
                             value);
}

TUCAMRET TUCAMSDKHandler::setPropertyValue(void* deviceHandle, int property,
                                           double value) {
  return TUCAM_Prop_SetValue(static_cast<HDTUCAM>(deviceHandle), property,
                             value);
}

TUCAMRET TUCAMSDKHandler::getPropertyAttr(void* deviceHandle,
                                          TUCAM_PROP_ATTR* propAttr) {
  return TUCAM_Prop_GetAttr(static_cast<HDTUCAM>(deviceHandle), propAttr);
}

// Capability operations
TUCAMRET TUCAMSDKHandler::getCapabilityValue(void* deviceHandle, int capability,
                                             int* value) {
  return TUCAM_Capa_GetValue(static_cast<HDTUCAM>(deviceHandle), capability,
                             value);
}

TUCAMRET TUCAMSDKHandler::setCapabilityValue(void* deviceHandle, int capability,
                                             int value) {
  return TUCAM_Capa_SetValue(static_cast<HDTUCAM>(deviceHandle), capability,
                             value);
}

TUCAMRET TUCAMSDKHandler::getCapabilityAttr(void* deviceHandle,
                                            TUCAM_CAPA_ATTR* capaAttr) {
  return TUCAM_Capa_GetAttr(static_cast<HDTUCAM>(deviceHandle), capaAttr);
}

TUCAMRET TUCAMSDKHandler::getCapabilityValueText(void* deviceHandle,
                                                 TUCAM_VALUE_TEXT* valueText) {
  return TUCAM_Capa_GetValueText(static_cast<HDTUCAM>(deviceHandle), valueText);
}

// Register operations
TUCAMRET TUCAMSDKHandler::readRegister(void* deviceHandle, TUCAM_REG_RW regRW) {
  return TUCAM_Reg_Read(static_cast<HDTUCAM>(deviceHandle), regRW);
}
