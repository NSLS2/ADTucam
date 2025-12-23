/**
 * TUCAMSDKHandler.h
 *
 * Concrete implementation of ICameraSDK interface using TUCAM SDK.
 *
 * Author: Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */

#ifndef TUCAMAPP_TUCAMSRC_TUCAMSDKHANDLER_H_
#define TUCAMAPP_TUCAMSRC_TUCAMSDKHANDLER_H_

#include "ICameraSDK.h"

/**
 * @brief Concrete implementation of ICameraSDK using TUCAM SDK
 *
 * This class provides a thin wrapper around the TUCAM SDK API calls.
 * It implements the ICameraSDK interface to enable dependency injection
 * and unit testing.
 */
class TUCAMSDKHandler : public ICameraSDK {
 public:
  TUCAMSDKHandler() = default;
  ~TUCAMSDKHandler() = default;

  // API lifecycle operations
  TUCAMRET initializeAPI(TUCAM_INIT* apiHandle) override;
  TUCAMRET uninitializeAPI() override;

  // Device operations
  TUCAMRET openDevice(TUCAM_OPEN* camHandle) override;
  TUCAMRET closeDevice(void* deviceHandle) override;
  TUCAMRET getDeviceInfo(void* deviceHandle,
                         TUCAM_VALUE_INFO* valueInfo) override;

  // Buffer operations
  TUCAMRET allocateBuffer(void* deviceHandle,
                          TUCAM_FRAME* frameHandle) override;
  TUCAMRET releaseBuffer(void* deviceHandle) override;
  TUCAMRET waitForFrame(void* deviceHandle, TUCAM_FRAME* frameHandle,
                        double timeout) override;
  TUCAMRET abortWait(void* deviceHandle) override;

  // Capture operations
  TUCAMRET startCapture(void* deviceHandle, int triggerMode) override;
  TUCAMRET stopCapture(void* deviceHandle) override;
  TUCAMRET doSoftwareTrigger(void* deviceHandle) override;

  // Trigger operations
  TUCAMRET getTrigger(void* deviceHandle,
                      TUCAM_TRIGGER_ATTR* triggerAttr) override;
  TUCAMRET setTrigger(void* deviceHandle,
                      TUCAM_TRIGGER_ATTR triggerAttr) override;
  TUCAMRET getTriggerOut(void* deviceHandle,
                         TUCAM_TRGOUT_ATTR* triggerOutAttr) override;
  TUCAMRET setTriggerOut(void* deviceHandle,
                         TUCAM_TRGOUT_ATTR triggerOutAttr) override;

  // ROI operations
  TUCAMRET getROI(void* deviceHandle, TUCAM_ROI_ATTR* roiAttr) override;
  TUCAMRET setROI(void* deviceHandle, TUCAM_ROI_ATTR roiAttr) override;

  // Property operations
  TUCAMRET getPropertyValue(void* deviceHandle, int property,
                            double* value) override;
  TUCAMRET setPropertyValue(void* deviceHandle, int property,
                            double value) override;
  TUCAMRET getPropertyAttr(void* deviceHandle,
                           TUCAM_PROP_ATTR* propAttr) override;

  // Capability operations
  TUCAMRET getCapabilityValue(void* deviceHandle, int capability,
                              int* value) override;
  TUCAMRET setCapabilityValue(void* deviceHandle, int capability,
                              int value) override;
  TUCAMRET getCapabilityAttr(void* deviceHandle,
                             TUCAM_CAPA_ATTR* capaAttr) override;
  TUCAMRET getCapabilityValueText(void* deviceHandle,
                                  TUCAM_VALUE_TEXT* valueText) override;

  // Register operations
  TUCAMRET readRegister(void* deviceHandle, TUCAM_REG_RW regRW) override;
};

#endif  // TUCAMAPP_TUCAMSRC_TUCAMSDKHANDLER_H_
