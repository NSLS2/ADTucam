/**
 * ICameraSDK.h
 *
 * Interface for camera SDK operations to enable unit testing and mocking.
 *
 * Author: Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */

#ifndef TUCAMAPP_TUCAMSRC_ICAMERASDK_H_
#define TUCAMAPP_TUCAMSRC_ICAMERASDK_H_

// TUCAM includes
#include "TUCamApi.h"
#include "TUDefine.h"

/**
 * @brief Interface for camera SDK operations
 *
 * This interface abstracts all TUCAM API calls to enable unit testing
 * and mocking of the camera functionality.
 */
class ICameraSDK {
 public:
  virtual ~ICameraSDK() = default;

  // API lifecycle operations
  virtual TUCAMRET initializeAPI(TUCAM_INIT* apiHandle) = 0;
  virtual TUCAMRET uninitializeAPI() = 0;

  // Device operations
  virtual TUCAMRET openDevice(TUCAM_OPEN* camHandle) = 0;
  virtual TUCAMRET closeDevice(void* deviceHandle) = 0;
  virtual TUCAMRET getDeviceInfo(void* deviceHandle,
                                 TUCAM_VALUE_INFO* valueInfo) = 0;

  // Buffer operations
  virtual TUCAMRET allocateBuffer(void* deviceHandle,
                                  TUCAM_FRAME* frameHandle) = 0;
  virtual TUCAMRET releaseBuffer(void* deviceHandle) = 0;
  virtual TUCAMRET waitForFrame(void* deviceHandle, TUCAM_FRAME* frameHandle,
                                double timeout) = 0;
  virtual TUCAMRET abortWait(void* deviceHandle) = 0;

  // Capture operations
  virtual TUCAMRET startCapture(void* deviceHandle, int triggerMode) = 0;
  virtual TUCAMRET stopCapture(void* deviceHandle) = 0;
  virtual TUCAMRET doSoftwareTrigger(void* deviceHandle) = 0;

  // Trigger operations
  virtual TUCAMRET getTrigger(void* deviceHandle,
                              TUCAM_TRIGGER_ATTR* triggerAttr) = 0;
  virtual TUCAMRET setTrigger(void* deviceHandle,
                              TUCAM_TRIGGER_ATTR triggerAttr) = 0;
  virtual TUCAMRET getTriggerOut(void* deviceHandle,
                                 TUCAM_TRGOUT_ATTR* triggerOutAttr) = 0;
  virtual TUCAMRET setTriggerOut(void* deviceHandle,
                                 TUCAM_TRGOUT_ATTR triggerOutAttr) = 0;

  // ROI operations
  virtual TUCAMRET getROI(void* deviceHandle, TUCAM_ROI_ATTR* roiAttr) = 0;
  virtual TUCAMRET setROI(void* deviceHandle, TUCAM_ROI_ATTR roiAttr) = 0;

  // Property operations
  virtual TUCAMRET getPropertyValue(void* deviceHandle, int property,
                                    double* value) = 0;
  virtual TUCAMRET setPropertyValue(void* deviceHandle, int property,
                                    double value) = 0;
  virtual TUCAMRET getPropertyAttr(void* deviceHandle,
                                   TUCAM_PROP_ATTR* propAttr) = 0;

  // Capability operations
  virtual TUCAMRET getCapabilityValue(void* deviceHandle, int capability,
                                      int* value) = 0;
  virtual TUCAMRET setCapabilityValue(void* deviceHandle, int capability,
                                      int value) = 0;
  virtual TUCAMRET getCapabilityAttr(void* deviceHandle,
                                     TUCAM_CAPA_ATTR* capaAttr) = 0;
  virtual TUCAMRET getCapabilityValueText(void* deviceHandle,
                                          TUCAM_VALUE_TEXT* valueText) = 0;

  // Register operations
  virtual TUCAMRET readRegister(void* deviceHandle, TUCAM_REG_RW regRW) = 0;
};

#endif  // TUCAMAPP_TUCAMSRC_ICAMERASDK_H_
