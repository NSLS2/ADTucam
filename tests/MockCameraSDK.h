/**
 * MockCameraSDK.h
 *
 * Google Mock implementation of ICameraSDK for unit testing.
 *
 * Author: Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */

#ifndef TESTS_MOCKCAMERASDK_H_
#define TESTS_MOCKCAMERASDK_H_

#include <gmock/gmock.h>

#include "ICameraSDK.h"

/**
 * @brief Google Mock implementation of ICameraSDK interface
 *
 * This mock class allows unit testing of ADTucam by controlling
 * the behavior of all camera SDK operations.
 */
class MockCameraSDK : public ICameraSDK {
 public:
  // API lifecycle operations
  MOCK_METHOD(TUCAMRET, initializeAPI, (TUCAM_INIT * apiHandle), (override));
  MOCK_METHOD(TUCAMRET, uninitializeAPI, (), (override));

  // Device operations
  MOCK_METHOD(TUCAMRET, openDevice, (TUCAM_OPEN * camHandle), (override));
  MOCK_METHOD(TUCAMRET, closeDevice, (void* deviceHandle), (override));
  MOCK_METHOD(TUCAMRET, getDeviceInfo,
              (void* deviceHandle, TUCAM_VALUE_INFO* valueInfo), (override));

  // Buffer operations
  MOCK_METHOD(TUCAMRET, allocateBuffer,
              (void* deviceHandle, TUCAM_FRAME* frameHandle), (override));
  MOCK_METHOD(TUCAMRET, releaseBuffer, (void* deviceHandle), (override));
  MOCK_METHOD(TUCAMRET, waitForFrame,
              (void* deviceHandle, TUCAM_FRAME* frameHandle, double timeout),
              (override));
  MOCK_METHOD(TUCAMRET, abortWait, (void* deviceHandle), (override));

  // Capture operations
  MOCK_METHOD(TUCAMRET, startCapture, (void* deviceHandle, int triggerMode),
              (override));
  MOCK_METHOD(TUCAMRET, stopCapture, (void* deviceHandle), (override));
  MOCK_METHOD(TUCAMRET, doSoftwareTrigger, (void* deviceHandle), (override));

  // Trigger operations
  MOCK_METHOD(TUCAMRET, getTrigger,
              (void* deviceHandle, TUCAM_TRIGGER_ATTR* triggerAttr),
              (override));
  MOCK_METHOD(TUCAMRET, setTrigger,
              (void* deviceHandle, TUCAM_TRIGGER_ATTR triggerAttr), (override));
  MOCK_METHOD(TUCAMRET, getTriggerOut,
              (void* deviceHandle, TUCAM_TRGOUT_ATTR* triggerOutAttr),
              (override));
  MOCK_METHOD(TUCAMRET, setTriggerOut,
              (void* deviceHandle, TUCAM_TRGOUT_ATTR triggerOutAttr),
              (override));

  // ROI operations
  MOCK_METHOD(TUCAMRET, getROI, (void* deviceHandle, TUCAM_ROI_ATTR* roiAttr),
              (override));
  MOCK_METHOD(TUCAMRET, setROI, (void* deviceHandle, TUCAM_ROI_ATTR roiAttr),
              (override));

  // Property operations
  MOCK_METHOD(TUCAMRET, getPropertyValue,
              (void* deviceHandle, int property, double* value), (override));
  MOCK_METHOD(TUCAMRET, setPropertyValue,
              (void* deviceHandle, int property, double value), (override));
  MOCK_METHOD(TUCAMRET, getPropertyAttr,
              (void* deviceHandle, TUCAM_PROP_ATTR* propAttr), (override));

  // Capability operations
  MOCK_METHOD(TUCAMRET, getCapabilityValue,
              (void* deviceHandle, int capability, int* value), (override));
  MOCK_METHOD(TUCAMRET, setCapabilityValue,
              (void* deviceHandle, int capability, int value), (override));
  MOCK_METHOD(TUCAMRET, getCapabilityAttr,
              (void* deviceHandle, TUCAM_CAPA_ATTR* capaAttr), (override));
  MOCK_METHOD(TUCAMRET, getCapabilityValueText,
              (void* deviceHandle, TUCAM_VALUE_TEXT* valueText), (override));

  // Register operations
  MOCK_METHOD(TUCAMRET, readRegister, (void* deviceHandle, TUCAM_REG_RW regRW),
              (override));
};

#endif  // TESTS_MOCKCAMERASDK_H_
