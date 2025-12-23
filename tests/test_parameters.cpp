/**
 * test_parameters.cpp
 *
 * Tests for parameter access and setting.
 *
 * Author: Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */

#include <chrono>
#include <future>
#include <string>
#include <thread>

#include "ADTucamTest.h"

using ::testing::_;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Return;

/**
 * Ensure that an error is returned when a parameter write fails.
 */
TEST_F(ADTucamTest, ParameterWriteFailure) {
  // Miscellaneous background operations that will always succeed
  mockConnectionLifecycle();
  EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, releaseBuffer(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getROI(_, _)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getCapabilityValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyAttr(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getCapabilityAttr(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // All set operations should fail
  EXPECT_CALL(*mockSDK, setPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_FAILURE));
  EXPECT_CALL(*mockSDK, setCapabilityValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_FAILURE));
  EXPECT_CALL(*mockSDK, setTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_FAILURE));
  EXPECT_CALL(*mockSDK, setTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_FAILURE));
  EXPECT_CALL(*mockSDK, setROI(_, _)).WillRepeatedly(Return(TUCAMRET_FAILURE));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);

  EXPECT_EQ(writeIntParam(&driver, ADTucam_TECStatusString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_GainModeString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_BinModeString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADMinXString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADMinYString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADSizeXString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADSizeYString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADReverseXString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADReverseYString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTriggerModeString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerExposureString, 0),
            asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut1ModeString, 0),
            asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut1EdgeString, 0),
            asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut2ModeString, 0),
            asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut2EdgeString, 0),
            asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut3ModeString, 0),
            asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut3EdgeString, 0),
            asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_FrameFormatString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_BitDepthString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_FanGearString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_ImageModeString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_AutoExposureString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_AutoLevelsString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_HistogramString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_EnhanceString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_DefectCorrString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_DenoiseString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_FlatCorrString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_PRNUString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_DataFormatString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_EnableGammaString, 0), asynError);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_EnableBlackLevelString, 0),
            asynError);

  EXPECT_EQ(writeFloatParam(&driver, ADAcquirePeriodString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADAcquireTimeString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TemperatureSetpointString, 0.0),
            asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADGainString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerDelayString, 0.0),
            asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut1DelayString, 0.0),
            asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut1WidthString, 0.0),
            asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut2DelayString, 0.0),
            asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut2WidthString, 0.0),
            asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut3DelayString, 0.0),
            asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut3WidthString, 0.0),
            asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_BrightnessString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_BlackLevelString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_SharpnessString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_NoiseLevelString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_HDRKString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_GammaString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_ContrastString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_LeftLevelString, 0.0), asynError);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_RightLevelString, 0.0), asynError);
}

/**
 * Ensure that a parameter write succeeds when all SDK calls succeed.
 */
TEST_F(ADTucamTest, ParameterWriteSuccess) {
  // Miscellaneous background operations that will always succeed
  mockConnectionLifecycle();
  EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, releaseBuffer(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getROI(_, _)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getCapabilityValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyAttr(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getCapabilityAttr(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // All set operations should succeed
  EXPECT_CALL(*mockSDK, setPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, setCapabilityValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, setTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, setTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, setROI(_, _)).WillRepeatedly(Return(TUCAMRET_SUCCESS));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);

  EXPECT_EQ(writeIntParam(&driver, ADTucam_TECStatusString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_GainModeString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_BinModeString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADMinXString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADMinYString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADSizeXString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADSizeYString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADReverseXString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADReverseYString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTriggerModeString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerExposureString, 0),
            asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut1ModeString, 0),
            asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut1EdgeString, 0),
            asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut2ModeString, 0),
            asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut2EdgeString, 0),
            asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut3ModeString, 0),
            asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_TriggerOut3EdgeString, 0),
            asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_FrameFormatString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_BitDepthString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_FanGearString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_ImageModeString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_AutoExposureString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_AutoLevelsString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_HistogramString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_EnhanceString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_DefectCorrString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_DenoiseString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_FlatCorrString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_PRNUString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_DataFormatString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_EnableGammaString, 0), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADTucam_EnableBlackLevelString, 0),
            asynSuccess);

  EXPECT_EQ(writeFloatParam(&driver, ADAcquirePeriodString, 0.0), asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADAcquireTimeString, 0.0), asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TemperatureSetpointString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADGainString, 0.0), asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerDelayString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut1DelayString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut1WidthString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut2DelayString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut2WidthString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut3DelayString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_TriggerOut3WidthString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_BrightnessString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_BlackLevelString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_SharpnessString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_NoiseLevelString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_HDRKString, 0.0), asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_GammaString, 0.0), asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_ContrastString, 0.0), asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_LeftLevelString, 0.0),
            asynSuccess);
  EXPECT_EQ(writeFloatParam(&driver, ADTucam_RightLevelString, 0.0),
            asynSuccess);
}

TEST_F(ADTucamTest, TECStatusControl) {
  // Miscellaneous background operations that will always succeed
  mockConnectionLifecycle();
  EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, releaseBuffer(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getROI(_, _)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // TEC status control succeeds
  EXPECT_CALL(*mockSDK, setCapabilityValue(_, TUIDC_ENABLETEC, _))
      .Times(2)
      .WillRepeatedly(
          Invoke([this](void* deviceHandle, int capability, int value) {
            this->capabilityCache_[capability] = value;
            return TUCAMRET_SUCCESS;
          }));
  EXPECT_CALL(*mockSDK, getCapabilityValue(_, TUIDC_ENABLETEC, _))
      .Times(2)
      .WillRepeatedly(
          Invoke([this](void* deviceHandle, int capability, int* value) {
            *value = this->capabilityCache_[capability];
            return TUCAMRET_SUCCESS;
          }));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADTucam_TECStatusString, 0), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADTucam_TECStatusString, &value),
            asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(writeIntParam(&driver, ADTucam_TECStatusString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADTucam_TECStatusString, &value),
            asynSuccess);
  EXPECT_EQ(value, 1);
}

/**
 * Test that we cannot change the bin mode while acquiring.
 */
TEST_F(ADTucamTest, BinModeControlWhileAcquiring) {
  // Miscellaneous background operations that will always succeed
  mockConnectionLifecycle();
  EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, releaseBuffer(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getROI(_, _)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // Called while acquiring
  EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
      .WillRepeatedly(Invoke(
          [](void* deviceHandle, TUCAM_FRAME* frameHandle, double timeout) {
            frameHandle->usWidth = 100;
            frameHandle->usHeight = 100;
            frameHandle->ucFormat = TUFRM_FMT_RAW;
            frameHandle->ucChannels = 1;
            frameHandle->ucElemBytes = 1;
            frameHandle->uiImgSize = 100 * 100;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            return TUCAMRET_SUCCESS;
          }));
  EXPECT_CALL(*mockSDK, abortWait(_)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, startCapture(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, stopCapture(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // Initial bin mode is 0
  EXPECT_CALL(*mockSDK, setCapabilityValue(_, TUIDC_RESOLUTION, 0))
      .WillOnce(Invoke([this](void* deviceHandle, int capability, int value) {
        this->capabilityCache_[capability] = value;
        return TUCAMRET_SUCCESS;
      }));
  // We get the current bin mode as the readback value
  EXPECT_CALL(*mockSDK, getCapabilityValue(_, TUIDC_RESOLUTION, _))
      .WillOnce(Invoke([this](void* deviceHandle, int capability, int* value) {
        *value = this->capabilityCache_[capability];
        return TUCAMRET_SUCCESS;
      }));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  writeIntParam(&driver, ADTucam_BinModeString, 0);
  writeIntParam(&driver, ADAcquireString, 1);

  int value;
  EXPECT_EQ(writeIntParam(&driver, ADTucam_BinModeString, 1), asynError);
  EXPECT_EQ(readIntParam(&driver, ADTucam_BinModeString, &value), asynSuccess);
  EXPECT_EQ(value, 0);
}

/**
 * Test that we can change the bin mode while acquisition is idle.
 */
TEST_F(ADTucamTest, BinModeControl) {
  // Miscellaneous background operations that will always succeed
  mockConnectionLifecycle();
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getROI(_, _)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // Proper sequence based on actual call order
  {
    InSequence seq;

    // Constructor: Initial allocateBuffer call
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));

    // Bin mode change sequence
    EXPECT_CALL(*mockSDK, releaseBuffer(_)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, setCapabilityValue(_, TUIDC_RESOLUTION, _))
        .WillOnce(Invoke([this](void* deviceHandle, int capability, int value) {
          this->capabilityCache_[capability] = value;
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, getCapabilityValue(_, TUIDC_RESOLUTION, _))
        .WillOnce(
            Invoke([this](void* deviceHandle, int capability, int* value) {
              *value = this->capabilityCache_[capability];
              return TUCAMRET_SUCCESS;
            }));
    EXPECT_CALL(*mockSDK, getROI(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));

    // Destructor: Final releaseBuffer call
    EXPECT_CALL(*mockSDK, releaseBuffer(_)).WillOnce(Return(TUCAMRET_SUCCESS));
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  int value;

  // Set bin mode to 0
  EXPECT_EQ(writeIntParam(&driver, ADTucam_BinModeString, 0), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADTucam_BinModeString, &value), asynSuccess);
  EXPECT_EQ(value, 0);
}

/**
 * Test that we can not start acquisition until the bin mode change has
 * completed.
 *
 * NOTE: Uses futures and promises to ensure proper sequencing of calls.
 */
TEST_F(ADTucamTest, WaitForBinModeChangeBeforeAcquiring) {
  // Miscellaneous background operations that will always succeed
  mockConnectionLifecycle();
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getROI(_, _)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, abortWait(_)).WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // Setup multithreaded promises & futures
  std::promise<void> binModeStartedPromise;
  std::future<void> binModeStartedFuture = binModeStartedPromise.get_future();
  std::promise<void> acquireReadyPromise;
  std::future<void> acquireReadyFuture = acquireReadyPromise.get_future();

  // Proper sequence based on actual call order
  {
    InSequence seq;

    // Constructor: Initial allocateBuffer call
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));

    // Bin mode change sequence
    EXPECT_CALL(*mockSDK, releaseBuffer(_))
        .WillOnce(Invoke(
            [&binModeStartedPromise, &acquireReadyFuture](void* deviceHandle) {
              // Signal that the bin mode change has started
              binModeStartedPromise.set_value();
              // Wait for acquisition to start
              acquireReadyFuture.wait();
              return TUCAMRET_SUCCESS;
            }));
    EXPECT_CALL(*mockSDK, setCapabilityValue(_, TUIDC_RESOLUTION, _))
        .WillOnce(Invoke([this](void* deviceHandle, int capability, int value) {
          this->capabilityCache_[capability] = value;
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, getCapabilityValue(_, TUIDC_RESOLUTION, _))
        .WillOnce(
            Invoke([this](void* deviceHandle, int capability, int* value) {
              *value = this->capabilityCache_[capability];
              return TUCAMRET_SUCCESS;
            }));
    EXPECT_CALL(*mockSDK, getROI(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));

    // Acquire sequence occurs after bin mode change has completed
    EXPECT_CALL(*mockSDK, startCapture(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillRepeatedly(Invoke(
            [](void* deviceHandle, TUCAM_FRAME* frameHandle, double timeout) {
              frameHandle->usWidth = 100;
              frameHandle->usHeight = 100;
              frameHandle->ucFormat = TUFRM_FMT_RAW;
              frameHandle->ucChannels = 1;
              frameHandle->ucElemBytes = 1;
              frameHandle->uiImgSize = 100 * 100;
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
              return TUCAMRET_SUCCESS;
            }));
    EXPECT_CALL(*mockSDK, stopCapture(_)).WillOnce(Return(TUCAMRET_SUCCESS));

    // Destructor: Final releaseBuffer call
    EXPECT_CALL(*mockSDK, releaseBuffer(_)).WillOnce(Return(TUCAMRET_SUCCESS));
  }
  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);

  // Allow for the driver background threads to start
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Set bin mode to 0 in one thread
  std::thread t1(
      [this, &driver]() { writeIntParam(&driver, ADTucam_BinModeString, 1); });

  // Start acquisition immediately in another thread
  std::thread t2([this, &driver, &binModeStartedFuture]() {
    binModeStartedFuture.wait();
    writeIntParam(&driver, ADAcquireString, 1);
  });

  // Allow the acquisition thread to hit the lock()
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  acquireReadyPromise.set_value();

  t1.join();
  t2.join();

  // Allow the acquisition thread to acquire some frames
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  int value;
  EXPECT_EQ(readIntParam(&driver, ADTucam_BinModeString, &value), asynSuccess);
  EXPECT_EQ(value, 1);
}

/**
 * Test that we cannot set the ROI while acquiring.
 */
TEST_F(ADTucamTest, SetROIWhileAcquiring) {
  // Miscellaneous background operations that will always succeed
  mockConnectionLifecycle();
  EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, releaseBuffer(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Invoke([](void* deviceHandle, TUCAM_VALUE_INFO* valInfo) {
        // Return sensible dimensions for ADMaxSizeX/ADMaxSizeY
        valInfo->nValue = 1024;
        return TUCAMRET_SUCCESS;
      }));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // Called while acquiring
  EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
      .WillRepeatedly(Invoke(
          [](void* deviceHandle, TUCAM_FRAME* frameHandle, double timeout) {
            frameHandle->usWidth = 100;
            frameHandle->usHeight = 100;
            frameHandle->ucFormat = TUFRM_FMT_RAW;
            frameHandle->ucChannels = 1;
            frameHandle->ucElemBytes = 1;
            frameHandle->uiImgSize = 100 * 100;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            return TUCAMRET_SUCCESS;
          }));
  EXPECT_CALL(*mockSDK, abortWait(_)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, startCapture(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, stopCapture(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // Initialize ROI to 1,2,3,4
  EXPECT_CALL(*mockSDK, setROI(_, _))
      .Times(4)
      .WillRepeatedly(Invoke([this](void* deviceHandle, TUCAM_ROI_ATTR value) {
        this->roiCache_ = value;
        return TUCAMRET_SUCCESS;
      }));
  // We get the current bin mode as the readback value
  EXPECT_CALL(*mockSDK, getROI(_, _))
      // Once during init, then 4 times during ROI change below
      .Times(5)
      .WillRepeatedly(Invoke([this](void* deviceHandle, TUCAM_ROI_ATTR* value) {
        *value = this->roiCache_;
        return TUCAMRET_SUCCESS;
      }));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  EXPECT_EQ(writeIntParam(&driver, ADMinXString, 1), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADMinYString, 2), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADSizeXString, 3), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADSizeYString, 4), asynSuccess);
  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);

  int value;
  EXPECT_EQ(writeIntParam(&driver, ADMinXString, 50), asynError);
  EXPECT_EQ(readIntParam(&driver, ADMinXString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  EXPECT_EQ(writeIntParam(&driver, ADMinYString, 51), asynError);
  EXPECT_EQ(readIntParam(&driver, ADMinYString, &value), asynSuccess);
  EXPECT_EQ(value, 2);

  EXPECT_EQ(writeIntParam(&driver, ADSizeXString, 52), asynError);
  EXPECT_EQ(readIntParam(&driver, ADSizeXString, &value), asynSuccess);
  EXPECT_EQ(value, 3);

  EXPECT_EQ(writeIntParam(&driver, ADSizeYString, 53), asynError);
  EXPECT_EQ(readIntParam(&driver, ADSizeYString, &value), asynSuccess);
  EXPECT_EQ(value, 4);
}

/**
 * Test that we can change the ROI while acquisition is idle.
 */
TEST_F(ADTucamTest, ROIControl) {
  // Miscellaneous background operations that will always succeed
  mockConnectionLifecycle();
  EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, releaseBuffer(_))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Invoke([](void* deviceHandle, TUCAM_VALUE_INFO* valInfo) {
        // Return sensible dimensions for ADMaxSizeX/ADMaxSizeY
        valInfo->nValue = 1024;
        return TUCAMRET_SUCCESS;
      }));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // Proper sequence based on actual call order
  {
    InSequence seq;

    // Constructor: Initial calls
    EXPECT_CALL(*mockSDK, getROI(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));

    // ROI change sequence
    EXPECT_CALL(*mockSDK, releaseBuffer(_)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, setROI(_, _))
        .WillOnce(Invoke([this](void* deviceHandle, TUCAM_ROI_ATTR value) {
          this->roiCache_ = value;
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, getROI(_, _))
        .WillOnce(Invoke([this](void* deviceHandle, TUCAM_ROI_ATTR* value) {
          *value = this->roiCache_;
          return TUCAMRET_SUCCESS;
        }));

    // Destructor: Final releaseBuffer call
    EXPECT_CALL(*mockSDK, releaseBuffer(_)).WillOnce(Return(TUCAMRET_SUCCESS));
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADMinXString, 50), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADMinXString, &value), asynSuccess);
  EXPECT_EQ(value, 50);
}

/**
 * Test that we cannot start acquisition until the ROI change has completed.
 *
 * NOTE: Uses futures and promises to ensure proper sequencing of calls.
 */
TEST_F(ADTucamTest, WaitForROIChangeBeforeAcquiring) {
  // Miscellaneous background operations that will always succeed
  mockConnectionLifecycle();
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Invoke([](void* deviceHandle, TUCAM_VALUE_INFO* valInfo) {
        // Return sensible dimensions for ADMaxSizeX/ADMaxSizeY
        valInfo->nValue = 1024;
        return TUCAMRET_SUCCESS;
      }));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getPropertyValue(_, _, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, abortWait(_)).WillRepeatedly(Return(TUCAMRET_SUCCESS));

  // Setup multithreaded promises & futures
  std::promise<void> roiChangeStartedPromise;
  std::future<void> roiChangeStartedFuture =
      roiChangeStartedPromise.get_future();
  std::promise<void> acquireReadyPromise;
  std::future<void> acquireReadyFuture = acquireReadyPromise.get_future();

  // Proper sequence based on actual call order
  {
    InSequence seq;

    // Constructor: Initial calls
    EXPECT_CALL(*mockSDK, getROI(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));

    // ROI change sequence
    EXPECT_CALL(*mockSDK, releaseBuffer(_))
        .WillOnce(Invoke([&roiChangeStartedPromise,
                          &acquireReadyFuture](void* deviceHandle) {
          // Signal that the ROI change has started
          roiChangeStartedPromise.set_value();
          // Wait for acquisition to start
          acquireReadyFuture.wait();
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, setROI(_, _))
        .WillOnce(Invoke([this](void* deviceHandle, TUCAM_ROI_ATTR value) {
          this->roiCache_ = value;
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, getROI(_, _))
        .WillOnce(Invoke([this](void* deviceHandle, TUCAM_ROI_ATTR* value) {
          *value = this->roiCache_;
          return TUCAMRET_SUCCESS;
        }));

    // Acquire sequence occurs after bin mode change has completed
    EXPECT_CALL(*mockSDK, startCapture(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillRepeatedly(Invoke(
            [](void* deviceHandle, TUCAM_FRAME* frameHandle, double timeout) {
              frameHandle->usWidth = 100;
              frameHandle->usHeight = 100;
              frameHandle->ucFormat = TUFRM_FMT_RAW;
              frameHandle->ucChannels = 1;
              frameHandle->ucElemBytes = 1;
              frameHandle->uiImgSize = 100 * 100;
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
              return TUCAMRET_SUCCESS;
            }));
    EXPECT_CALL(*mockSDK, stopCapture(_)).WillOnce(Return(TUCAMRET_SUCCESS));

    // Destructor: Final releaseBuffer call
    EXPECT_CALL(*mockSDK, releaseBuffer(_)).WillOnce(Return(TUCAMRET_SUCCESS));
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);

  // Allow for the driver background threads to start
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Set bin mode to 0 in one thread
  std::thread t1(
      [this, &driver]() { writeIntParam(&driver, ADMinXString, 50); });

  // Start acquisition immediately in another thread
  std::thread t2([this, &driver, &roiChangeStartedFuture]() {
    roiChangeStartedFuture.wait();
    writeIntParam(&driver, ADAcquireString, 1);
  });

  // Allow the acquisition thread to hit the lock()
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  acquireReadyPromise.set_value();

  t1.join();
  t2.join();

  // Allow the acquisition thread to acquire some frames
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  int value;
  EXPECT_EQ(readIntParam(&driver, ADMinXString, &value), asynSuccess);
  EXPECT_EQ(value, 50);
}
