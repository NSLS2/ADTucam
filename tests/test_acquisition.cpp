/**
 * test_acquisition.cpp
 *
 * Tests for acquisition.
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

TEST_F(ADTucamTest, ContinuousAcquisitionCycle) {
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

  // Acquisition cycle
  {
    InSequence seq;

    EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_SEQUENCE))
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
  }

  EXPECT_CALL(*mockSDK, abortWait(_)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Free run trigger mode
  writeIntParam(&driver, ADTriggerModeString, 0);
  writeIntParam(&driver, ADImageModeString, ADImageContinuous);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for a few frames to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 0), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  // Wait for stop capture to complete
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST_F(ADTucamTest, ConsecutiveStartsAndStops) {
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

  // Called during acquisition cycle
  EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_SEQUENCE))
      .Times(5)
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
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
  EXPECT_CALL(*mockSDK, stopCapture(_))
      .Times(5)
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, abortWait(_)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Free run trigger mode
  writeIntParam(&driver, ADTriggerModeString, 0);
  writeIntParam(&driver, ADImageModeString, ADImageContinuous);
  int value;

  for (int i = 0; i < 5; i++) {
    EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
    EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
    EXPECT_EQ(value, 1);

    // Wait for a few frames to process
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 0), asynSuccess);
    EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
    EXPECT_EQ(value, 0);

    // Wait for stop capture to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

/**
 * Test that starting acquire > 1 time does not interfere with the first
 * acquisition cycle.
 */
TEST_F(ADTucamTest, RepeatedAcquiresDoNotInterfere) {
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

  // Acquisition cycle
  {
    InSequence seq;

    EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_SEQUENCE))
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
  }

  EXPECT_CALL(*mockSDK, abortWait(_)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Free run trigger mode
  writeIntParam(&driver, ADTriggerModeString, 0);
  writeIntParam(&driver, ADImageModeString, ADImageContinuous);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynError);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for a few frames to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynError);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynError);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for a few frames to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 0), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  // Wait for stop capture to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/**
 * Test that repeated calls to ADAcquire=0 do not queue up stop capture calls.
 */
TEST_F(ADTucamTest, RepeatedStopsDoNotQueue) {
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
  EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));

  // Acquisition cycle
  EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_SEQUENCE))
      .Times(2)
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
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
  EXPECT_CALL(*mockSDK, stopCapture(_))
      .Times(2)
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, abortWait(_)).WillRepeatedly(Return(TUCAMRET_SUCCESS));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Free run trigger mode
  writeIntParam(&driver, ADTriggerModeString, 0);
  writeIntParam(&driver, ADImageModeString, ADImageContinuous);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for a few images to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 0), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 0), asynError);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 0), asynError);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  // Wait for stop capture to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for a few images to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Ensure that a few images were acquired
  EXPECT_EQ(readIntParam(&driver, ADNumImagesCounterString, &value),
            asynSuccess);
  EXPECT_GT(value, 5);

  // Stop acquisition
  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 0), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  // Wait for stop capture to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST_F(ADTucamTest, MultipleAcquisitionCycle) {
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
  EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, abortWait(_)).WillOnce(Return(TUCAMRET_SUCCESS));

  // Acquisition cycle
  {
    InSequence seq;

    EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_SEQUENCE))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .Times(5)
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
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Free run trigger mode
  writeIntParam(&driver, ADTriggerModeString, 0);
  writeIntParam(&driver, ADImageModeString, ADImageMultiple);
  writeIntParam(&driver, ADNumImagesString, 5);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for acquisition to start
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  EXPECT_EQ(readIntParam(&driver, ADStatusString, &value), asynSuccess);
  EXPECT_EQ(value, ADStatusAcquire);

  // Wait for all 5 frames to be acquired
  std::this_thread::sleep_for(std::chrono::milliseconds(90));

  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(readIntParam(&driver, ADNumImagesCounterString, &value),
            asynSuccess);
  EXPECT_EQ(value, 5);

  EXPECT_EQ(readIntParam(&driver, ADStatusString, &value), asynSuccess);
  EXPECT_EQ(value, ADStatusIdle);

  EXPECT_EQ(readIntParam(&driver, NDArrayCounterString, &value), asynSuccess);
  EXPECT_EQ(value, 5);
}

TEST_F(ADTucamTest, SingleAcquisitionCycle) {
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
  EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, abortWait(_)).WillOnce(Return(TUCAMRET_SUCCESS));

  // Acquisition cycle
  {
    InSequence seq;

    EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_SEQUENCE))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Free run trigger mode
  writeIntParam(&driver, ADTriggerModeString, 0);
  writeIntParam(&driver, ADImageModeString, ADImageSingle);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for acquisition to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(readIntParam(&driver, ADNumImagesCounterString, &value),
            asynSuccess);
  EXPECT_EQ(value, 1);

  EXPECT_EQ(readIntParam(&driver, ADStatusString, &value), asynSuccess);
  EXPECT_EQ(value, ADStatusIdle);

  EXPECT_EQ(readIntParam(&driver, NDArrayCounterString, &value), asynSuccess);
  EXPECT_EQ(value, 1);
}

TEST_F(ADTucamTest, SingleSoftwareTriggerAcquisitionCycle) {
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

  // Only called once as part of the destructor
  EXPECT_CALL(*mockSDK, abortWait(_)).WillOnce(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, stopCapture(_)).WillOnce(Return(TUCAMRET_SUCCESS));

  // Acquisition cycle
  {
    InSequence seq;

    EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_TRIGGER_SOFTWARE))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Software trigger mode
  writeIntParam(&driver, ADTriggerModeString, TUCCM_TRIGGER_SOFTWARE);
  writeIntParam(&driver, ADImageModeString, ADImageSingle);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for acquisition to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(readIntParam(&driver, ADNumImagesCounterString, &value),
            asynSuccess);
  EXPECT_EQ(value, 1);
}

TEST_F(ADTucamTest, MultipleSoftwareTriggerAcquisitionCycle) {
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

  // Only called once as part of the destructor
  EXPECT_CALL(*mockSDK, abortWait(_)).WillOnce(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, stopCapture(_)).WillOnce(Return(TUCAMRET_SUCCESS));

  // Acquisition cycle
  {
    InSequence seq;

    EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_TRIGGER_SOFTWARE))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Software trigger mode
  writeIntParam(&driver, ADTriggerModeString, TUCCM_TRIGGER_SOFTWARE);
  writeIntParam(&driver, ADImageModeString, ADImageMultiple);
  writeIntParam(&driver, ADNumImagesString, 5);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for acquisition to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(readIntParam(&driver, ADNumImagesCounterString, &value),
            asynSuccess);
  EXPECT_EQ(value, 5);
}

TEST_F(ADTucamTest, MultipleAcquisitionCyclesSoftwareTrigger) {
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

  // Only called once as part of the destructor
  EXPECT_CALL(*mockSDK, abortWait(_)).WillOnce(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, stopCapture(_)).WillOnce(Return(TUCAMRET_SUCCESS));

  // Acquisition cycle
  {
    InSequence seq;

    EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_TRIGGER_SOFTWARE))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
    EXPECT_CALL(*mockSDK, doSoftwareTrigger(_))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke(
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
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Software trigger mode
  writeIntParam(&driver, ADTriggerModeString, TUCCM_TRIGGER_SOFTWARE);
  writeIntParam(&driver, ADImageModeString, ADImageMultiple);
  writeIntParam(&driver, ADNumImagesString, 2);
  int value;

  // Explicitly start capture
  EXPECT_EQ(writeIntParam(&driver, ADTucam_CaptureString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADTucam_CaptureString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Frame 1 & 2
  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(readIntParam(&driver, ADNumImagesCounterString, &value),
            asynSuccess);
  EXPECT_EQ(value, 2);

  // Frame 3 & 4
  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(readIntParam(&driver, ADNumImagesCounterString, &value),
            asynSuccess);
  EXPECT_EQ(value, 2);

  // Frame 5 & 6
  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(readIntParam(&driver, ADNumImagesCounterString, &value),
            asynSuccess);
  EXPECT_EQ(value, 2);

  // Total should be 6
  EXPECT_EQ(readIntParam(&driver, NDArrayCounterString, &value), asynSuccess);
  EXPECT_EQ(value, 6);

  // Explicitly stop capture
  EXPECT_EQ(writeIntParam(&driver, ADTucam_CaptureString, 0), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADTucam_CaptureString, &value), asynSuccess);
  EXPECT_EQ(value, 0);
}

TEST_F(ADTucamTest, AcquisitionTimeout) {
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
  EXPECT_CALL(*mockSDK, setTrigger(_, _)).WillOnce(Return(TUCAMRET_SUCCESS));

  // Set up thread communication
  std::promise<void> abortWaitPromise;
  std::future<void> abortWaitFuture = abortWaitPromise.get_future();

  // Acquisition cycle
  {
    InSequence seq;

    EXPECT_CALL(*mockSDK, startCapture(_, TUCCM_SEQUENCE))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    // Wait for abortWait to be called to simulate a timeout from the timeout
    // thread
    EXPECT_CALL(*mockSDK, waitForFrame(_, _, _))
        .WillOnce(Invoke([&abortWaitFuture](void* deviceHandle,
                                            TUCAM_FRAME* frameHandle,
                                            double timeout) {
          abortWaitFuture.wait();
          return TUCAMRET_ABORT;
        }));
    EXPECT_CALL(*mockSDK, abortWait(_))
        .WillOnce(Invoke([&abortWaitPromise](void* deviceHandle) {
          abortWaitPromise.set_value();
          return TUCAMRET_SUCCESS;
        }))
        // Called an extra time as part of the destructor
        .WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, stopCapture(_)).WillOnce(Return(TUCAMRET_SUCCESS));
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  // Free run trigger mode
  writeIntParam(&driver, ADTriggerModeString, 0);
  writeIntParam(&driver, ADImageModeString, ADImageSingle);
  int value;

  EXPECT_EQ(writeIntParam(&driver, ADAcquireString, 1), asynSuccess);
  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 1);

  // Wait for acquisition to be aborted (minimum of 30 seconds)
  std::this_thread::sleep_for(std::chrono::milliseconds(31000));

  EXPECT_EQ(readIntParam(&driver, ADAcquireString, &value), asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(readIntParam(&driver, ADNumImagesCounterString, &value),
            asynSuccess);
  EXPECT_EQ(value, 0);

  EXPECT_EQ(readIntParam(&driver, ADStatusString, &value), asynSuccess);
  EXPECT_EQ(value, ADStatusError);

  EXPECT_EQ(readIntParam(&driver, NDArrayCounterString, &value), asynSuccess);
  EXPECT_EQ(value, 0);
}
