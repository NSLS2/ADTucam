/**
 * test_connection.cpp
 *
 * Tests for camera connection and teardown.
 *
 * Author: Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */
#include <string>

#include "ADTucamTest.h"

using ::testing::_;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Return;

/*
 * Test successful camera connection and teardown.
 */
TEST_F(ADTucamTest, SuccessfulConnectionLifecycle) {
  // Critical lifecycle calls that MUST be in sequence
  {
    InSequence seq;
    EXPECT_CALL(*mockSDK, initializeAPI(_))
        .WillOnce(Invoke([](TUCAM_INIT* apiHandle) {
          apiHandle->uiCamCount = 1;
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, openDevice(_))
        .WillOnce(Invoke([](TUCAM_OPEN* camHandle) {
          camHandle->hIdxTUCam = reinterpret_cast<HDTUCAM>(0x12345678);
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_SUCCESS));
    // ... constructor completes, background threads start ...
    // Teardown (destructor) - must happen after construction
    EXPECT_CALL(*mockSDK, releaseBuffer(_)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, closeDevice(_)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, uninitializeAPI()).WillOnce(Return(TUCAMRET_SUCCESS));
  }

  // These calls can happen anytime during initialization or background
  // operations and are not a critical part of a successful connection lifecycle
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

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
}

/**
 * Test failed camera initialization
 */
TEST_F(ADTucamTest, FailedInitializeAPI) {
  EXPECT_CALL(*mockSDK, initializeAPI(_)).WillOnce(Return(TUCAMRET_FAILURE));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  int status;
  EXPECT_EQ(readIntParam(&driver, ADStatusString, &status), asynSuccess);
  EXPECT_EQ(status, ADStatusDisconnected);
  char statusMessage[1024];
  EXPECT_EQ(
      readStringParam(&driver, ADStatusMessageString, statusMessage, 1024),
      asynSuccess);
  EXPECT_EQ(strcmp(statusMessage, "Camera connection failed"), 0);
}

/**
 * Test failed camera open
 */
TEST_F(ADTucamTest, FailedOpenDevice) {
  {
    InSequence seq;
    EXPECT_CALL(*mockSDK, initializeAPI(_))
        .WillOnce(Invoke([](TUCAM_INIT* apiHandle) {
          apiHandle->uiCamCount = 1;
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, openDevice(_)).WillOnce(Return(TUCAMRET_FAILURE));
    EXPECT_CALL(*mockSDK, uninitializeAPI()).WillOnce(Return(TUCAMRET_SUCCESS));
  }

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  int status;
  EXPECT_EQ(readIntParam(&driver, ADStatusString, &status), asynSuccess);
  EXPECT_EQ(status, ADStatusDisconnected);
  char statusMessage[1024];
  EXPECT_EQ(
      readStringParam(&driver, ADStatusMessageString, statusMessage, 1024),
      asynSuccess);
  EXPECT_EQ(strcmp(statusMessage, "Camera connection failed"), 0);
}

TEST_F(ADTucamTest, FailedAllocateBuffer) {
  {
    InSequence seq;
    EXPECT_CALL(*mockSDK, initializeAPI(_))
        .WillOnce(Invoke([](TUCAM_INIT* apiHandle) {
          apiHandle->uiCamCount = 1;
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, openDevice(_)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, allocateBuffer(_, _))
        .WillOnce(Return(TUCAMRET_FAILURE));
    EXPECT_CALL(*mockSDK, closeDevice(_)).WillOnce(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, uninitializeAPI()).WillOnce(Return(TUCAMRET_SUCCESS));
  }
  // These calls can happen anytime during initialization or background
  // operations
  EXPECT_CALL(*mockSDK, getDeviceInfo(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, readRegister(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getROI(_, _)).WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTrigger(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  EXPECT_CALL(*mockSDK, getTriggerOut(_, _))
      .WillRepeatedly(Return(TUCAMRET_SUCCESS));

  std::string portName = getUniquePortName();
  ADTucam driver = ADTucam(portName.c_str(), 0, mockSDK);
  int status;
  EXPECT_EQ(readIntParam(&driver, ADStatusString, &status), asynSuccess);
  EXPECT_EQ(status, ADStatusDisconnected);
  char statusMessage[1024];
  EXPECT_EQ(
      readStringParam(&driver, ADStatusMessageString, statusMessage, 1024),
      asynSuccess);
  EXPECT_EQ(strcmp(statusMessage, "Camera connection failed"), 0);
}
