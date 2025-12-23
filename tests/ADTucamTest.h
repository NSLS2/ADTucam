/**
 * ADTucamTestBase.h
 *
 * Base class for ADTucam tests.
 *
 * Author: Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */

#ifndef TESTS_ADTUCAMTEST_H_
#define TESTS_ADTUCAMTEST_H_

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <map>
#include <string>

#include "ADTucam.h"
#include "MockCameraSDK.h"

using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::SetArgPointee;
using ::testing::StrictMock;

class ADTucamTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mockSDK = new StrictMock<MockCameraSDK>();
    propertyCache_ = std::map<int, double>();
    capabilityCache_ = std::map<int, int>();
    roiCache_ = TUCAM_ROI_ATTR();
  }

  void TearDown() override {
    delete mockSDK;
    propertyCache_.clear();
    capabilityCache_.clear();
  }

  /**
   * Generate a unique port name for each test.
   * Uses a static counter to ensure uniqueness across all tests.
   */
  std::string getUniquePortName() {
    static int portCounter = 0;
    return "TEST_PORT_" + std::to_string(++portCounter);
  }

  /**
   * Mock the connection setup for the driver.
   */
  void mockConnectionSetup() {
    EXPECT_CALL(*mockSDK, initializeAPI(_))
        .WillRepeatedly(Invoke([](TUCAM_INIT* apiHandle) {
          apiHandle->uiCamCount = 1;
          return TUCAMRET_SUCCESS;
        }));
    EXPECT_CALL(*mockSDK, openDevice(_))
        .WillRepeatedly(Invoke([](TUCAM_OPEN* camHandle) {
          camHandle->hIdxTUCam = reinterpret_cast<HDTUCAM>(0x12345678);
          return TUCAMRET_SUCCESS;
        }));
  }

  /**
   * Mock the connection teardown for the driver.
   */
  void mockConnectionTeardown() {
    EXPECT_CALL(*mockSDK, closeDevice(_))
        .WillRepeatedly(Return(TUCAMRET_SUCCESS));
    EXPECT_CALL(*mockSDK, uninitializeAPI())
        .WillRepeatedly(Return(TUCAMRET_SUCCESS));
  }

  /**
   * Mock the full connection lifecycle for the driver.
   */
  void mockConnectionLifecycle() {
    mockConnectionSetup();
    mockConnectionTeardown();
  }

  // =========================================================================
  // HELPER METHODS - Simulate Channel Access → asyn → driver interface
  // =========================================================================

  /**
   * Simulate writing an integer parameter via Channel Access.
   * This replicates the path: PV write → record → asyn → writeInt32()
   */
  asynStatus writeIntParam(ADTucam* driver, const char* paramName, int value) {
    int index;
    int status = driver->findParam(paramName, &index);
    if (status != asynSuccess) {
      ADD_FAILURE() << "Parameter not found: " << paramName;
      return asynError;
    }

    // Create minimal asynUser - only reason field needed for driver logic
    asynUser user = {};   // Zero-initialize all fields
    user.reason = index;  // This is what the driver checks to determine action

    // asyn calls writeInt32 after locking the driver
    driver->lock();
    status |= driver->writeInt32(&user, value);
    driver->unlock();

    return (asynStatus)status;
  }

  /**
   * Simulate writing a float parameter via Channel Access.
   * This replicates the path: PV write → record → asyn → writeFloat64()
   */
  asynStatus writeFloatParam(ADTucam* driver, const char* paramName,
                             double value) {
    int index;
    int status = driver->findParam(paramName, &index);
    if (status != asynSuccess) {
      ADD_FAILURE() << "Parameter not found: " << paramName;
      return asynError;
    }

    // Create minimal asynUser - only reason field needed for driver logic
    asynUser user = {};   // Zero-initialize all fields
    user.reason = index;  // This is what the driver checks to determine action

    // asyn calls writeFloat64 after locking the driver
    driver->lock();
    status |= driver->writeFloat64(&user, value);
    driver->unlock();

    return (asynStatus)status;
  }

  /**
   * Read back an integer parameter value for verification.
   * Uses the parameter library directly (public asynPortDriver method).
   */
  asynStatus readIntParam(ADTucam* driver, const char* paramName, int* value) {
    int index;
    int status = driver->findParam(paramName, &index);
    if (status != asynSuccess) {
      ADD_FAILURE() << "Parameter not found: " << paramName;
      return asynError;
    }
    status |= driver->getIntegerParam(index, value);
    return (asynStatus)status;
  }

  /**
   * Read back a float parameter value for verification.
   * Uses the parameter library directly (public asynPortDriver method).
   */
  asynStatus readFloatParam(ADTucam* driver, const char* paramName,
                            double* value) {
    int index;
    int status = driver->findParam(paramName, &index);
    if (status != asynSuccess) {
      ADD_FAILURE() << "Parameter not found: " << paramName;
      return asynError;
    }
    status |= driver->getDoubleParam(index, value);
    return (asynStatus)status;
  }

  /**
   * Read back a string parameter value for verification.
   * Uses the parameter library directly (public asynPortDriver method).
   */
  asynStatus readStringParam(ADTucam* driver, const char* paramName, char* value,
                             int maxChars) {
    int index;
    int status = driver->findParam(paramName, &index);
    if (status != asynSuccess) {
      ADD_FAILURE() << "Parameter not found: " << paramName;
      return asynError;
    }
    status |= driver->getStringParam(index, maxChars, value);
    return (asynStatus)status;
  }

  StrictMock<MockCameraSDK>* mockSDK;

  // Property and capability cache for maintaining state during tests
  std::map<int, double> propertyCache_;
  std::map<int, int> capabilityCache_;
  TUCAM_ROI_ATTR roiCache_;
};

#endif  // TESTS_ADTUCAMTEST_H_
