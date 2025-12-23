/**
 * test_ADTucam.cpp
 *
 * Unit tests for ADTucam using MockCameraSDK.
 *
 * Author: Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>

// Main function to run tests
int main(int argc, char** argv) {
  // Initialize Google Test/Mock
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::InitGoogleMock(&argc, argv);

  return RUN_ALL_TESTS();
}
