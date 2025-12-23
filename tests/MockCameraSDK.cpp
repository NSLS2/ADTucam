/**
 * MockCameraSDK.cpp
 *
 * Implementation of helper methods for MockCameraSDK.
 *
 * Author: Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */

#include "MockCameraSDK.h"

#include <cstring>

#include "TUDefine.h"

using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::SetArgPointee;
