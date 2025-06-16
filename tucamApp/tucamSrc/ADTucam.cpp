/**
 * ADTucam.cpp
 *
 * Main source file for the ADTucam EPICS areaDetector driver.
 *
 * Author(s): Jakub Wlodek, Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */
#include "ADTucam.h"

#include <algorithm>
#include <chrono>

// TUCAM include
#include "TUCamApi.h"

#define DRIVER_VERSION 1
#define DRIVER_REVISION 0
#define DRIVER_MODIFICATION 0

// Error message formatters
#define ERR(msg)                                                         \
  asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: %s\n", driverName, \
            functionName, msg)

#define ERR_ARGS(fmt, ...)                                                    \
  asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: " fmt "\n", driverName, \
            functionName, __VA_ARGS__);

// Warning message formatters
#define WARN(msg)                                                          \
  asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s::%s: %s\n", driverName, \
            functionName, msg)

#define WARN_ARGS(fmt, ...)                                         \
  asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s::%s: " fmt "\n", \
            driverName, functionName, __VA_ARGS__);

// Info message formatters
#define INFO(msg)                                                       \
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: %s\n", driverName, \
            functionName, msg)

#define INFO_ARGS(fmt, ...)                                                  \
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: " fmt "\n", driverName, \
            functionName, __VA_ARGS__);

// Debug message formatters
#define DEBUG(msg)                                                      \
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: %s\n", driverName, \
            functionName, msg)

#define DEBUG_ARGS(fmt, ...)                                                 \
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: " fmt "\n", driverName, \
            functionName, __VA_ARGS__);

/**
 * @brief Function that instatiates a driver object. Called from IOC shell
 *
 */
extern "C" int ADTucamConfig(const char *portName, int cameraId) {
  new ADTucam(portName, cameraId);
  return (asynSuccess);
}

/**
 * @brief Wrapper C function passed to epicsThreadCreate to create temperature
 * monitor thread
 * @param drvPvt Pointer to instance of ADTucam driver object
 */
static void monitorTemperatureThreadC(void *drvPvt) {
  ADTucam *pPvt = (ADTucam *)drvPvt;  // NOLINT(readability/casting)
  pPvt->monitorTemperatureThread();
}

/**
 * @brief Wrapper C function passed to epicsThreadCreate to create acquisition
 * thread
 * @param drvPvt Pointer to instance of ADTucam driver object
 */
static void acquisitionThreadC(void *drvPvt) {
  ADTucam *pPvt = (ADTucam *)drvPvt;  // NOLINT(readability/casting)
  pPvt->acquisitionThread();
}

/**
 * @brief Callback function called when exit is ran from IOC shell, deletes
 * driver object instance
 * @param drvPvt Pointer to instance of ADTucam driver object
 */
static void exitCallbackC(void *drvPvt) {
  ADTucam *pPvt = (ADTucam *)drvPvt;  // NOLINT(readability/casting)
  delete pPvt;
}

/**
 * Formats currently supported by the camera SDK.
 */
static const int frameFormats[3] = {TUFRM_FMT_RAW, TUFRM_FMT_USUAl,
                                    TUFRM_FMT_RGB888};

static const char *driverName = "ADTucam";
static int TUCAMInitialized = 0;

/**
 * @brief Main constructor for ADTucam driver
 *
 * @param portName Unique asyn port name
 * @param cameraId ID of the camera used to connect
 */
ADTucam::ADTucam(const char *portName, int cameraId)
    : ADDriver(portName, 1, NUM_TUCAM_PARAMS, 0, 0, 0, 0, 0, 1, 0, 0),
      cameraId_(cameraId),
      exiting_(0),
      pRaw_(NULL),
      triggerOutSupport_(0) {
  const char *functionName = "ADTucam";

  char versionString[20];
  asynStatus status = asynSuccess;

  // Initialize new parameters in parameter library
  createParam(ADTucam_TemperatureSetpointString, asynParamFloat64,
              &ADTucam_TemperatureSetpoint);
  createParam(ADTucam_TemperatureString, asynParamFloat64, &ADTucam_Temperature);
  createParam(ADTucam_TemperatureEmergencySignalString, asynParamInt32,
              &ADTucam_TemperatureEmergencySignal);
  createParam(ADTucam_AutoTECThresholdString, asynParamFloat64,
              &ADTucam_AutoTECThreshold);
  createParam(ADTucam_AutoTECString, asynParamInt32, &ADTucam_AutoTEC);
  createParam(ADTucam_TECStatusString, asynParamInt32, &ADTucam_TECStatus);
  createParam(ADTucam_GainModeString, asynParamInt32, &ADTucam_GainMode);
  createParam(ADTucam_RetryOnTimeoutString, asynParamInt32,
              &ADTucam_RetryOnTimeout);
  createParam(ADTucam_NumRetriesString, asynParamInt32, &ADTucam_NumRetries);
  createParam(ADTucam_BinModeString, asynParamInt32, &ADTucam_BinMode);
  createParam(ADTucam_BusString, asynParamOctet, &ADTucam_Bus);
  createParam(ADTucam_ProductIDString, asynParamFloat64, &ADTucam_ProductID);
  createParam(ADTucam_TransferRateString, asynParamFloat64,
              &ADTucam_TransferRate);
  createParam(ADTucam_FrameSpeedString, asynParamInt32, &ADTucam_FrameSpeed);
  createParam(ADTucam_BitDepthString, asynParamInt32, &ADTucam_BitDepth);
  createParam(ADTucam_FanGearString, asynParamInt32, &ADTucam_FanGear);
  createParam(ADTucam_ImageModeString, asynParamInt32, &ADTucam_ImageMode);
  createParam(ADTucam_AutoExposureString, asynParamInt32, &ADTucam_AutoExposure);
  createParam(ADTucam_AutoLevelsString, asynParamInt32, &ADTucam_AutoLevels);
  createParam(ADTucam_HistogramString, asynParamInt32, &ADTucam_Histogram);
  createParam(ADTucam_EnhanceString, asynParamInt32, &ADTucam_Enhance);
  createParam(ADTucam_DefectCorrString, asynParamInt32, &ADTucam_DefectCorr);
  createParam(ADTucam_DenoiseString, asynParamInt32, &ADTucam_Denoise);
  createParam(ADTucam_FlatCorrString, asynParamInt32, &ADTucam_FlatCorr);
  createParam(ADTucam_DynRgeCorrString, asynParamInt32, &ADTucam_DynRgeCorr);
  createParam(ADTucam_FrameFormatString, asynParamInt32, &ADTucam_FrameFormat);
  createParam(ADTucam_BrightnessString, asynParamFloat64, &ADTucam_Brightness);
  createParam(ADTucam_BlackLevelString, asynParamFloat64, &ADTucam_BlackLevel);
  createParam(ADTucam_SharpnessString, asynParamFloat64, &ADTucam_Sharpness);
  createParam(ADTucam_NoiseLevelString, asynParamFloat64, &ADTucam_NoiseLevel);
  createParam(ADTucam_HDRKString, asynParamFloat64, &ADTucam_HDRK);
  createParam(ADTucam_GammaString, asynParamFloat64, &ADTucam_Gamma);
  createParam(ADTucam_ContrastString, asynParamFloat64, &ADTucam_Contrast);
  createParam(ADTucam_LeftLevelString, asynParamFloat64, &ADTucam_LeftLevel);
  createParam(ADTucam_RightLevelString, asynParamFloat64, &ADTucam_RightLevel);
  createParam(ADTucam_TriggerEdgeString, asynParamInt32, &ADTucam_TriggerEdge);
  createParam(ADTucam_TriggerExposureString, asynParamInt32,
              &ADTucam_TriggerExposure);
  createParam(ADTucam_TriggerDelayString, asynParamFloat64,
              &ADTucam_TriggerDelay);
  createParam(ADTucam_TriggerSoftwareString, asynParamInt32,
              &ADTucam_TriggerSoftware);
  createParam(ADTucam_TriggerOut1ModeString, asynParamInt32,
              &ADTucam_TriggerOut1Mode);
  createParam(ADTucam_TriggerOut1EdgeString, asynParamInt32,
              &ADTucam_TriggerOut1Edge);
  createParam(ADTucam_TriggerOut1DelayString, asynParamFloat64,
              &ADTucam_TriggerOut1Delay);
  createParam(ADTucam_TriggerOut1WidthString, asynParamFloat64,
              &ADTucam_TriggerOut1Width);
  createParam(ADTucam_TriggerOut2ModeString, asynParamInt32,
              &ADTucam_TriggerOut2Mode);
  createParam(ADTucam_TriggerOut2EdgeString, asynParamInt32,
              &ADTucam_TriggerOut2Edge);
  createParam(ADTucam_TriggerOut2DelayString, asynParamFloat64,
              &ADTucam_TriggerOut2Delay);
  createParam(ADTucam_TriggerOut2WidthString, asynParamFloat64,
              &ADTucam_TriggerOut2Width);
  createParam(ADTucam_TriggerOut3ModeString, asynParamInt32,
              &ADTucam_TriggerOut3Mode);
  createParam(ADTucam_TriggerOut3EdgeString, asynParamInt32,
              &ADTucam_TriggerOut3Edge);
  createParam(ADTucam_TriggerOut3DelayString, asynParamFloat64,
              &ADTucam_TriggerOut3Delay);
  createParam(ADTucam_TriggerOut3WidthString, asynParamFloat64,
              &ADTucam_TriggerOut3Width);

  /* Set initial values for some parameters */
  setIntegerParam(NDDataType, NDUInt16);
  setIntegerParam(NDColorMode, NDColorModeMono);
  setIntegerParam(NDArraySizeZ, 0);
  setStringParam(ADStringToServer, "<not used by driver>");
  setStringParam(ADStringFromServer, "<not used by driver>");
  setStringParam(ADManufacturer, "Tucsen");
  epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d",
                DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
  setStringParam(NDDriverVersion, versionString);

  status = this->connectCamera();
  if (status != asynSuccess) {
    ERR("camera connection failed");
    TUCAM_Api_Uninit();
    setIntegerParam(ADStatus, ADStatusDisconnected);
    setStringParam(ADStatusMessage, "camera connection failed");
    callParamCallbacks();
    report(stdout, 1);
    return;
  }
  callParamCallbacks();

  startEventId_ = epicsEventCreate(epicsEventEmpty);
  /* Launch temperature monitoring task */
  this->monitoringActive = true;
  epicsThreadOpts opts;
  opts.priority = epicsThreadPriorityMedium;
  opts.stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
  opts.joinable = 1;
  epicsThreadCreateOpt("ADTucamMonitorTemperatureThread",
                       (EPICSTHREADFUNC)monitorTemperatureThreadC, this, &opts);

  /* Launch shutdown task */
  epicsAtExit(exitCallbackC, (void *)this);  // NOLINT(readability/casting)
}

/**
 * @brief ADTucam destructor, called on exit to cleanup spawned threads.
 */
ADTucam::~ADTucam() {
  const char *functionName = "~ADTucam";

  // Stop acquisiton if active
  this->stopAcquisitionThread(ADStatusIdle);

  if (this->monitoringActive) {
    INFO("Shutting down monitor thread...");
    this->monitoringActive = false;
    epicsThreadMustJoin(this->monitorThreadId);
    INFO("Done.");
  }

  if (this->camHandle_.hIdxTUCam != NULL) {
    INFO("Closing Camera...");
    this->disconnectCamera();
  }

  if (TUCAMInitialized == 1) {
    // uninitialize SDK
    INFO("Uninitializing SDK...");
    TUCAMInitialized--;
    TUCAM_Api_Uninit();
    INFO("Done.");
  }
}

/**
 * @brief Warns the user if the temperature exceeds 40 degrees.
 */
asynStatus ADTucam::warnOnExtremeTemperature(double temperatureVal) {
  const char *functionName = "warnOnExtremeTemperature";
  asynStatus status = asynSuccess;
  int emergencySignal;
  getIntegerParam(ADTucam_TemperatureEmergencySignal, &emergencySignal);
  if (temperatureVal >= 40.0 && !emergencySignal) {
    status = setIntegerParam(ADTucam_TemperatureEmergencySignal, 1);
    WARN_ARGS("Temperature exceeds 40.0 degrees: %f!!", temperatureVal);
  } else if (temperatureVal < 40.0 && emergencySignal) {
    status = setIntegerParam(ADTucam_TemperatureEmergencySignal, 0);
    INFO_ARGS("Temperature is back to normal: %f", temperatureVal);
  }

  return status;
}

/**
 * @brief Automatically handles enabling/disabling the TEC depending on the
 * temperature gradient.
 *
 * The TEC enables heat to be dissipated from the camera *only if* a temperature
 * gradient exists on both sides of the TEC plate. I.e. a coolant must be
 * flowing on the other side of the plate to remove the heat produced by the
 * camera.
 *
 * This method handles the following scenarios:
 * - If the temperature drops under `x` degrees, we can safely enable the TEC
 * since we can assume that coolant is flowing. `x` should be at a maximum room
 * temperature.
 * - If the temperature exceeds `x` degrees, we need to disable the TEC to
 * reduce the power consumption of the camera which buys more time for an
 * operator to shut down the camera safely. We should still flag this as an
 * emergency to the user since coolant is most likely not flowing.
 *
 * If we had enabled the TEC and no coolant was flowing, the camera's
 * temperature would rapidly increase due to the increased power consumption of
 * the TEC.
 */
asynStatus ADTucam::handleTEC(double temperatureVal) {
  const char *functionName = "handleTEC";
  asynStatus status = asynSuccess;
  int tecStatus;
  double tecThreshold;

  getDoubleParam(ADTucam_AutoTECThreshold, &tecThreshold);
  // TEC is active but we are above the threshold
  // We assume no temperature gradient is present
  if (this->tecActive && temperatureVal > tecThreshold) {
    WARN_ARGS("Temperature exceeds %f degrees: %f!!", tecThreshold,
              temperatureVal);
    TUCAM_Capa_SetValue(this->camHandle_.hIdxTUCam, TUIDC_ENABLETEC, 0);
    TUCAM_Capa_GetValue(this->camHandle_.hIdxTUCam, TUIDC_ENABLETEC,
                        &tecStatus);
    if (tecStatus == 0) {
      WARN("TEC DISABLED.");
      this->tecActive = false;
    }
    status = setIntegerParam(ADTucam_TECStatus, tecStatus);
  } else if (!this->tecActive && temperatureVal <= tecThreshold) {
    // TEC is inactive but we are below the threshold
    // We assume a temperature gradient is present
    WARN_ARGS("Temperature has reached an acceptable value for TEC: %f",
              temperatureVal);
    TUCAM_Capa_SetValue(this->camHandle_.hIdxTUCam, TUIDC_ENABLETEC, 1);
    TUCAM_Capa_GetValue(this->camHandle_.hIdxTUCam, TUIDC_ENABLETEC,
                        &tecStatus);
    if (tecStatus == 1) {
      WARN("TEC ENABLED.");
      this->tecActive = true;
    }
    status = setIntegerParam(ADTucam_TECStatus, tecStatus);
  }

  return status;
}

/**
 * @brief Monitors the temperature of the camera and updates the temperature PV
 */
void ADTucam::monitorTemperatureThread() {
  const char *functionName = "monitorTemperatureThread";
  double temperatureVal = 1.0f;
  int autoTEC;
  INFO("Temperature monitor thread active.");

  while (this->monitoringActive) {
    lock();
    TUCAM_Prop_GetValue(this->camHandle_.hIdxTUCam, TUIDP_TEMPERATURE,
                        &temperatureVal);
    setDoubleParam(ADTucam_Temperature, temperatureVal);
    this->warnOnExtremeTemperature(temperatureVal);
    // Auto-enable/disable TEC
    getIntegerParam(ADTucam_AutoTEC, &autoTEC);
    if (autoTEC) {
      this->handleTEC(temperatureVal);
    }
    unlock();
    callParamCallbacks();
    epicsThreadSleep(1);
  }
}

/**
 * @brief Captures images from the camera and sends them to the areaDetector
 * plugin.
 */
void ADTucam::acquisitionThread() {
  const char *functionName = "acquisitionThread";
  int status, tucStatus;
  int acquiring;
  int imageMode;
  int arrayCallbacks;
  int targetNumImages, arrayCounter, imageCounter;
  int numImages;
  int triggerMode;
  int retryOnTimeout, numRetries, retryCounter = 0;
  NDColorMode_t colorMode = NDColorModeMono;  // only grayscale at the moment
  NDDataType_t dataType = NDUInt16;           //  only 16 bit unsigned for now.

  // Lock the driver to prevent concurrent writes to params
  // This is important to ensure that the camera is not being configured
  // while we are capturing images.
  this->lock();

  // Start capturing...
  status = this->startCapture();
  if (status == asynError) {
    ERR("Failed to start capturing!");
    this->unlock();
    return;
  }
  callParamCallbacks();

  // Get the time that the camera capturing started for proper timestamping
  // later.
  std::chrono::system_clock::time_point startTime =
      std::chrono::system_clock::now();
  int64_t nsStartTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            startTime.time_since_epoch())
                            .count();
  double msStartTime = nsStartTime * 1.e-6;

  while (this->acquisitionActive) {
    // Retrieve any variable settings from PVs
    getIntegerParam(ADNumImages, &targetNumImages);
    getIntegerParam(ADTucam_RetryOnTimeout, &retryOnTimeout);
    getIntegerParam(ADTucam_NumRetries, &numRetries);

    status = this->grabImage(msStartTime);
    if (status == asynError) {
      if (this->pRaw_) this->pRaw_->release();
      this->pRaw_ = NULL;
      if (!this->acquisitionActive) {
        this->unlock();
        setIntegerParam(ADStatus, ADStatusIdle);
        setStringParam(ADStatusMessage, "Acquisition aborted by user");
        callParamCallbacks();
        return;
      }
      if (retryOnTimeout == 1 && retryCounter < numRetries) {
        retryCounter++;
        WARN_ARGS("Failed to grab image, retry %d out of %d...", retryCounter,
                  numRetries);
        continue;
      }
      this->stopCapture(ADStatusError);
      // Unlock the driver to allow other threads to access it
      setStringParam(ADStatusMessage, "Failed to grab image!");
      this->unlock();
      callParamCallbacks();
      return;
    }
    retryCounter = 0;
    getIntegerParam(NDArrayCounter, &arrayCounter);
    getIntegerParam(ADNumImages, &numImages);
    getIntegerParam(ADNumImagesCounter, &imageCounter);
    getIntegerParam(ADImageMode, &imageMode);
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

    arrayCounter++;
    imageCounter++;

    setIntegerParam(NDArrayCounter, arrayCounter);
    setIntegerParam(ADNumImagesCounter, imageCounter);

    // Perform plugin callbacks on collected frame
    if (arrayCallbacks) {
      this->unlock();
      // NOTE: Can be an expensive operation depending on plugin configuration
      doCallbacksGenericPointer(this->pRaw_, NDArrayData, 0);
      this->lock();
    }

    // Free array buffer
    if (this->pRaw_) this->pRaw_->release();
    this->pRaw_ = NULL;

    callParamCallbacks();
    // If in single mode, finish acq, if in multiple mode and reached target
    // number complete acquisition.
    if (imageMode == ADImageSingle ||
        (imageMode == ADImageMultiple && imageCounter == targetNumImages)) {
      break;
    }
  }
  setStringParam(ADStatusMessage, "Acquisition completed");
  this->stopCapture(ADStatusIdle);
  this->unlock();
  callParamCallbacks();
}

/**
 * @brief Override of ADDriver function - reads the capability of the camera and
 * dynamically fills mbbi and mbbo fields at iocInit.
 *
 * @param pasynUser Pointer to record asynUser instance
 * @param strings Array of strings to be filled with capability names
 * @param values Array of integers to be filled with capability values
 * @param severities Array of integers to be filled with capability severities
 * @param nElements Number of elements in the arrays
 * @param nIn Number of elements filled in the arrays
 *
 * @returns asynSuccess if read was successful, asynError otherwise
 */
asynStatus ADTucam::readEnum(asynUser *pasynUser, char *strings[], int values[],
                            int severities[], size_t nElements, size_t *nIn) {
  int status = asynSuccess;
  int function = pasynUser->reason;
  const char *functionName = "readEnum";

  if (function == ADTucam_BinMode) {
    status = getCapabilityText(TUIDC_RESOLUTION, strings, values, severities,
                               nElements, nIn);
  } else {
    *nIn = 0;
    status = asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Override of ADDriver function - performs callbacks on write events to
 * int PVs
 *
 * @param pasynUser Pointer to record asynUser instance
 * @param value Value written to PV
 *
 * @returns asynSuccess if write was successful, asynError otherwise
 */
asynStatus ADTucam::writeInt32(asynUser *pasynUser, epicsInt32 value) {
  const char *functionName = "writeInt32";
  int status = asynSuccess;
  int tucStatus;
  int function = pasynUser->reason;
  int currentExpRes;

  /* Set the parameter and readback in the parameter library.  This may be
   * overwritten when we read back the status at the end, but that's OK */
  status = setIntegerParam(function, value);

  if (function == ADAcquire) {
    if (this->acquisitionActive && value == 0) {
      this->stopAcquisitionThread(ADStatusIdle);
    } else if (!this->acquisitionActive && value == 1) {
      this->startAcquisitionThread();
    } else if (value == 0) {
      ERR("Acquisition not active!");
      status = asynError;
    } else {
      ERR("Acquisition already active!");
      status = asynError;
    }
  } else if (function == ADTucam_TECStatus) {
    int tecStatus;
    TUCAM_Capa_SetValue(this->camHandle_.hIdxTUCam, TUIDC_ENABLETEC, value);
    TUCAM_Capa_GetValue(this->camHandle_.hIdxTUCam, TUIDC_ENABLETEC,
                        &tecStatus);
    if (tecStatus == 1) {
      DEBUG("Enabled TEC");
      this->tecActive = true;
    } else {
      DEBUG("Disabled TEC!");
      this->tecActive = false;
    }
  } else if (function == ADTucam_GainMode) {
    // TUIDP_GLOBALGAIN is a property which requires float64 (double)
    // Although at the EPICS level we want this to be an integer (enum)
    // Possible values are:
    //  0 -> HDR
    //  1 -> HighGain
    //  2 -> LowGain
    double gainMode;
    TUCAM_Prop_SetValue(this->camHandle_.hIdxTUCam, TUIDP_GLOBALGAIN,
                        static_cast<double>(value));
    TUCAM_Prop_GetValue(this->camHandle_.hIdxTUCam, TUIDP_GLOBALGAIN,
                        &gainMode);
    status = setIntegerParam(ADTucam_GainMode, static_cast<int>(gainMode));
    if (gainMode != value) {
      ERR_ARGS("Failed to set gain mode to %d. Readback shows %d.", value,
               static_cast<int>(gainMode));
    } else {
      INFO_ARGS("Successfully set gain mode to %d.",
                static_cast<int>(gainMode));
    }
  } else if (function == ADTucam_BinMode) {
    // Set the binninig mode.
    // Possible values are:
    //  0 -> None
    //  1 -> 2x2
    //  2 -> 4x4
    // Requires the driver lock to be held to prevent writes
    // while the camera is acquiring images.
    this->lock();
    if (this->acquisitionActive) {
      ERR("Cannot change binning mode while acquisition is active!");
      this->unlock();
      return asynError;
    }
    int binMode;
    status |= setCapability(TUIDC_RESOLUTION, value);
    getCapability(TUIDC_RESOLUTION, binMode);
    status |= setIntegerParam(ADTucam_BinMode, binMode);
    status |= setCurrentROI();
    this->unlock();
  } else if (function == ADMinX || function == ADMinY || function == ADSizeX ||
             function == ADSizeY) {
    this->lock();
    if (this->acquisitionActive) {
      ERR("Cannot change ROI while acquisition is active!");
      this->unlock();
      return asynError;
    }
    status |= setCameraROI();
    status |= setCurrentROI();
    this->unlock();
  } else if (function == ADReverseX) {
    status = setCapability(TUIDC_HORIZONTAL, value);
    status = getCapability(TUIDC_HORIZONTAL, value);
    if (status) {
      value = 0;
    }
    status = setIntegerParam(ADReverseX, value);
  } else if (function == ADReverseY) {
    status = setCapability(TUIDC_VERTICAL, value);
    status = getCapability(TUIDC_VERTICAL, value);
    if (status) {
      value = 0;
    }
    status = setIntegerParam(ADReverseY, value);
  } else if (function == ADTriggerMode || function == ADTucam_TriggerExposure) {
    status = this->setCameraTrigger();
    status = this->setCurrentTrigger();
  } else if (function == ADTucam_TriggerOut1Mode ||
             function == ADTucam_TriggerOut1Edge) {
    if (triggerOutSupport_) {
      status = this->setCameraTriggerOut(0);
      status = this->setCurrentTriggerOut(0);
    } else {
      status = setIntegerParam(function, 0);
    }
  } else if (function == ADTucam_TriggerOut2Mode ||
             function == ADTucam_TriggerOut2Edge) {
    if (triggerOutSupport_) {
      status = this->setCameraTriggerOut(1);
      status = this->setCurrentTriggerOut(1);
    } else {
      status |= setIntegerParam(function, 0);
    }
  } else if (function == ADTucam_TriggerOut3Mode ||
             function == ADTucam_TriggerOut3Edge) {
    if (triggerOutSupport_) {
      status |= this->setCameraTriggerOut(2);
      status |= this->setCurrentTriggerOut(2);
    } else {
      status |= setIntegerParam(function, 0);
    }
  } else if (function == ADTucam_FrameFormat) {
    frameHandle_.ucFormatGet = frameFormats[value];
  } else if (function == ADTucam_BitDepth) {
    status |= setCapability(TUIDC_BITOFDEPTH, value);
    status |= getCapability(TUIDC_BITOFDEPTH, value);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_FanGear) {
    status |= setCapability(TUIDC_FAN_GEAR, value);
    status |= getCapability(TUIDC_FAN_GEAR, value);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_ImageMode) {
    status |= setCapability(TUIDC_IMGMODESELECT, value);
    status |= getCapability(TUIDC_IMGMODESELECT, value);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_AutoExposure) {
    status |= setCapability(TUIDC_ATEXPOSURE, value);
    status |= getCapability(TUIDC_ATEXPOSURE, value);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_AutoLevels) {
    status |= setCapability(TUIDC_ATLEVELS, value);
    status |= getCapability(TUIDC_ATLEVELS, value);
    status |= setIntegerParam(function, value);
    int hist = (value != 0);
    status |= setCapability(TUIDC_HISTC, hist);
    status |= getCapability(TUIDC_HISTC, hist);
    status |= setIntegerParam(ADTucam_Histogram, hist);
  } else if (function == ADTucam_Histogram) {
    status |= setCapability(TUIDC_HISTC, value);
    status |= getCapability(TUIDC_HISTC, value);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_Enhance) {
    status |= setCapability(TUIDC_ENHANCE, value);
    status |= getCapability(TUIDC_ENHANCE, value);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_DefectCorr) {
    status |= setCapability(TUIDC_DFTCORRECTION, value);
    status |= getCapability(TUIDC_DFTCORRECTION, value);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_Denoise) {
    status |= setCapability(TUIDC_ENABLEDENOISE, value);
    status |= getCapability(TUIDC_ENABLEDENOISE, value);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_FlatCorr) {
    status |= setCapability(TUIDC_FLTCORRECTION, value);
    status |= getCapability(TUIDC_FLTCORRECTION, value);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_TriggerSoftware) {
    int acquire, triggerMode;
    getIntegerParam(ADAcquire, &acquire);
    getIntegerParam(ADTriggerMode, &triggerMode);
    if (acquire && triggerMode == TUCCM_TRIGGER_SOFTWARE) {
      tucStatus = TUCAM_Cap_DoSoftwareTrigger(this->camHandle_.hIdxTUCam);
      if (tucStatus != TUCAMRET_SUCCESS) {
        ERR_ARGS("Failed to do software trigger (0x%x)", tucStatus);
        status = asynError;
      }
    }
  } else {
    if (function < FIRST_TUCAM_PARAM) {
      status = ADDriver::writeInt32(pasynUser, value);
    }
  }
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();

  if (status != asynSuccess) {
    ERR_ARGS("error, status=%d function=%d, value=%d", status, function, value);
  } else {
    DEBUG_ARGS("function=%d, value=%d", function, value);
  }

  return (asynStatus)status;
}

/**
 * @brief Override of ADDriver function - performs callbacks on write events to
 * flaot PVs
 *
 * @param pasynUser Pointer to record asynUser instance
 * @param value Value written to PV
 *
 * @returns asynSuccess if write was successful, asynError otherwise
 */
asynStatus ADTucam::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
  const char *functionName = "writeFloat64";
  int status = asynSuccess;
  int function = pasynUser->reason;

  /* Set the parameter and readback in the parameter library.  This may be
   * overwritten when we read back the status at the end, but that's OK */
  status |= setDoubleParam(function, value);

  double period;
  getDoubleParam(ADAcquirePeriod, &period);
  if (function == ADAcquirePeriod) {
    if (period <= 0) {
      period = 0.001;
    }
    double frequency = 1 / period;
    INFO_ARGS("Setting frame rate to: %f fps", frequency);
    TUCAM_Prop_SetValue(this->camHandle_.hIdxTUCam, TUIDP_FRAME_RATE,
                        frequency);
    TUCAM_Prop_GetValue(this->camHandle_.hIdxTUCam, TUIDP_FRAME_RATE,
                        &frequency);
    setDoubleParam(ADAcquirePeriod, 1 / frequency);
    INFO_ARGS("Frame rate set to: %f fps", frequency);
  }
  if (function == ADAcquireTime) {
    // if exposure time is set to something longer than acquire period, change
    // acquire period to be the same
    double expose_msec;

    if (value > period) {
      setDoubleParam(ADAcquirePeriod, value);
    }
    // The factor of 1000 was used in the other IOC
    expose_msec = value * 1000.0;
    TUCAM_Prop_SetValue(this->camHandle_.hIdxTUCam, TUIDP_EXPOSURETM,
                        expose_msec);
    TUCAM_Prop_GetValue(this->camHandle_.hIdxTUCam, TUIDP_EXPOSURETM,
                        &expose_msec);
    setDoubleParam(ADAcquireTime, expose_msec / 1000.0);
  } else if (function == ADTucam_TemperatureSetpoint) {
    // The TUIDP_TEMPERATURE takes a value between 0 and 100 which maps to -50C
    // to 50C The user input is in the range -50C to 50C so we adjust it
    // accordingly
    double minTemperature = -50.0;
    double maxTemperature = 50.0;
    double temperature = value;
    if (temperature < minTemperature || temperature > maxTemperature) {
      WARN_ARGS(
          "Temperature setpoint %f is out of range. Valid range is %f to "
          "%f! Clamping the value to the valid range...",
          value, minTemperature, maxTemperature);
      temperature =
          std::min(maxTemperature, std::max(temperature, minTemperature));
      // Set the corrected value
      status |= setDoubleParam(ADTucam_TemperatureSetpoint, temperature);
    }
    double rescaledTemperature = temperature + 50;
    TUCAM_Prop_SetValue(this->camHandle_.hIdxTUCam, TUIDP_TEMPERATURE,
                        rescaledTemperature);
    DEBUG_ARGS("New temperature setpoint detected: degrees=%fC, setting=%f",
               temperature, rescaledTemperature);
  } else if (function == ADGain) {
    status |= setProperty(TUIDP_GLOBALGAIN, value);
    status |= getProperty(TUIDP_GLOBALGAIN, value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_TriggerDelay) {
    status |= this->setCameraTrigger();
    status |= this->setCurrentTrigger();
  } else if (function == ADTucam_TriggerOut1Delay ||
             function == ADTucam_TriggerOut1Width) {
    if (triggerOutSupport_) {
      status |= this->setCameraTriggerOut(0);
      status |= this->setCurrentTriggerOut(0);
    } else {
      status |= setDoubleParam(function, 0);
    }
  } else if (function == ADTucam_TriggerOut2Delay ||
             function == ADTucam_TriggerOut2Width) {
    if (triggerOutSupport_) {
      status |= this->setCameraTriggerOut(1);
      status |= this->setCurrentTriggerOut(1);
    } else {
      status |= setDoubleParam(function, 0);
    }
  } else if (function == ADTucam_TriggerOut3Delay ||
             function == ADTucam_TriggerOut3Width) {
    if (triggerOutSupport_) {
      status |= this->setCameraTriggerOut(2);
      status |= this->setCurrentTriggerOut(2);
    } else {
      status = setDoubleParam(function, 0);
    }
  } else if (function == ADTucam_Brightness) {
    status = setProperty(TUIDP_BRIGHTNESS, value);
    status = getProperty(TUIDP_BRIGHTNESS, value);
    status = setDoubleParam(function, value);
  } else if (function == ADTucam_BlackLevel) {
    status = setProperty(TUIDP_BLACKLEVEL, value);
    status = getProperty(TUIDP_BLACKLEVEL, value);
    if (status) value = 0;
    status = setDoubleParam(function, value);
  } else if (function == ADTucam_Sharpness) {
    status = setProperty(TUIDP_SHARPNESS, value);
    status = getProperty(TUIDP_SHARPNESS, value);
    if (status) value = 0;
    status = setDoubleParam(function, value);
  } else if (function == ADTucam_NoiseLevel) {
    status = setProperty(TUIDP_NOISELEVEL, value);
    status = getProperty(TUIDP_NOISELEVEL, value);
    if (status) value = 0;
    status = setDoubleParam(function, value);
  } else if (function == ADTucam_HDRK) {
    status = setProperty(TUIDP_HDR_KVALUE, value);
    status = getProperty(TUIDP_HDR_KVALUE, value);
    if (status) value = 0;
    status = setDoubleParam(function, value);
  } else if (function == ADTucam_Gamma) {
    status = setProperty(TUIDP_GAMMA, value);
    status = getProperty(TUIDP_GAMMA, value);
    if (status) value = 0;
    status = setDoubleParam(function, value);
  } else if (function == ADTucam_Contrast) {
    status = setProperty(TUIDP_CONTRAST, value);
    status = getProperty(TUIDP_CONTRAST, value);
    if (status) value = 0;
    status = setDoubleParam(function, value);
  } else if (function == ADTucam_LeftLevel) {
    status = setProperty(TUIDP_LFTLEVELS, value);
    status = getProperty(TUIDP_LFTLEVELS, value);
    if (status) value = 0;
    status = setDoubleParam(function, value);
  } else if (function == ADTucam_RightLevel) {
    status = setProperty(TUIDP_RGTLEVELS, value);
    status = getProperty(TUIDP_RGTLEVELS, value);
    if (status) value = 0;
    status = setDoubleParam(function, value);
  } else if (function == ADTucam_AutoTECThreshold) {
    // Dangerous to have auto TEC threshold above 40 degrees
    if (value >= 40.0) {
      WARN_ARGS(
          "Requested TEC threshold %f is above 40 degrees! Setting it to "
          "40 degrees...",
          value);
      setDoubleParam(ADTucam_AutoTECThreshold, 40.0);
    }
  } else {
    if (function < FIRST_TUCAM_PARAM) {
      status = ADDriver::writeFloat64(pasynUser, value);
    }
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();

  if (status) {
    ERR_ARGS("error, status=%d function=%d, value=%f", status, function, value);
  } else {
    DEBUG_ARGS("function=%d, value=%f", function, value);
  }

  return (asynStatus)status;
}

/**
 * @brief Grabs a single image from the camera and stores it in the NDArray
 * buffer.
 *
 * We assume that the driver is locked when this function is called.
 *
 * @param msStartTime Time in milliseconds when the acquisition started
 *
 * @returns asynSuccess if image was successfully grabbed, asynError otherwise
 */
asynStatus ADTucam::grabImage(double msStartTime) {
  static const char *functionName = "grabImage";
  asynStatus status = asynSuccess;
  int tucStatus;
  int nCols, nRows;
  int pixelFormat, channels, pixelBytes;
  size_t dataSize, tDataSize;
  NDDataType_t dataType = NDUInt16;
  NDColorMode_t colorMode = NDColorModeMono;
  int numColors = 1;
  int pixelSize = 2;
  size_t dims[3] = {0};
  int nDims;
  int count;
  double exposureTimeout, exposureTime, acquirePeriod;
  TUCAM_IMG_HEADER frameHeader;

  getDoubleParam(ADAcquireTime, &exposureTime);
  getDoubleParam(ADAcquirePeriod, &acquirePeriod);

  exposureTimeout = (exposureTime + acquirePeriod) * 1000 + 2000;

  INFO("Waiting for frame...");
  this->unlock();
  tucStatus = TUCAM_Buf_WaitForFrame(camHandle_.hIdxTUCam, &frameHandle_,
                                     exposureTimeout);
  this->lock();
  if (tucStatus == TUCAMRET_ABORT) {
    INFO_ARGS("acquisition aborted by user (0x%x)", tucStatus);
    return asynError;
  } else if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("failed to wait for frame (0x%x)", tucStatus);
    return asynError;
  }

  // Width & height are array dimensions
  nCols = frameHandle_.usWidth;
  nRows = frameHandle_.usHeight;

  // Image format
  pixelFormat = frameHandle_.ucFormat;
  // Number of channels
  channels = frameHandle_.ucChannels;
  // Bytes per pixel
  pixelBytes = frameHandle_.ucElemBytes;
  // Frame image data size
  tDataSize = frameHandle_.uiImgSize;

  /* There is zero documentation on what the formats mean
   * Most of the below is gleaned through trial and error */
  if (pixelFormat == TUFRM_FMT_RAW) {
    // Raw data - no filtering applied
    if (pixelBytes == 1) {
      dataType = NDUInt8;
      pixelSize = 1;
    } else if (pixelBytes == 2) {
      dataType = NDUInt16;
      pixelSize = 2;
    }
    colorMode = NDColorModeMono;
    numColors = 1;
  } else if (pixelFormat == TUFRM_FMT_USUAl) {
    if (pixelBytes == 1) {
      dataType = NDUInt8;
      pixelSize = 1;
    } else if (pixelBytes == 2) {
      dataType = NDUInt16;
      pixelSize = 2;
    }

    if (channels == 1) {
      colorMode = NDColorModeMono;
      numColors = 1;
    } else if (channels == 3) {
      colorMode = NDColorModeRGB1;
      numColors = 3;
    }
  } else if (pixelFormat == TUFRM_FMT_RGB888) {
    dataType = NDUInt8;
    pixelSize = 1;
    colorMode = NDColorModeRGB1;
    numColors = 3;
  } else {
    ERR_ARGS("Unsupported pixel format %d", pixelFormat);
    return asynError;
  }
  if (numColors == 1) {
    nDims = 2;
    dims[0] = nCols;
    dims[1] = nRows;
  } else {
    nDims = 3;
    dims[0] = 3;
    dims[1] = nCols;
    dims[2] = nRows;
  }

  dataSize = dims[0] * dims[1] * pixelSize;
  if (nDims == 3) dataSize *= dims[2];

  if (dataSize != tDataSize) {
    ERR_ARGS("data size mismatch: calculated=%zu, reported=%zu", dataSize,
             tDataSize);
    return asynError;
  }

  setIntegerParam(NDArraySizeX, nCols);
  setIntegerParam(NDArraySizeY, nRows);
  setIntegerParam(NDArraySize, static_cast<int>(dataSize));
  setIntegerParam(NDDataType, dataType);
  setIntegerParam(NDColorMode, colorMode);

  // Copy frame to an NDArray buffer
  pRaw_ = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
  if (!pRaw_) {
    ERR_ARGS(
        "[%s] Serious problem: not enough buffers left. Aborting acquisition!",
        portName);
    return asynError;
  }
  memcpy(pRaw_->pData, this->frameHandle_.pBuffer + this->frameHandle_.usOffset,
         dataSize);
  getIntegerParam(NDArrayCounter, &count);
  pRaw_->uniqueId = count;
  updateTimeStamp(&pRaw_->epicsTS);

  // Copy the header data from the frame
  // `frameHeader.dblTimeStamp` is in ms since the TUCAM_Cap_Start (it is the
  // acquire/exposure start time) `frameHeader.dblTimeLast` is in ms since the
  // frameHeader.dblTimeStamp (it is the acquire end time, NOT the exposure end
  // time) Precision is to nanoseconds
  memcpy(&frameHeader, this->frameHandle_.pBuffer, sizeof(TUCAM_IMG_HEADER));
  DEBUG_ARGS("Tucam timestamp start: %f ms", frameHeader.dblTimeStamp);
  DEBUG_ARGS("Tucam timestamp end: %f ms", frameHeader.dblTimeLast);
  double timeStamp = (msStartTime + frameHeader.dblTimeStamp) * 1.e-3;
  DEBUG_ARGS("Tucam timestamp FINAL: %f s", timeStamp);
  pRaw_->timeStamp = timeStamp;

  getAttributes(pRaw_->pAttributeList);

  pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32,
                             &colorMode);

  return status;
}

/**
 * @brief Allocates camera buffer and starts acquisition.
 *
 * We assume that the driver is locked when this function is called to avoid
 * camera changes from a concurrent thread.
 *
 * @returns asynSuccess if acquisition was successfully started, asynError
 * otherwise
 */
asynStatus ADTucam::startCapture() {
  static const char *functionName = "startCapture";
  int status = asynSuccess;
  int tucStatus;
  int triggerMode;

  tucStatus = TUCAM_Buf_Alloc(this->camHandle_.hIdxTUCam, &this->frameHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to allocate buffer, status=%d", tucStatus);
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADStatus, ADStatusError);
    setStringParam(ADStatusMessage,
                   "Failed to allocate buffer to start acquisition");
    callParamCallbacks();
    return asynError;
  }

  getIntegerParam(ADTriggerMode, &triggerMode);
  tucStatus = TUCAM_Cap_Start(this->camHandle_.hIdxTUCam, triggerMode);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to start acquisition, status=%d", tucStatus);
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADStatus, ADStatusError);
    setStringParam(ADStatusMessage, "Failed to start acquisition");
    callParamCallbacks();
    return asynError;
  }

  status |= setIntegerParam(ADStatus, ADStatusAcquire);
  status |= setStringParam(ADStatusMessage, "Acquiring...");
  INFO("Acquisition started.");
  status |= setIntegerParam(ADNumImagesCounter, 0);
  status |= setIntegerParam(ADAcquire, 1);
  this->acquisitionActive = true;

  return (asynStatus)status;
}

/**
 * @brief Aborts & stops acquisition and releases camera buffer.
 *
 * @param acquireStatus Status to set the ADStatus PV to
 *
 * @returns asynSuccess if acquisition was successfully stopped, asynError
 * otherwise
 */
asynStatus ADTucam::stopCapture(int acquireStatus) {
  static const char *functionName = "stopCapture";
  int status = asynSuccess;
  int tucStatus;

  if (this->acquisitionActive) {
    this->acquisitionActive = false;
    this->unlock();
    tucStatus = TUCAM_Buf_AbortWait(camHandle_.hIdxTUCam);
    if (tucStatus != TUCAMRET_SUCCESS) {
      ERR_ARGS("unable to abort wait (%d)", tucStatus);
    }

    tucStatus = TUCAM_Cap_Stop(camHandle_.hIdxTUCam);
    if (tucStatus != TUCAMRET_SUCCESS) {
      ERR_ARGS("unable to stop acquisition (%d)", tucStatus);
    }

    tucStatus = TUCAM_Buf_Release(camHandle_.hIdxTUCam);
    if (tucStatus != TUCAMRET_SUCCESS) {
      ERR_ARGS("unable to release camera buffer (%d)", tucStatus);
    }

    this->lock();
    status = setIntegerParam(ADStatus, acquireStatus);
    status = setIntegerParam(ADAcquire, 0);
  } else {
    WARN("Acquisition not active!");
  }

  return (asynStatus)status;
}

/**
 * @brief Starts acquisition thread.
 *
 * @returns asynSuccess if acquisition thread was successfully started,
 * asynError otherwise
 */
asynStatus ADTucam::startAcquisitionThread() {
  static const char *functionName = "startAcquisitionThread";
  int status = asynSuccess;

  epicsEventSignal(startEventId_);
  epicsThreadOpts opts;
  opts.priority = epicsThreadPriorityHigh;
  opts.stackSize = epicsThreadGetStackSize(epicsThreadStackBig);
  opts.joinable = 1;
  INFO("Spawning main acquisition thread...");
  this->acquisitionThreadId =
      epicsThreadCreateOpt("ADTucamAcquisitionThread",
                           (EPICSTHREADFUNC)acquisitionThreadC, this, &opts);
  this->acquisitionActive = true;
  callParamCallbacks();

  return (asynStatus)status;
}

/**
 * @brief Stops acquisition thread.
 *
 * @param acquireStatus Status to set the ADStatus PV to
 *
 * @returns asynSuccess if acquisition thread was successfully stopped,
 * asynError otherwise
 */
asynStatus ADTucam::stopAcquisitionThread(int acquireStatus) {
  static const char *functionName = "stopAcquisitionThread";
  int status = asynSuccess;
  int tucStatus;

  this->stopCapture(acquireStatus);
  INFO("Waiting for acquisition thread to join...");
  epicsThreadMustJoin(this->acquisitionThreadId);
  INFO("Acquisition stopped.");
  callParamCallbacks();

  return (asynStatus)status;
}

/**
 * @brief Connect to the camera and set the PVs with the camera information.
 *
 * @returns asynSuccess if camera was successfully connected, asynError
 * otherwise
 */
asynStatus ADTucam::connectCamera() {
  static const char *functionName = "connectCamera";
  int tucStatus;
  int status = asynSuccess;

  // Init API
  char szPath[1024] = {0};
  getcwd(szPath, 1024);
  apiHandle_.pstrConfigPath = szPath;
  apiHandle_.uiCamCount = 0;

  tucStatus = TUCAM_Api_Init(&apiHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("TUCAM API init failed (0x%x)", tucStatus);
    return asynError;
  }
  if (apiHandle_.uiCamCount < 1) {
    ERR_ARGS("no camera detected (0x%x)", tucStatus);
    return asynError;
  }

  TUCAMInitialized++;

  // Init camera
  camHandle_.hIdxTUCam = NULL;
  camHandle_.uiIdxOpen = cameraId_;

  tucStatus = TUCAM_Dev_Open(&camHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("open camera device failed (0x%x)", tucStatus);
    return asynError;
  }

  // Set PVs from camera information
  status |= setCamInfo(ADModel, TUIDI_CAMERA_MODEL, 0);
  status |= setCamInfo(ADFirmwareVersion, TUIDI_VERSION_FRMW, 0);
  status |= setCamInfo(ADSDKVersion, TUIDI_VERSION_API, 0);
  status |= setCamInfo(ADTucam_Bus, TUIDI_BUS, 0);
  status |= setCamInfo(ADTucam_ProductID, TUIDI_PRODUCT, 1);
  status |= setCamInfo(ADMaxSizeX, TUIDI_CURRENT_WIDTH, 2);
  status |= setCamInfo(ADMaxSizeY, TUIDI_CURRENT_HEIGHT, 2);
  status |= setSerialNumber();
  status |= setCurrentROI();
  status |= this->setCurrentTrigger();
  triggerOutSupport_ = 1;
  for (int port = 0; port < 3; port++) {
    if (this->setCurrentTriggerOut(port) != asynSuccess) {
      triggerOutSupport_ = 0;
      break;
    }
  }

  return (asynStatus)status;
}

/**
 * @brief Disconnect from the camera.
 *
 * @returns asynSuccess if camera was successfully disconnected, asynError
 * otherwise
 */
asynStatus ADTucam::disconnectCamera(void) {
  static const char *functionName = "disconnectCamera";
  int tucStatus;
  int acquiring;
  int status;

  // Stop acquiring, if active
  if (this->acquisitionActive) {
    status = this->stopCapture(ADStatusIdle);
  }

  tucStatus = TUCAM_Dev_Close(camHandle_.hIdxTUCam);
  if (tucStatus != TUCAMRET_SUCCESS) {
    status = asynError;
    ERR_ARGS("unable close camera (0x%x)\n", tucStatus);
  }
  return (asynStatus)status;
}

/**
 * @brief Set EPICS PVs to match the camera's current trigger mode.
 *
 * @returns asynSuccess if trigger mode was successfully set, asynError
 * otherwise
 */
asynStatus ADTucam::setCurrentTrigger() {
  static const char *functionName = "setCurrentTrigger";
  int status = asynSuccess;
  int tucStatus;

  tucStatus = TUCAM_Cap_GetTrigger(camHandle_.hIdxTUCam, &triggerHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to get trigger, status=%d", tucStatus);
    return asynError;
  }

  status = setIntegerParam(ADTriggerMode, triggerHandle_.nTgrMode);
  status = setIntegerParam(ADTucam_TriggerEdge, triggerHandle_.nEdgeMode);
  status = setIntegerParam(ADTucam_TriggerExposure, triggerHandle_.nExpMode);
  status = setDoubleParam(ADTucam_TriggerDelay, triggerHandle_.nDelayTm / 1.0e6);

  return (asynStatus)status;
}

/**
 * @brief Read EPICS PVs to set the camera's trigger mode.
 *
 * @returns asynSuccess if trigger mode was successfully set, asynError
 * otherwise
 */
asynStatus ADTucam::setCameraTrigger() {
  static const char *functionName = "setCameraTrigger";
  int status = asynSuccess;
  int tucStatus;
  int triggerMode, triggerEdge, triggerExposure;
  double triggerDelay;

  getIntegerParam(ADTriggerMode, &triggerMode);
  getIntegerParam(ADTucam_TriggerEdge, &triggerEdge);
  getIntegerParam(ADTucam_TriggerExposure, &triggerExposure);
  getDoubleParam(ADTucam_TriggerDelay, &triggerDelay);

  frameHandle_.uiRsdSize = 1;

  triggerHandle_.nTgrMode = triggerMode;
  triggerHandle_.nEdgeMode = triggerEdge;
  triggerHandle_.nExpMode = triggerExposure;
  triggerHandle_.nFrames = 1;
  triggerHandle_.nDelayTm = static_cast<int>(triggerDelay * 1e6);

  tucStatus = TUCAM_Cap_SetTrigger(camHandle_.hIdxTUCam, triggerHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to set trigger, status=%d", tucStatus);
    return asynError;
  }

  return (asynStatus)status;
}

/**
 * @brief Set EPICS PVs using the camera's current trigger out settings.
 *
 * @param port Trigger out port to set
 *
 * @returns asynSuccess if trigger out settings were successfully set, asynError
 * otherwise
 */
asynStatus ADTucam::setCurrentTriggerOut(int port) {
  static const char *functionName = "setCurrentTriggerOut";
  int status = asynSuccess;
  int tucStatus;

  tucStatus =
      TUCAM_Cap_GetTriggerOut(camHandle_.hIdxTUCam, &triggerOutHandle_[port]);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to get trigger out, port=%d, status=%d\n", port,
             tucStatus);
    return asynError;
  }

  if (port == 0) {
    status |= setIntegerParam(ADTucam_TriggerOut1Mode,
                              triggerOutHandle_[port].nTgrOutMode);
    status |= setIntegerParam(ADTucam_TriggerOut1Edge,
                              triggerOutHandle_[port].nEdgeMode);
    status |= setDoubleParam(ADTucam_TriggerOut1Delay,
                             triggerOutHandle_[port].nDelayTm / 1.0e6);
    status |= setDoubleParam(ADTucam_TriggerOut1Width,
                             triggerOutHandle_[port].nWidth / 1.0e6);
  } else if (port == 1) {
    status |= setIntegerParam(ADTucam_TriggerOut2Mode,
                              triggerOutHandle_[port].nTgrOutMode);
    status |= setIntegerParam(ADTucam_TriggerOut2Edge,
                              triggerOutHandle_[port].nEdgeMode);
    status |= setDoubleParam(ADTucam_TriggerOut2Delay,
                             triggerOutHandle_[port].nDelayTm / 1.0e6);
    status |= setDoubleParam(ADTucam_TriggerOut2Width,
                             triggerOutHandle_[port].nWidth / 1.0e6);
  } else if (port == 2) {
    status |= setIntegerParam(ADTucam_TriggerOut3Mode,
                              triggerOutHandle_[port].nTgrOutMode);
    status |= setIntegerParam(ADTucam_TriggerOut3Edge,
                              triggerOutHandle_[port].nEdgeMode);
    status |= setDoubleParam(ADTucam_TriggerOut3Delay,
                             triggerOutHandle_[port].nDelayTm / 1.0e6);
    status |= setDoubleParam(ADTucam_TriggerOut3Width,
                             triggerOutHandle_[port].nWidth / 1.0e6);
  }

  return (asynStatus)status;
}

/**
 * @brief Read EPICS PVs to set the camera's trigger out settings.
 *
 * @param port Trigger out port to set
 *
 * @returns asynSuccess if trigger out settings were successfully set, asynError
 * otherwise
 */
asynStatus ADTucam::setCameraTriggerOut(int port) {
  static const char *functionName = "setCameraTriggerOut";
  int status = asynSuccess;
  int tucStatus;
  int triggerMode, triggerEdge;
  double triggerDelay, triggerWidth;

  if (port == 0) {
    getIntegerParam(ADTucam_TriggerOut1Mode, &triggerMode);
    getIntegerParam(ADTucam_TriggerOut1Edge, &triggerEdge);
    getDoubleParam(ADTucam_TriggerOut1Delay, &triggerDelay);
    getDoubleParam(ADTucam_TriggerOut1Width, &triggerWidth);
  } else if (port == 1) {
    getIntegerParam(ADTucam_TriggerOut2Mode, &triggerMode);
    getIntegerParam(ADTucam_TriggerOut2Edge, &triggerEdge);
    getDoubleParam(ADTucam_TriggerOut2Delay, &triggerDelay);
    getDoubleParam(ADTucam_TriggerOut2Width, &triggerWidth);
  } else if (port == 2) {
    getIntegerParam(ADTucam_TriggerOut3Mode, &triggerMode);
    getIntegerParam(ADTucam_TriggerOut3Edge, &triggerEdge);
    getDoubleParam(ADTucam_TriggerOut3Delay, &triggerDelay);
    getDoubleParam(ADTucam_TriggerOut3Width, &triggerWidth);
  }

  triggerOutHandle_[port].nTgrOutPort = port;
  triggerOutHandle_[port].nTgrOutMode = triggerMode;
  triggerOutHandle_[port].nEdgeMode = triggerEdge;
  triggerOutHandle_[port].nDelayTm = static_cast<int>(triggerDelay * 1e6);
  triggerOutHandle_[port].nWidth = static_cast<int>(triggerWidth * 1e6);

  tucStatus =
      TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, triggerOutHandle_[port]);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to set trigger out, port=%d, status=%d\n", port,
             tucStatus);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Set EPICS PVs using the camera's current ROI settings.
 *
 * @return asynStatus
 */
asynStatus ADTucam::setCurrentROI() {
  static const char *functionName = "setCurrentROI";
  int minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;
  int tucStatus;
  int status = asynSuccess;

  getIntegerParam(ADMaxSizeX, &maxSizeX);
  getIntegerParam(ADMaxSizeY, &maxSizeY);

  TUCAM_ROI_ATTR roiAttr;
  tucStatus = TUCAM_Cap_GetROI(camHandle_.hIdxTUCam, &roiAttr);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("failed to get ROI, status=%d", tucStatus);
    return asynError;
  }

  if (roiAttr.bEnable) {
    minX = roiAttr.nHOffset;
    minY = roiAttr.nVOffset;
    sizeX = roiAttr.nWidth;
    sizeY = roiAttr.nHeight;
  } else {
    minX = 0;
    minY = 0;
    sizeX = maxSizeX;
    sizeY = maxSizeY;
  }

  status = setIntegerParam(ADMinX, minX);
  status = setIntegerParam(ADMinY, minY);
  status = setIntegerParam(ADSizeX, sizeX);
  status = setIntegerParam(ADSizeY, sizeY);

  return (asynStatus)status;
}

/**
 * @brief Read EPICS PVs to set the camera's ROI settings.
 *
 * @return asynStatus
 */
asynStatus ADTucam::setCameraROI() {
  static const char *functionName = "setCameraROI";
  int status = asynSuccess;
  int tucStatus;
  int minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;

  getIntegerParam(ADMinX, &minX);
  getIntegerParam(ADMinY, &minY);
  getIntegerParam(ADSizeX, &sizeX);
  getIntegerParam(ADSizeY, &sizeY);
  getIntegerParam(ADMaxSizeX, &maxSizeX);
  getIntegerParam(ADMaxSizeY, &maxSizeY);

  if (minX + sizeX > maxSizeX) {
    sizeX = maxSizeX - minX;
    status = setIntegerParam(ADSizeX, sizeX);
  }
  if (minY + sizeY > maxSizeY) {
    sizeY = maxSizeY - minY;
    status = setIntegerParam(ADSizeX, sizeY);
  }

  TUCAM_ROI_ATTR roiAttr;
  roiAttr.bEnable = true;
  roiAttr.nHOffset = minX;
  roiAttr.nVOffset = minY;
  roiAttr.nWidth = sizeX;
  roiAttr.nHeight = sizeY;

  tucStatus = TUCAM_Cap_SetROI(this->camHandle_.hIdxTUCam, roiAttr);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("SetROI error, status=0x%x\n", tucStatus);
    return asynError;
  }

  tucStatus = TUCAM_Cap_GetROI(this->camHandle_.hIdxTUCam, &roiAttr);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("GetROI error, status=%d\n", tucStatus);
    return asynError;
  }

  status |= setIntegerParam(ADMinX, roiAttr.nHOffset);
  status |= setIntegerParam(ADMinY, roiAttr.nVOffset);
  status |= setIntegerParam(ADSizeX, roiAttr.nWidth);
  status |= setIntegerParam(ADSizeY, roiAttr.nHeight);

  return (asynStatus)status;
}

/**
 * @brief Get the camera information.
 * @param nID TUIDI option to read
 * @param sBuf Buffer to store the string information
 * @param val Integer to store the value
 * @return asynStatus
 */
asynStatus ADTucam::getCamInfo(int nID, char *sBuf, int &val) {
  static const char *functionName = "getCamInfo";

  // Get camera information
  int tucStatus;
  TUCAM_VALUE_INFO valInfo;
  const int sSize = 1024;
  char sInfo[sSize] = {0};
  valInfo.pText = sInfo;
  valInfo.nTextSize = 1024;

  valInfo.nID = nID;
  tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
  if (tucStatus == TUCAMRET_SUCCESS) {
    val = valInfo.nValue;
    sBuf = valInfo.pText;
    return asynSuccess;
  } else {
    ERR_ARGS("could not get %d (0x%x)", nID, tucStatus);
    return asynError;
  }

  return asynSuccess;
}

/**
 * @brief Set EPICS PVs using camera information.
 * @param param EPICS parameter to set
 * @param nID TUIDI option to read
 * @param dtype Data type of the parameter
 *              0 - string
 *              1 - double
 *              2 - integer
 * @return asynStatus
 */
asynStatus ADTucam::setCamInfo(int param, int nID, int dtype) {
  static const char *functionName = "setCamInfo";
  int status = asynSuccess;
  const int sSize = 1024;
  char sInfo[sSize] = {0};
  int nValue;

  status = this->getCamInfo(nID, sInfo, nValue);
  if (status == asynSuccess) {
    if (param == ADTucam_Bus) {
      if (nValue == 768) {
        status |= setStringParam(ADTucam_Bus, "USB3.0");
      } else {
        status |= setStringParam(ADTucam_Bus, "USB2.0");
      }
    } else if (dtype == 0) {
      status |= setStringParam(param, sInfo);
    } else if (dtype == 1) {
      status |= setDoubleParam(param, nValue);
    } else if (dtype == 2) {
      status |= setIntegerParam(param, nValue);
    }
  } else {
    ERR_ARGS("param %d id %d (error=0x%x)", param, nID, status);
    return asynError;
  }

  return (asynStatus)status;
}

/**
 * @brief Set EPICS PV using the camera's serial number.
 * @return asynStatus
 */
asynStatus ADTucam::setSerialNumber() {
  static const char *functionName = "setSerialNumber";
  int status = asynSuccess;
  int tucStatus;
  char cSN[TUSN_SIZE] = {0};
  TUCAM_REG_RW regRW;

  regRW.nRegType = TUREG_SN;
  regRW.pBuf = &cSN[0];
  regRW.nBufSize = TUSN_SIZE;

  tucStatus = TUCAM_Reg_Read(camHandle_.hIdxTUCam, regRW);
  if (tucStatus == TUCAMRET_SUCCESS) {
    setStringParam(ADSerialNumber, cSN);
  } else {
    ERR_ARGS("unable to get serial number (error=0x%x)", tucStatus);
    return asynError;
  }

  return (asynStatus)status;
}

/**
 * @brief Get the camera's property value.
 *
 * @param property TUIDP property to get
 * @param value Value to store the property value
 *
 * @return asynStatus
 */
asynStatus ADTucam::getProperty(int property, double &value) {
  static const char *functionName = "getProperty";
  int status = asynSuccess;
  int tucStatus;

  tucStatus = TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, property, &value);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to get property %d (0x%x)", property, tucStatus);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Set the camera's property value.
 *
 * @param property TUIDP property to set
 * @param value Value to set the property to
 *
 * @return asynStatus
 */
asynStatus ADTucam::setProperty(int property, double value) {
  static const char *functionName = "setProperty";
  int status = asynSuccess;
  int tucStatus;
  TUCAM_PROP_ATTR attrProp;

  attrProp.nIdxChn = 0;
  attrProp.idProp = property;
  tucStatus = TUCAM_Prop_GetAttr(camHandle_.hIdxTUCam, &attrProp);
  if (tucStatus == TUCAMRET_SUCCESS) {
    INFO_ARGS("property value range [%f %f]", attrProp.dbValMin,
              attrProp.dbValMax);
    if (value < attrProp.dbValMin) {
      value = attrProp.dbValMin;
      WARN_ARGS("Clipping set min value: %d, %f", property, value);
    } else if (value > attrProp.dbValMax) {
      value = attrProp.dbValMax;
      WARN_ARGS("Clipping set max value: %d, %f\n", property, value);
    }
  }
  INFO_ARGS("value %f", value);

  tucStatus = TUCAM_Prop_SetValue(camHandle_.hIdxTUCam, property, value);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to set property %d to %f (0x%x)", property, value,
             tucStatus);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Get the camera's capability value.
 *
 * @param property TUIDC capability to get
 * @param value Value to store the capability value
 *
 * @return asynStatus
 */
asynStatus ADTucam::getCapability(int property, int &val) {
  static const char *functionName = "getCapability";
  int status = asynSuccess;
  int tucStatus;

  tucStatus = TUCAM_Capa_GetValue(this->camHandle_.hIdxTUCam, property, &val);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to get capability %d=%d\n", property, val);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Set the camera's capability value.
 *
 * @param property TUIDC capability to set
 * @param value Value to set the capability to
 *
 * @return asynStatus
 */
asynStatus ADTucam::setCapability(int property, int val) {
  static const char *functionName = "setCapability";
  int status = asynSuccess;
  int tucStatus;

  tucStatus = TUCAM_Capa_SetValue(this->camHandle_.hIdxTUCam, property, val);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to set capability %d=%d (0x%x)\n", property, val,
             tucStatus);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Reads the capability of the camera.
 *
 * @param property TUIDC capability to read
 * @param strings Array of strings to be filled with capability names
 * @param values Array of integers to be filled with capability values
 * @param severities Array of integers to be filled with capability severities
 * @param nElements Number of elements in the arrays
 * @param nIn Number of elements filled in the arrays
 *
 * @returns asynSuccess if read was successful, asynError otherwise
 */
asynStatus ADTucam::getCapabilityText(int property, char *strings[],
                                     int values[], int severities[],
                                     size_t nElements, size_t *nIn) {
  static const char *functionName = "getCapabilityText";
  int status = asynSuccess;
  int tucStatus;
  int i = 0;

  TUCAM_CAPA_ATTR attrCapa;
  tucStatus = TUCAM_Capa_GetAttr(this->camHandle_.hIdxTUCam, &attrCapa);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to get capability %d (0x%x)\n", property, tucStatus);
    *nIn = 0;
    return asynError;
  }

  for (i = 0; i < attrCapa.nValMax - attrCapa.nValMin + 1; i++) {
    char szRes[64] = {0};
    TUCAM_VALUE_TEXT valText;
    valText.dbValue = i;
    valText.nID = property;
    valText.nTextSize = 64;
    valText.pText = &szRes[0];

    tucStatus = TUCAM_Capa_GetValueText(this->camHandle_.hIdxTUCam, &valText);

    if (tucStatus != TUCAMRET_SUCCESS) {
      ERR_ARGS("unable to get capability text %d:%d (0x%x)\n", property, i,
               tucStatus);
    }

    if (strings[i]) {
      free(strings[i]);
    }

    INFO_ARGS("Capability text: %s", valText.pText);

    strings[i] = epicsStrDup(valText.pText);
    values[i] = i + attrCapa.nValMin;
    severities[i] = 0;
  }

  *nIn = i;
  return (asynStatus)status;
}

/* Code for iocsh registration */
/* ADTucamConfig */
static const iocshArg ADTucamConfigArg0 = {"Port name", iocshArgString};
static const iocshArg ADTucamConfigArg1 = {"Camera Id", iocshArgInt};
static const iocshArg *const ADTucamConfigArgs[] = {&ADTucamConfigArg0,
                                                   &ADTucamConfigArg1};

static const iocshFuncDef configTucam = {"ADTucamConfig", 2, ADTucamConfigArgs};

static void configTucamCallFunc(const iocshArgBuf *args) {
  ADTucamConfig(args[0].sval, args[1].ival);
}

static void ADTucamRegister(void) {
  iocshRegister(&configTucam, configTucamCallFunc);
}

extern "C" {
epicsExportRegistrar(ADTucamRegister);
}
