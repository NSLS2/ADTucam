/**
 * ADTucam.h
 *
 * ADTucam EPICS areaDetector driver header file
 *
 * Author: Jakub Wlodek, Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */

// Include guard
#ifndef TUCAMAPP_TUCAMSRC_ADTUCAM_H_
#define TUCAMAPP_TUCAMSRC_ADTUCAM_H_

// Driver version numbers
#define ADTUCAM_VERSION 0
#define ADTUCAM_REVISION 0
#define ADTUCAM_MODIFICATION 1

// Standard Includes
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// EPICS includes
#include <cantProceed.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <unordered_set>

#include "ADDriver.h"

// TUCAM include
#include "TUCamApi.h"
#include "TUDefine.h"

// Camera SDK interface
#include "ICameraSDK.h"

// Camera property strings
#define ADTucam_CaptureString "TUCAM_CAPTURE"
#define ADTucam_TemperatureSetpointString "TUCAM_TEMP_SETPOINT"
#define ADTucam_TemperatureString "TUCAM_TEMP"
#define ADTucam_TemperatureEmergencySignalString "TUCAM_TEMP_EMERGENCY"
#define ADTucam_AutoTECThresholdString "TUCAM_AUTO_TEC_THRESH"
#define ADTucam_AutoTECString "TUCAM_AUTO_TEC"
#define ADTucam_TECStatusString "TUCAM_TEC_STATUS"
#define ADTucam_GainModeString "TUCAM_GAIN_MODE"
#define ADTucam_RetryOnTimeoutString "TUCAM_RETRY_ON_TIMEOUT"
#define ADTucam_NumRetriesString "TUCAM_NUM_RETRIES"
#define ADTucam_BusString "TUCAM_BUS"
#define ADTucam_ProductIDString "TUCAM_PRODUCT_ID"
#define ADTucam_TransferRateString "TUCAM_TRANSFER_RATE"
#define ADTucam_FrameSpeedString "TUCAM_FRAME_SPEED"
#define ADTucam_BitDepthString "TUCAM_BIT_DEPTH"
#define ADTucam_FrameFormatString "TUCAM_FRAME_FORMAT"
#define ADTucam_BinModeString "TUCAM_BIN_MODE"
#define ADTucam_ImageModeString "TUCAM_IMG_MODE"
#define ADTucam_AutoExposureString "TUCAM_AUTO_EXPOSURE"
#define ADTucam_FanGearString "TUCAM_FAN_GEAR"
#define ADTucam_AutoLevelsString "TUCAM_AUTO_LEVELS"
#define ADTucam_HistogramString "TUCAM_HISTOGRAM"
#define ADTucam_EnhanceString "TUCAM_ENHANCE"
#define ADTucam_DefectCorrString "TUCAM_DEFECT_CORR"
#define ADTucam_DenoiseString "TUCAM_ENABLE_DENOISE"
#define ADTucam_FlatCorrString "TUCAM_FLAT_CORR"
#define ADTucam_DynRgeCorrString "TUCAM_DYN_RGE_CORR"
#define ADTucam_BrightnessString "TUCAM_BRIGHTNESS"
#define ADTucam_BlackLevelString "TUCAM_BLACK_LEVEL"
#define ADTucam_SharpnessString "TUCAM_SHARPNESS"
#define ADTucam_NoiseLevelString "TUCAM_NOISE_LEVEL"
#define ADTucam_HDRKString "TUCAM_HDRK"
#define ADTucam_GammaString "TUCAM_GAMMA"
#define ADTucam_ContrastString "TUCAM_CONTRAST"
#define ADTucam_LeftLevelString "TUCAM_LEFT_LEVELS"
#define ADTucam_RightLevelString "TUCAM_RIGHT_LEVELS"
#define ADTucam_TriggerEdgeString "TUCAM_TRIG_EDGE"
#define ADTucam_TriggerExposureString "TUCAM_TRIG_EXP"
#define ADTucam_TriggerDelayString "TUCAM_TRIG_DLY"
#define ADTucam_TriggerSoftwareString "TUCAM_TRIG_SOFT"
#define ADTucam_TriggerOut1ModeString "TUCAM_TRGOUT1_MODE"
#define ADTucam_TriggerOut1EdgeString "TUCAM_TRGOUT1_EDGE"
#define ADTucam_TriggerOut1DelayString "TUCAM_TRGOUT1_DLY"
#define ADTucam_TriggerOut1WidthString "TUCAM_TRGOUT1_WIDTH"
#define ADTucam_TriggerOut2ModeString "TUCAM_TRGOUT2_MODE"
#define ADTucam_TriggerOut2EdgeString "TUCAM_TRGOUT2_EDGE"
#define ADTucam_TriggerOut2DelayString "TUCAM_TRGOUT2_DLY"
#define ADTucam_TriggerOut2WidthString "TUCAM_TRGOUT2_WIDTH"
#define ADTucam_TriggerOut3ModeString "TUCAM_TRGOUT3_MODE"
#define ADTucam_TriggerOut3EdgeString "TUCAM_TRGOUT3_EDGE"
#define ADTucam_TriggerOut3DelayString "TUCAM_TRGOUT3_DLY"
#define ADTucam_TriggerOut3WidthString "TUCAM_TRGOUT3_WIDTH"
#define ADTucam_PRNUString "TUCAM_PRNU"
#define ADTucam_DataFormatString "TUCAM_DATA_FORMAT"
#define ADTucam_EnableGammaString "TUCAM_ENABLE_GAMMA"
#define ADTucam_EnableBlackLevelString "TUCAM_ENABLE_BLACK_LEVEL"

class ADTucam : public ADDriver {
 public:
#ifndef UNIT_TESTING
  ADTucam(const char *portName, int cameraId);
#endif
  ADTucam(const char *portName, int cameraId, ICameraSDK *sdkHandler);
  ~ADTucam();
  void monitorTemperatureThread();
  void acquisitionThread();
  void frameTimeoutThread();

  /* These are the methods that we override from ADDriver */
  virtual asynStatus readEnum(asynUser *pasynUser, char *strings[],
                              int values[], int severities[], size_t nElements,
                              size_t *nIn);
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

 protected:
  int ADTucam_Capture;
#define FIRST_TUCAM_PARAM ADTucam_Capture
  int ADTucam_TemperatureSetpoint;
  int ADTucam_Temperature;
  int ADTucam_TemperatureEmergencySignal;
  int ADTucam_AutoTECThreshold;
  int ADTucam_AutoTEC;
  int ADTucam_TECStatus;
  int ADTucam_GainMode;
  int ADTucam_RetryOnTimeout;
  int ADTucam_NumRetries;
  int ADTucam_Bus;
  int ADTucam_ProductID;
  int ADTucam_TransferRate;
  int ADTucam_FrameSpeed;
  int ADTucam_BitDepth;
  int ADTucam_BinMode;
  int ADTucam_FanGear;
  int ADTucam_ImageMode;
  int ADTucam_AutoExposure;
  int ADTucam_FrameFormat;
  int ADTucam_AutoLevels;
  int ADTucam_Histogram;
  int ADTucam_Enhance;
  int ADTucam_DefectCorr;
  int ADTucam_Denoise;
  int ADTucam_FlatCorr;
  int ADTucam_DynRgeCorr;
  int ADTucam_Brightness;
  int ADTucam_BlackLevel;
  int ADTucam_Sharpness;
  int ADTucam_NoiseLevel;
  int ADTucam_HDRK;
  int ADTucam_Gamma;
  int ADTucam_Contrast;
  int ADTucam_LeftLevel;
  int ADTucam_RightLevel;
  int ADTucam_TriggerEdge;
  int ADTucam_TriggerExposure;
  int ADTucam_TriggerDelay;
  int ADTucam_TriggerSoftware;
  int ADTucam_TriggerOut1Mode;
  int ADTucam_TriggerOut1Edge;
  int ADTucam_TriggerOut1Delay;
  int ADTucam_TriggerOut1Width;
  int ADTucam_TriggerOut2Mode;
  int ADTucam_TriggerOut2Edge;
  int ADTucam_TriggerOut2Delay;
  int ADTucam_TriggerOut2Width;
  int ADTucam_TriggerOut3Mode;
  int ADTucam_TriggerOut3Edge;
  int ADTucam_TriggerOut3Delay;
  int ADTucam_TriggerOut3Width;
  int ADTucam_PRNU;
  int ADTucam_DataFormat;
  int ADTucam_EnableGamma;
  int ADTucam_EnableBlackLevel;
#define LAST_TUCAM_PARAM ADTucam_EnableBlackLevel

 private:
  bool tecActive = false;
  /* Whether we should clean up the handler on exit */
  bool cleanupHandler_;

  void setup();
  void createParamLibrary();
  void setupPersistentThreads();
  void handleStopEvent();
  asynStatus handleTEC(double temperatureVal);
  asynStatus warnOnExtremeTemperature(double temperatureVal);
  asynStatus handleAcquisitionRequest(int value);
  asynStatus handleCaptureRequest(int value);
  asynStatus setAcquirePeriod(double period);

  asynStatus grabImage(epicsTimeStamp *startTimeStamp);
  asynStatus startAcquisition();
  asynStatus stopAcquisition();
  asynStatus startCapture();
  asynStatus stopCapture();

  asynStatus connectCamera();
  asynStatus disconnectCamera();

  asynStatus setCurrentTrigger();
  asynStatus setCameraTrigger(int triggerMode, int triggerEdge,
                              int triggerExposure, double triggerDelay);
  asynStatus setCurrentTriggerOut(int port);
  asynStatus setCameraTriggerOut(int port, int triggerMode, int triggerEdge,
                                 double triggerDelay, double triggerWidth);
  asynStatus setCurrentROI();
  asynStatus setCameraROI(int minX, int minY, int sizeX, int sizeY);

  /* camera property control functions */
  asynStatus getCamInfo(int nID, char *sBuf, int &val);
  asynStatus setCamInfo(int param, int nID, int dtype);
  asynStatus setSerialNumber();
  asynStatus getProperty(int nID, double *value);
  asynStatus setProperty(int nID, double value);
  asynStatus getCapability(int property, int *value);
  asynStatus setCapability(int property, int value);
  asynStatus getCapabilityText(int property, char *strings[], int values[],
                               int severities[], size_t nElements, size_t *nIn);

  /* Data */
  int cameraId_;
  TUCAM_INIT apiHandle_;
  TUCAM_OPEN camHandle_;
  TUCAM_FRAME frameHandle_;
  TUCAM_TRIGGER_ATTR triggerHandle_;
  TUCAM_TRGOUT_ATTR triggerOutHandle_[3];
  epicsEventId startEventId_;
  epicsEventId stopEventId_;
  epicsEventId startFrameTimeoutEventId_;
  epicsEventId stopFrameTimeoutEventId_;
  epicsThreadId acquisitionThreadId_;
  epicsThreadId monitorTemperatureThreadId_;
  epicsThreadId frameTimeoutThreadId_;
  NDArray *pRaw_;
  int triggerOutSupport_;
  ICameraSDK *sdkHandler_;
  volatile bool shutdownRequested_;
  bool initialized_;
  std::unordered_set<int> idleOnlyIntParams_;
};

#define NUM_TUCAM_PARAMS ((int)(&LAST_TUCAM_PARAM - &FIRST_TUCAM_PARAM + 1))

#endif  // TUCAMAPP_TUCAMSRC_ADTUCAM_H_
