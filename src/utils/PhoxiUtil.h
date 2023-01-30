#pragma once
#include "PhoXi.h"

//Print out list of device info to standard output
void printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList);
//Print out device info to standard output
void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo);
//Print out frame info to standard output
void printFrameInfo(const pho::api::PFrame &Frame);
//Print out frame data to standard output
void printFrameData(const pho::api::PFrame &Frame);

// more print stuff
void PrintCapturingSettings(const pho::api::PhoXiCapturingSettings &CapturingSettings);
void PrintProcessingSettings(const pho::api::PhoXiProcessingSettings &ProcessingSettings);
void PrintCoordinatesSettings(const pho::api::PhoXiCoordinatesSettings &CoordinatesSettings);
void PrintCalibrationSettings(const pho::api::PhoXiCalibrationSettings &CalibrationSettings);
void PrintVector(const std::string &name, const pho::api::Point3_64f &vector);
void PrintMatrix(const std::string &name, const pho::api::CameraMatrix64f &matrix);
void printProfilesList(const std::vector<pho::api::PhoXiProfileDescriptor> &ProfilesList);