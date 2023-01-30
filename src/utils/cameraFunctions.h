#pragma once
#include "cameras/CameraWrapper.h"
open3d::camera::PinholeCameraIntrinsic getScaledIntrinsic(const open3d::camera::PinholeCameraIntrinsic &intr, int newWidth, int newHeight);
CameraWrapper *getCameraBySerialNumber(std::shared_ptr<std::vector<CameraWrapper>> cameras, std::string SerialNumber);
bool SortbySerialNumber(const CameraWrapper &c1, const CameraWrapper &c2);
cv::Mat getUndistortionMap(const cv::Mat DistortionMap);