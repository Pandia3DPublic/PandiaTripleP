#pragma once
//std lib
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <list>
#include <fstream>
#include <cmath>
#include <dirent.h> // for file handling
//3rparty
#include <open3d/Open3D.h>
#include "open3d/pipelines/registration/GlobalOptimization.h"
#include "open3d/core/EigenConverter.h"
#include <json/json.h> //jsoncpp
#include <pybind11/pybind11.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED //needed for phoxi
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
//camera
#include <k4a/k4a.hpp> //Azure Kinect C++ wrapper
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Zivid.h>
#include "nxLib.h" //ensenso
#define PHOXI_OPENCV_SUPPORT
#define PHO_IGNORE_CV_VERSION_RESTRICTION
#include "PhoXi.h" //Phoxi Scanner
//custom files
#include "utils/PandiaTimer.h"

