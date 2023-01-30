#pragma once
#include "../cameras/CameraWrapper.h"

class CalibrationSingleton
{
public:
    CalibrationSingleton();
    ~CalibrationSingleton();
    //########## Basic Variables #############
    float SquareLength = 0.0485; //in m
    int Pattern_w = 9; //Calibration pattern, both width and height need to be set
    int Pattern_h = 6;
    std::vector<double> StereoErrors; //error values in stereoCalibrate to get avg

    cv::Mat getImagewithCorners(std::shared_ptr<CameraWrapper> cam);
    bool DetectandSaveCorners(std::vector<std::shared_ptr<CameraWrapper>> &cameras, bool fastEnd);
    bool DetectandSaveCorners(std::vector<std::shared_ptr<CameraWrapper>> &cameras, std::vector<std::shared_ptr<CameraWrapper>> &badCams, bool fastEnd);
    void CalibrateCameras(std::vector<std::shared_ptr<CameraWrapper>> &cameras);
    Eigen::Matrix4d getTransformBetween(std::shared_ptr<CameraWrapper> sourceCam, std::shared_ptr<CameraWrapper> targetCam);
    void AdjustCameraExposure(std::vector<std::shared_ptr<CameraWrapper>> &cameras);
    void ResetCamerasToAutoExposure(std::vector<std::shared_ptr<CameraWrapper>> &cameras);
    void SetCameraEmitterOnOff(std::vector<std::shared_ptr<CameraWrapper>> &cameras, bool setOn);
    void setGravityVectors(std::vector<std::shared_ptr<CameraWrapper>> &cameras);

};
