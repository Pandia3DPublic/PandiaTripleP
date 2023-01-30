#pragma once
#include "CameraWrapper.h"

class KinectCamera : public CameraWrapper
{
private:
    static std::vector<int> IndicesConnected;
    static std::vector<std::string> SerialNumbersConnected; //note: indices and serial vectors must be same size todo std::pair
public:
    KinectCamera(/* args */);
    ~KinectCamera();

    static std::shared_ptr<KinectCamera> CreateCameraPointer();
    static std::vector<std::string> getAvailableCameras();

    bool connectBySerialNumber(const std::string &serialNumber);
    bool record(int n, int n_wait = 0) override;
    bool disconnect() override;

    k4a_device_configuration_t KinectConfig;
    k4a::device KinectDevice;
    k4a::calibration KinectCalibration;
    k4a::transformation KinectTransform;

    //############## overriden base class functions ################

};


