#pragma once
#include "CameraWrapper.h"

class ZividCamera : public CameraWrapper
{
private:
    bool disconnect() override;

public:
    ZividCamera(/* args */);
    ~ZividCamera();

    static std::vector<std::string> getAvailableCameras(); // returns serial numbers of available cams
    static std::shared_ptr<ZividCamera> CreateCameraPointer();
    bool connectBySerialNumber(const std::string &serialNumber);
    bool record(int n, int n_wait = 0) override;

    static Zivid::Application ZividApp;
    static std::vector<Zivid::Camera> ZividCameraList; // set in getNumberofConnectedDevices
    Zivid::Camera ZividCam;
    Zivid::Settings ZividSettings;
    Zivid::Array2D<Zivid::PointXYZ> Zivid2DPoints;

};
