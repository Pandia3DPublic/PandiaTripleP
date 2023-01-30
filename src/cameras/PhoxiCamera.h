#pragma once
#include "CameraWrapper.h"

class PhoxiCamera : public CameraWrapper
{
private:
    bool disconnect() override;
public:
    PhoxiCamera(/* args */);
    ~PhoxiCamera();

    static std::vector<std::string> getAvailableCameras(); //returns serial numbers of available cams
    static std::shared_ptr<PhoxiCamera> CreateCameraPointer();
    bool connectBySerialNumber(const std::string &serialNumber);
    bool record(int n, int n_wait = 0) override;

    static pho::api::PhoXiFactory PhoxiFactory;
    static std::vector<pho::api::PhoXiDeviceInformation> PhoxiDeviceList;
    pho::api::PPhoXi PhoxiDevice;


};

