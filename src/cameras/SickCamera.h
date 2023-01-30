#pragma once
#include "CameraWrapper.h"
#include "SickCamera/VisionaryControl.h"
#include "SickCamera/CoLaParameterReader.h"
#include "SickCamera/CoLaParameterWriter.h"
#include "SickCamera/VisionaryTMiniData.h"    // Header specific for the Time of Flight data
#include "SickCamera/VisionaryDataStream.h"
#include "SickCamera/VisionaryAutoIPScan.h"


class SickCamera : public CameraWrapper
{
public:
    struct PandiaSickDeviceInfo
    {
        std::string IpAddress;
        unsigned int IpPort = 2114u;
        bool busy = false;
    };
private:
    bool disconnect() override;
public:
    SickCamera(/* args */);
    ~SickCamera();

    static std::vector<std::string> getAvailableCameras(); //returns serial numbers of available cams
    static std::shared_ptr<SickCamera> CreateCameraPointer();
    bool connectBySerialNumber(const std::string &serialNumber);
    bool record(int n, int n_wait = 0) override;

    static std::vector<PandiaSickDeviceInfo> DeviceInfoList;
    std::shared_ptr<visionary::VisionaryTMiniData> pDataHandler;
    std::shared_ptr<visionary::VisionaryDataStream> dataStream;
    std::shared_ptr<visionary::VisionaryControl> visionaryControl;


};
