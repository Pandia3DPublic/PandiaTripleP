#pragma once
#include "CameraWrapper.h"

class EnsensoCamera : public CameraWrapper
{
private:
    static int CameraObjectCount; //for nxLibInitialize and nxLibFinalize, class instance count
    static bool IsNxLibInitialized;
    bool disconnect() override;
public:
    EnsensoCamera(/* args */);
    ~EnsensoCamera();

    static std::vector<std::string> getAvailableCameras(); //returns serial numbers of available cams
    static std::shared_ptr<EnsensoCamera> CreateCameraPointer();
    bool connectBySerialNumber(const std::string &serialNumber);
    bool record(int n, int n_wait = 0) override;
    bool combineRecordedFrames(Eigen::Vector3d beltDirection, double beltSpeed);

    template <typename T>
    bool setEnsensoSetting(const NxLibItem &settingNode, const std::string &itmString, T value);

    static NxLibItem EnsensoRoot;
    static NxLibItem EnsensoCameraList; //note: EnsensoCameraList.count() is always 2 more because of additional non camera nodes
    NxLibItem EnsensoCam;
    NxLibItem CaptureSettings;
    NxLibItem ProcessingSettings;


};
