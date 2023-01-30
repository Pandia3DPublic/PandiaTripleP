#pragma once
#include "CameraWrapper.h"

class IntelCamera : public CameraWrapper
{
private:
    bool recordWithOption(int n, int n_wait, bool checkDepthImage); 
    /* data */
public:
    IntelCamera(/* args */);
    ~IntelCamera();

    static std::vector<std::string> getAvailableCameras(); //returns serial numbers of available cams
    static std::shared_ptr<IntelCamera> CreateCameraPointer();
    bool connectBySerialNumber(const std::string &serialNumber);
    bool record(int n, int n_wait = 0) override;
    bool recordWithoutChecks(int n, int n_wait = 0);
    bool disconnect() override;

    void startStreaming(int n_warmup = 0);
    void stopStreaming();
    bool resetConnection(bool streamAfterReset = true);

    void printAllIntelSensorOptions();
    void printIntelSensorOption(const rs2::sensor &sensor, rs2_option option);
    void setIntelSensorOption(const rs2::sensor &sensor, rs2_option option, float value);
    rs2::stream_profile getIntelStreamProfile(const rs2::sensor &s, int w, int h, int fps, rs2_format format, std::string stream_name);
    bool isIntelFramesetValid(const rs2::frameset &fs);

    bool IsStreaming = false; // true if the camera is streaming data to the host device, is set in start/stopStreaming
    static rs2::context IntelContext; // for multicam
    rs2::device IntelDevice;
    rs2::sensor IntelRGBSensor;
    rs2::sensor IntelDepthSensor; // intel stereo module which includes depth and infrared streams
    rs2::stream_profile IntelRGBProfile;
    std::vector<rs2::stream_profile> IntelDepthProfiles; // first element is depth, second infrared profile
    rs2::syncer IntelSyncer = rs2::syncer(10);           //to get synced rgb and depth data
    std::shared_ptr<rs2::align> Align_to_depth;

    // Intel filters, see https://dev.intelrealsense.com/docs/post-processing-filters and https://dev.intelrealsense.com/docs/depth-post-processing
    rs2::disparity_transform Depth_to_disparity = rs2::disparity_transform(true);  // transforms to disparity space which is 1/Distance
    rs2::disparity_transform Disparity_to_depth = rs2::disparity_transform(false); // back to normal
    rs2::decimation_filter Dec_filter;                                             // Decimation - reduces depth frame density
    rs2::threshold_filter Thr_filter;                                              // Threshold  - removes values outside recommended range
    rs2::spatial_filter Spat_filter;                                               // spatial filter



};

