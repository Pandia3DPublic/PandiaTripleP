#include "IntelCamera.h"
#include "utils/conversions.h"
#include "utils/imageFunctions.h"
#include "utils/vishelper.h"
using namespace std;

rs2::context IntelCamera::IntelContext;

IntelCamera::IntelCamera(/* args */)
{
    CameraType = CameraTypes::Intel;
}

IntelCamera::~IntelCamera()
{
    disconnect();
}

std::vector<std::string> IntelCamera::getAvailableCameras()
{
    std::vector<std::string> availableCams;
    try
    {
        auto devices = IntelCamera::IntelContext.query_devices();
        for (int i = 0; i < devices.size(); i++)
        {
            availableCams.push_back(devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in getNumberofConnectedDevices: " << e.what() << '\n';
    }
    return availableCams;
}

bool IntelCamera::connectBySerialNumber(const std::string &serialNumber)
{
    if (Connected)
    {
        cout << "Info: Camera is already connected" << endl;
        return true;
    }
    rs2::log_to_console(rs2_log_severity::RS2_LOG_SEVERITY_ERROR); // RS2_LOG_SEVERITY_WARN
    try
    {
        auto devices = IntelContext.query_devices();
        int camIndex = -1;
        for (int i = 0; i < devices.size(); i++)
        {
            if (devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serialNumber)
            {
                camIndex = i;
                break;
            }
        }
        if (camIndex == -1)
        {
            cout << "Did not find camera with serialnumber " << serialNumber << endl;
            return false;
        }
        IntelDevice = devices[camIndex];
        SerialNumber = IntelDevice.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        // set sensor and its start options
        for (auto s : IntelDevice.query_sensors())
        {
            if (std::string(s.get_info(RS2_CAMERA_INFO_NAME)) == "RGB Camera")
            {
                IntelRGBSensor = s;
                setIntelSensorOption(IntelRGBSensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, true);
            }
            if (std::string(s.get_info(RS2_CAMERA_INFO_NAME)) == "Stereo Module")
            {
                IntelDepthSensor = s;
                setIntelSensorOption(IntelDepthSensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, true);
                setIntelSensorOption(IntelDepthSensor, RS2_OPTION_EMITTER_ENABLED, true);
            }
        }
        int width = 1280;
        int height = 720;
        int fps = 30;
        IntelRGBProfile = getIntelStreamProfile(IntelRGBSensor, width, height, fps, RS2_FORMAT_RGB8, "Color");
        IntelDepthProfiles.push_back(getIntelStreamProfile(IntelDepthSensor, width, height, fps, RS2_FORMAT_Z16, "Depth"));
        IntelDepthProfiles.push_back(getIntelStreamProfile(IntelDepthSensor, width, height, fps, RS2_FORMAT_Y8, "Infrared 1")); // one should denote left ir cam
        cout << "Intel device with id " << SerialNumber << " connected!" << endl;
        Connected = true;
    }
    catch (rs2::error &e)
    {
        cout << "Error: Connect failed: " << e.what() << endl;
        return false;
    }

    try
    {
        Align_to_depth = make_shared<rs2::align>(RS2_STREAM_DEPTH);
        // no undistortion for intel camera
        auto ColorIntr = IntelRGBProfile.as<rs2::video_stream_profile>().get_intrinsics();
        ColorLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(ColorIntr.width, ColorIntr.height, ColorIntr.fx, ColorIntr.fy, ColorIntr.ppx, ColorIntr.ppy);
        auto DepthIntr = IntelDepthProfiles[0].as<rs2::video_stream_profile>().get_intrinsics();
        DepthLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(DepthIntr.width, DepthIntr.height, DepthIntr.fx, DepthIntr.fy, DepthIntr.ppx, DepthIntr.ppy);
        auto InfraredIntr = IntelDepthProfiles[1].as<rs2::video_stream_profile>().get_intrinsics();
        CalibrationIntrinsic = open3d::camera::PinholeCameraIntrinsic(InfraredIntr.width, InfraredIntr.height, InfraredIntr.fx, InfraredIntr.fy, InfraredIntr.ppx, InfraredIntr.ppy);
    }
    catch(rs2::error &e)
    {
        cout << "Error: Connect failed (Calibration): " << e.what() << endl;
        return false;
    }
    
  
    return true;
}

bool IntelCamera::record(int n, int n_wait)
{
    return recordWithOption(n, n_wait, true);
}

bool IntelCamera::recordWithoutChecks(int n, int n_wait)
{
    return recordWithOption(n, n_wait, false);
}

bool IntelCamera::recordWithOption(int n, int n_wait, bool checkDepthImage)
{
    int nFramesOld = rgbdImages.size();
    startStreaming(50);
    int nReset = 0;
    int nResetMax = 3;
    for (int i = 0; i < n + nReset; i++)
    {
        try
        {
            PandiaTimer framesetTimer;
            bool validFrameset = false;
            rs2::frameset fs;
            while (!validFrameset && framesetTimer.seconds() < 5)
            {
                fs = IntelSyncer.wait_for_frames(); // does not copy, only adds a reference
                validFrameset = isIntelFramesetValid(fs);
            }
            if (!validFrameset)
                cout << "Warning: Could not get a valid frameset for camera " << SerialNumber << endl;
            //############ processing ###################
            //Note: Processsing must happen here due to stupid intel pipeline memory management
            //need to take color here since align to depth changes color
            auto color_frame_full = fs.get_color_frame();
            cv::Mat cvColorFR = FrametoMat(color_frame_full);
            auto infrared_frame = fs.get_infrared_frame(1); //left infrared image
            cv::Mat cvCalibrationImage = FrametoMat(infrared_frame).clone();

            // align color to depth
            auto alignedFS = Align_to_depth->process(fs);
            auto color_frame_aligned = alignedFS.get_color_frame();
            cv::Mat cvColorAligned = FrametoMat(color_frame_aligned);
            auto depth_frame = alignedFS.get_depth_frame();

            //############## apply filters, order is important ####################
            depth_frame = Thr_filter.process(depth_frame);
            depth_frame = Depth_to_disparity.process(depth_frame);
            depth_frame = Spat_filter.process(depth_frame); //spatial filter does hole filling, which we should not do!
            depth_frame = Disparity_to_depth.process(depth_frame);
            cv::Mat cvDepth = FrametoMatMillimeters(depth_frame);

            // ############## realsense pcd #####################
            // note: this uses full resolution and unaligned color and depth images
            rs2::pointcloud pcd;
            pcd.map_to(color_frame_full);
            rs2::points points = pcd.calculate(depth_frame);
            auto vertices = points.get_vertices();
            auto tex_coords = points.get_texture_coordinates(); // note: tex coords should be between [0,1] however <0 or >1 is possible since projection might suffer from distortions
            auto o3dpcd = make_shared<open3d::geometry::PointCloud>();
            for (int i = 0; i < points.size(); i++)
            {
                if (vertices[i].z)
                {
                    Eigen::Vector3d p = Eigen::Vector3d(vertices[i].x, vertices[i].y, vertices[i].z);
                    o3dpcd->points_.push_back(p);
                    double u = tex_coords[i].u;
                    double v = tex_coords[i].v;
                    // force invalid tex coords to be in range [0,1]
                    if (u > 1)
                        u = 1;
                    if (v > 1)
                        v = 1;
                    if (u < 0)
                        u = 0;
                    if (v < 0)
                        v = 0;
                    auto cc = cvColorFR.at<cv::Vec3b>(v * (cvColorFR.rows - 1), u * (cvColorFR.cols - 1));
                    Eigen::Vector3d c = Eigen::Vector3d(cc(0), cc(1), cc(2));
                    c /= 255;
                    o3dpcd->colors_.push_back(c);
                }
            }

            //############### convert to open3d ############################
            std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
            rgbdImg->color_ = *OpenCVToOpen3D(cvColorAligned);
            rgbdImg->depth_ = *OpenCVToOpen3D(cvDepth);
            rgbdImg->depth_ = *rgbdImg->depth_.ConvertDepthToFloatImage(1000,100);
            //################ check depth image ############################
            if (checkDepthImage)
            {
                if (isDepthImageValid(rgbdImg->depth_))
                {
                    rgbdImages.push_back(rgbdImg);
                    Pcds.push_back(o3dpcd);
                    FRCalibrationImages.push_back(cvCalibrationImage);
                }
                else
                {
                    // open3d::visualization::DrawGeometries({make_shared<open3d::geometry::Image>(rgbdImg->depth_)});
                    cout << "Warning: Depth frame of camera " << SerialNumber << " is invalid!" << endl;
                    if (nReset < nResetMax)
                    {
                        cout << "Resetting camera then recording again..." << endl;
                        resetConnection(true);
                        nReset++;
                    }
                }
            }
            else
            {
                rgbdImages.push_back(rgbdImg);
                Pcds.push_back(o3dpcd);
                FRCalibrationImages.push_back(cvCalibrationImage);
            }
        }
        catch (const rs2::error &e)
        {
            std::cerr << e.what() << '\n';
            cout << "Error: Recording or processing frames " << i << " of cam " << SerialNumber << " failed." << endl;
            if (nReset < nResetMax)
            {
                cout << "Resetting camera then recording again..." << endl;
                resetConnection(true);
                nReset++;
            }
        }
    }
    stopStreaming();
    ApproxVoxelSize = 1/std::sqrt(DepthLDTIntrinsic.width_*DepthLDTIntrinsic.height_);

    return rgbdImages.size() > nFramesOld;
}

void IntelCamera::startStreaming(int n_warmup)
{
    if (!IsStreaming)
    {
        try
        {
            IntelRGBSensor.open(IntelRGBProfile);
            IntelDepthSensor.open(IntelDepthProfiles);
            IntelRGBSensor.start(IntelSyncer);
            IntelDepthSensor.start(IntelSyncer);
            IsStreaming = true;

            // note: warmup frames seem to be more stable than sleep as it can sometimes take longer to get a frame.
            // std::this_thread::sleep_for(2s);
            for (int i = 0; i < n_warmup; i++) // 30 frames warmup for autoexposure and stream stability
            {
                try
                {
                    rs2::frameset warumUpCapture = IntelSyncer.wait_for_frames();
                }
                catch (const rs2::error &e)
                {
                    std::cerr << e.what() << '\n';
                    cout << "Error: Getting warmup frame " << i << " in startStreaming failed!" << endl;
                }
            }
        }
        catch (rs2::error &e)
        {
            cerr << e.what() << endl;
            cout << "Error: Intel sensor start failed!" << endl;
            return;
        }
    }
}

void IntelCamera::stopStreaming()
{
    if (IsStreaming)
    {
        try
        {
            IntelRGBSensor.stop();
            IntelDepthSensor.stop();
            IntelRGBSensor.close();
            IntelDepthSensor.close();
            IsStreaming = false;
        }
        catch (rs2::error &e)
        {
            cerr << e.what() << endl;
            cout << "Error: Intel sensor stop failed!" << endl;
        }
    }
}

bool IntelCamera::resetConnection(bool streamAfterReset /* = true*/)
{
    bool resetted = false;
    PandiaTimer timeout;
    int t_max = 15;
    while (!resetted && timeout.seconds() < t_max)
    {
        try
        {
            cout << "Resetting camera " << SerialNumber << endl;
            stopStreaming();
            // intel realsense sdk doensn't have a reliable method to check if a camera is ready again after a hardware_reset
            // we should therefore wait a painful 10s before attemting to start the sensors again
            // see https://github.com/IntelRealSense/librealsense/issues/9287#issuecomment-872606987
            IntelDevice.hardware_reset(); //reset usb connection
            int n_wait = 0;
            while (n_wait++ < 10)
            {
                cout << "waiting for usb reconnect (" << n_wait << "s / 10s)" << endl;
                std::this_thread::sleep_for(1s);
            }
            rs2::device_hub IntelHub(IntelContext);
            while (!resetted && timeout.seconds() < t_max)
            {
                cout << "searching device..." << endl;
                rs2::device device = IntelHub.wait_for_device(); //Note that device hub will get any device, if you have more than 1 connected it could return the other device
                if (std::string(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) == SerialNumber)
                {
                    cout << "found device!" << endl;
                    // hardware reset should invalidate device handles but sensors apparantly still work after????
                    // device and sensors are all shared_ptr so setting them again probably does nothing as reference count
                    // is always > 0 while CameraWrapper instance is alive

                    IntelDevice = device;
                    resetted = true;
                    break;
                }
            }
            if (streamAfterReset)
            {
                startStreaming(n_wait);
            }
            cout << "Done resetting camera " << SerialNumber << endl;
            break;
        }
        catch (rs2::error &e)
        {
            cerr << e.what() << endl;
            cout << "Reset connection failed!" << endl;
            this_thread::sleep_for(1000ms);
        }
    }
    return resetted;
}

void IntelCamera::printAllIntelSensorOptions()
{
    for (auto &&s : IntelDevice.query_sensors())
    {
        cout << s.get_info(RS2_CAMERA_INFO_NAME) << " Sensor options: " << endl;
        auto options = s.get_supported_options();
        for (auto o : options)
        {
            if (s.supports(o))
            {
                auto optRange = s.get_option_range(o);
                cout << s.get_option_name(o) << "  #  ";
                cout << "default " << optRange.def << " | ";
                cout << "min " << optRange.min << " | ";
                cout << "max " << optRange.max << " | ";
                cout << "step " << optRange.step << endl;
            }
        }
        cout << endl;
    }
}

void IntelCamera::printIntelSensorOption(const rs2::sensor &sensor, rs2_option option)
{
    if (!sensor.supports(option))
    {
        cout << "Error: Intel Sensor option is not supported!" << endl;
        return;
    }
    auto optRange = sensor.get_option_range(option);
    cout << sensor.get_option_name(option) << "  #  ";
    cout << "default " << optRange.def << " | ";
    cout << "min " << optRange.min << " | ";
    cout << "max " << optRange.max << " | ";
    cout << "step " << optRange.step << endl;
}

void IntelCamera::setIntelSensorOption(const rs2::sensor &sensor, rs2_option option, float value)
{
    if (!sensor.supports(option))
    {
        cout << "This option is not supported by this sensor" << std::endl;
        return;
    }

    try
    {
        sensor.set_option(option, value);
    }
    catch (const rs2::error &e)
    {
        // Some options can only be set while the camera is streaming,
        // and generally the hardware might fail so it is good practice to catch exceptions from set_option
        std::cout << "Failed to set option " << option << ". (" << e.what() << ")" << std::endl;
    }
}

// call rs-enumerate-devices for info of available streams
rs2::stream_profile IntelCamera::getIntelStreamProfile(const rs2::sensor &s, int w, int h, int fps, rs2_format format, std::string stream_name)
{
    for (auto p : s.get_stream_profiles())
    {
        // cout << p.as<rs2::video_stream_profile>().width() << endl;
        // cout << p.as<rs2::video_stream_profile>().height() << endl;
        // cout << p.fps() << endl;
        // cout << p.format() << endl;
        // cout << p.as<rs2::video_stream_profile>().stream_name() << endl;
        // cout << "###########" << endl;
        if (p.as<rs2::video_stream_profile>().width() == w &&
            p.as<rs2::video_stream_profile>().height() == h &&
            p.fps() == fps &&
            p.format() == format &&
            p.as<rs2::video_stream_profile>().stream_name() == stream_name)
        {
            return p;
        }
    }
    cout << "Warning: Get Intel Stream Profile failed!" << endl;
    return rs2::stream_profile();
}

// for frameset with color, depth and infrared frame
bool IntelCamera::isIntelFramesetValid(const rs2::frameset &fs)
{
    bool valid = true;
    if (!fs.get_color_frame())
    {
        // cout << "Warning: Color frame is invalid!" << endl;
        valid = false;
    }
    if (!fs.get_depth_frame())
    {
        // cout << "Warning: Depth frame is invalid!" << endl;
        valid = false;
    }
    if (!fs.get_infrared_frame())
    {
        // cout << "Warning: Infrared frame is invalid!" << endl;
        valid = false;
    }
    return valid;
}

std::shared_ptr<IntelCamera> IntelCamera::CreateCameraPointer()
{
    return make_shared<IntelCamera>();
}

bool IntelCamera::disconnect()
{
    try
    {
        stopStreaming();
        Connected = false;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return !Connected;
}

