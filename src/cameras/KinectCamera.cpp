#include "KinectCamera.h"
#include "utils/conversions.h"
#include "utils/cameraFunctions.h"

using namespace std;

std::vector<int> KinectCamera::IndicesConnected;
std::vector<std::string> KinectCamera::SerialNumbersConnected;

KinectCamera::KinectCamera(/* args */)
{
    CameraType = CameraTypes::Kinect;
}

KinectCamera::~KinectCamera()
{
    disconnect();
}

std::vector<std::string> KinectCamera::getAvailableCameras()
{
    std::vector<std::string> availableCams;
    try
    {
        // note: unfortunately we have to open the device in order to get the serial number
        int n_cams = k4a_device_get_installed_count();
        for (int i = 0; i < n_cams; i++)
        {
            if (std::find(KinectCamera::IndicesConnected.begin(), KinectCamera::IndicesConnected.end(), i) == KinectCamera::IndicesConnected.end()) // if camera is not already connected
            {
                k4a::device cam;
                cam = cam.open(i);
                availableCams.push_back(cam.get_serialnum());
                cam.close();
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in getAvailableCameras: " << e.what() << '\n';
    }
    return availableCams;
}

bool KinectCamera::connectBySerialNumber(const std::string &serialNumber)
{
    if (Connected)
    {
        cout << "Info: Camera is already connected" << endl;
        return true;
    }
    try
    {
        int n_cams = k4a_device_get_installed_count();
        int cameraIndex = -1;
        for (int i = 0; i < n_cams; i++)
        {
            if (std::find(KinectCamera::IndicesConnected.begin(), KinectCamera::IndicesConnected.end(), i) == KinectCamera::IndicesConnected.end()) // if camera is not already connected
            {
                k4a::device cam;
                cam = cam.open(i);
                if (serialNumber == cam.get_serialnum())
                {
                    cameraIndex = i;
                }
                cam.close();
            }
        }
        if (cameraIndex == -1) // serial number not found
        {
            return false;
        }
        KinectDevice = KinectDevice.open(cameraIndex); // throws exception if there is no device
        SerialNumber = KinectDevice.get_serialnum();
        cout << "Azure Kinect device with id " << SerialNumber << " opened!" << endl;
        IndicesConnected.push_back(cameraIndex);
        SerialNumbersConnected.push_back(SerialNumber);
        Connected = true;
    }
    catch (k4a::error e)
    {
        cout << "Error in connect: " << e.what() << endl;
        return false;
    }
    // todo check if this config is the best
    KinectConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    KinectConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;  // note this takes processing power since some stupid mjpeg format is native.
    KinectConfig.color_resolution = K4A_COLOR_RESOLUTION_1536P; // res is 2048x1536, this is the smallest 4:3 resolution which has max overlap with depth
    // KinectConfig.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;    // binned res is 320x288
    KinectConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;     //res is 640x576 for K4A_DEPTH_MODE_NFOV_UNBINNED
    KinectConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    KinectConfig.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    KinectConfig.synchronized_images_only = true;

    KinectDevice.start_cameras(&KinectConfig);
    KinectCalibration = KinectDevice.get_calibration(KinectConfig.depth_mode, KinectConfig.color_resolution);
    KinectTransform = k4a::transformation(KinectCalibration); // this takes some time
    // get basic ldt intrinsic
    auto &ColorCalib = KinectCalibration.color_camera_calibration;
    auto &ColorParams = ColorCalib.intrinsics.parameters.param;
    ColorLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(ColorCalib.resolution_width, ColorCalib.resolution_height,
                                                               ColorParams.fx, ColorParams.fy, ColorParams.cx, ColorParams.cy);
    CalibrationIntrinsic = ColorLDTIntrinsic;
    auto &DepthCalib = KinectCalibration.depth_camera_calibration;
    auto &DepthParams = DepthCalib.intrinsics.parameters.param;
    DepthLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(DepthCalib.resolution_width, DepthCalib.resolution_height,
                                                               DepthParams.fx, DepthParams.fy, DepthParams.cx, DepthParams.cy);

    BrownianCalibrationCamDistortionCoeffs = {ColorParams.k1, ColorParams.k2, ColorParams.p1, ColorParams.p2, ColorParams.k3, ColorParams.k4, ColorParams.k5, ColorParams.k6};
    ApproxVoxelSize = 1/std::sqrt(DepthLDTIntrinsic.width_*DepthLDTIntrinsic.height_);

    return true;
}

bool KinectCamera::record(int n, int n_wait)
{
    int nFramesOld = rgbdImages.size();
    k4a::capture warmupCapture;
    for (int i = 0; i < n_wait; i++)
    {
        KinectDevice.get_capture(&warmupCapture); // dropping several frames for auto-exposure
    }
    int nfail = 0;
    int nfailMax = 5;
    for (int i = 0; i < n + nfail; i++)
    {
        auto c = make_shared<k4a::capture>();
        try
        {
            KinectDevice.get_capture(c.get());
        }
        catch (k4a::error e)
        {
            cerr << e.what() << endl;
            nfail++;
            if (nfail >= nfailMax)
            {
                return false;
            }
            continue;
        }

        k4a::image color = c->get_color_image();
        k4a::image depth = c->get_depth_image();
        k4a::image infrared = c->get_ir_image();
        FRCalibrationImages.push_back(KinectImageToOpenCV(color));
        color = KinectTransform.color_image_to_depth_camera(depth, color);
        auto pcdimg = KinectTransform.depth_image_to_point_cloud(depth, K4A_CALIBRATION_TYPE_DEPTH);
        auto o3dpcd = KinectPCDToOpen3D(pcdimg, &color);
        Pcds.push_back(o3dpcd);

        // rgbd open3d
        // std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
        // rgbdImg->color_ = *KinectImageToOpen3D(color);
        // rgbdImg->depth_ = *getDepthImageFromPCD(*o3dpcd);
        // rgbdImages.push_back(rgbdImg);
        rgbdImages.push_back(getRGBDImageFromPCD(*o3dpcd));
    }
    return rgbdImages.size() > nFramesOld ? true : false;
}

std::shared_ptr<KinectCamera> KinectCamera::CreateCameraPointer()
{
    return make_shared<KinectCamera>();
}

bool KinectCamera::disconnect()
{
    try
    {
        auto it = std::find(SerialNumbersConnected.begin(), SerialNumbersConnected.end(), SerialNumber);
        if (it != SerialNumbersConnected.end()) // if serial number found
        {
            if (Connected)
            {
                int index = it - SerialNumbersConnected.begin();
                SerialNumbersConnected.erase(it);
                IndicesConnected.erase(IndicesConnected.begin() + index);
                KinectDevice.close();
                Connected = false;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return !Connected;
}
