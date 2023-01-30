#include "EnsensoCamera.h"
#include "utils/cameraFunctions.h"
#include "utils/conversions.h"
#include "utils/jsonUtil.h"
#include "utils/vishelper.h"
using namespace std;

NxLibItem EnsensoCamera::EnsensoRoot;
NxLibItem EnsensoCamera::EnsensoCameraList;
int EnsensoCamera::CameraObjectCount = 0;
bool EnsensoCamera::IsNxLibInitialized = false;

EnsensoCamera::EnsensoCamera(/* args */)
{
    CameraType = CameraTypes::Ensenso;
    EnsensoCamera::CameraObjectCount++;
    if (!EnsensoCamera::IsNxLibInitialized)
    {
        cout << "Opening NxLib and waiting for cameras to be detected\n";
        try
        {
            nxLibInitialize(true);
            EnsensoCamera::IsNxLibInitialized = true;
            EnsensoCamera::EnsensoRoot = NxLibItem();
            EnsensoCamera::EnsensoCameraList = NxLibItem("/Cameras");
        }
        catch (NxLibException &ex)
        {
            cout << "Error in ensenso camera constructor " << ex.getItemPath() << " " << ex.getErrorText() << endl;
        }
    }
}

EnsensoCamera::~EnsensoCamera()
{
    disconnect();
    EnsensoCamera::CameraObjectCount--;
    if (EnsensoCamera::IsNxLibInitialized && EnsensoCamera::CameraObjectCount <= 0)
    {
        try
        {
            printf("Closing NxLib\n");
            nxLibFinalize(); //if nxLibInitialize() was called, nxLibFinalize() should be called before program exit, otherwise we segfault
            EnsensoCamera::IsNxLibInitialized = false;
        }
        catch (NxLibException &ex)
        {
            cout << "Error in ensenso camera deconstructor " << ex.getItemPath() << " " << ex.getErrorText() << endl;
        }
    }
}

std::vector<std::string> EnsensoCamera::getAvailableCameras()
{
    std::vector<std::string> availableCams;
    try
    {
        if (!EnsensoCamera::IsNxLibInitialized)
        {
            cout << "Opening NxLib and waiting for cameras to be detected\n";
            nxLibInitialize(true);
            EnsensoCamera::IsNxLibInitialized = true;
            EnsensoCamera::EnsensoRoot = NxLibItem();
            EnsensoCamera::EnsensoCameraList = NxLibItem("/Cameras");
        }
        int camCount = EnsensoCamera::EnsensoCameraList.count(); // note: there are two additional non direct camera nodes attached to /Cameras/
        for (int i = 0; i < camCount; i++)
        {
            auto cam = EnsensoCamera::EnsensoCameraList[i];
            if (cam.exists() && cam.name() != "ByEepromId" && cam.name() != "BySerialNo" && cam["Status"]["Available"] == true)
            {
                availableCams.push_back(cam[itmSerialNumber].asString());
            }
        }
    }
    catch (NxLibException &ex)
    {
        cout << "Error: getAvailableCameras failed " << ex.getItemPath() << " " << ex.getErrorText() << endl;
    }
    return availableCams;
}

bool EnsensoCamera::connectBySerialNumber(const std::string &serialNumber)
{
    // ############################ open camera ###############################
    try
    {
        int camCount = EnsensoCameraList.count(); // note: there are two additional non direct camera nodes attached to /Cameras/
        for (int i = 0; i < camCount; i++)
        {
            if (EnsensoCameraList[i].name() != "ByEepromId" && EnsensoCameraList[i].name() != "BySerialNo" && EnsensoCameraList[i][itmSerialNumber].asString() == serialNumber)
            {
                EnsensoCam = EnsensoCameraList[i]; //have to access this with int instead of serialNumber string, otherwise EnsensoCam.name() doesn't work for capture
                break;
            }
        }
        if (!EnsensoCam["Status"].exists())
        {
            return false;
        }
        if (EnsensoCam["Status"]["Available"] == true)
        {
            SerialNumber = EnsensoCam[itmSerialNumber].asString();
            cout << "Opening Camera " << SerialNumber << endl;
            NxLibCommand open(cmdOpen);                   // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
            open.parameters()[itmCameras] = SerialNumber; // Set parameters for the open command
            open.execute();
            cout << "Successfully connected!" << endl;
            Connected = true;
        }
        else
        {
            cout << "Connect failed as device is busy...\n";
            return false;
        }
    }
    catch (NxLibException &ex)
    {
        cout << "Error: Connect failed (Opening Camera) " << ex.getItemPath() << " " << ex.getErrorText() << endl;
        return false;
    }
    // ############################ set settings ###############################
    try
    {
        CaptureSettings = EnsensoCam[itmParameters][itmCapture];
        setEnsensoSetting(CaptureSettings, itmAutoExposure, true);
        setEnsensoSetting(CaptureSettings, itmAutoGain, false);
        setEnsensoSetting(CaptureSettings, itmTriggerMode, valSoftware); // valContinious
        // setEnsensoSetting(CaptureSettings, itmTriggerDelay, 60); //for trigger mode continious
        setEnsensoSetting(CaptureSettings, itmProjector, true);

        ProcessingSettings = EnsensoCam[itmParameters][itmDisparityMap][itmPostProcessing];
        //Median filtering will reduce noise inside surfaces while maintaining sharp edges, but object corners will be rounded
        setEnsensoSetting(ProcessingSettings, itmMedianFilterRadius, 2); // 1 for quality, 2 for volulme, unit is pixels
    }
    catch (NxLibException &ex)
    {
        cout << "Error: Connect failed (Settings) " << ex.getItemPath() << " " << ex.getErrorText() << endl;
        return false;
    }
    // ############################ single capture ###############################
    try
    {
        // Capture one image to get frame info
        NxLibCommand(cmdCapture, EnsensoCam.name()).execute(); // Without parameters, most commands just operate on all open cameras
        NxLibCommand(cmdRectifyImages, EnsensoCam.name()).execute();
        NxLibCommand(cmdComputeDisparityMap, EnsensoCam.name()).execute();
        NxLibCommand(cmdComputePointMap, EnsensoCam.name()).execute();
    }
    catch (NxLibException &ex)
    {
        cout << "Error: Connect failed (Single Capture) " << ex.getItemPath() << " " << ex.getErrorText() << endl;
        return false;
    }
    // ############################ read calibration ###############################
    try
    {
        // width, height. ensenso s10 pointmap is 364x272, raw and rectified imgs are 1456x1088
        int fullResWidth, fullResHeight;
        int pointMapWidth, pointMapHeight;
        EnsensoCam[itmImages][itmRectified].getBinaryDataInfo(&fullResWidth, &fullResHeight, 0, 0, 0, 0);
        EnsensoCam[itmImages][itmPointMap].getBinaryDataInfo(&pointMapWidth, &pointMapHeight, 0, 0, 0, 0);

        // get intrinsics from node tree
        int32_t *treePtr;
        std::string calibrationPath = "/Cameras/" + SerialNumber + "/Calibration/";
        auto calibStr = nxLibGetJson(treePtr, calibrationPath.c_str(), 0, 15, 0);
        Json::Value calibJson = StringToJson(calibStr);
        auto intrMat = calibJson["Dynamic"]["Stereo"]["Left"]["Camera"];
        // auto dc = calibJson["Monocular"]["Left"]["Distortion"]; //unclear so don't use them

        CalibrationIntrinsic = open3d::camera::PinholeCameraIntrinsic(fullResWidth, fullResHeight, intrMat[0][0].asDouble(), intrMat[1][1].asDouble(), intrMat[2][0].asDouble(), intrMat[2][1].asDouble());
        DepthLDTIntrinsic = getScaledIntrinsic(CalibrationIntrinsic, pointMapWidth, pointMapHeight);
        ColorLDTIntrinsic = DepthLDTIntrinsic;

        ApproxVoxelSize = 1/std::sqrt(DepthLDTIntrinsic.width_*DepthLDTIntrinsic.height_);

        return true;
    }
    catch (NxLibException &ex)
    {
        cout << "Error: Connect failed (Calibration) " << ex.getItemPath() << " " << ex.getErrorText() << endl;
        return false;
    }
}

bool EnsensoCamera::record(int n, int n_wait)
{
    // note: there are raw and rectified images (color), as well as a pointMap (pcd)
    // raw is distorted, rectified is undistorted and they have the same dimensions
    // pointmap has smaller dimensions than images, thus the depth image is smaller too
    // we use full resolution rectified images for calibration and resize them to pointmap dimensions for rgbd image
    // we ignore distorted raw images as distortion coeffs are unclear and not well documented by ensenso
    int nFramesOld = rgbdImages.size();
    for (int i = 0; i < n_wait; i++)
    {
        try
        {
            NxLibCommand(cmdCapture, EnsensoCam.name()).execute(); // Without parameters, most commands just operate on all open cameras
        }
        catch (NxLibException &ex)
        {
            // cout << "Warning: Dropped warmup frame " << i << ": " << ex.getItemPath() << " " << ex.getErrorText() << endl;
        }
    }
    int nFail = 0;
    int nFailMax = 30;
    for (int i = 0; i < n + nFail; i++)
    {
        try
        {
            // Execute the 'Capture', 'ComputeDisparityMap' and 'ComputePointMap' commands
            NxLibCommand(cmdCapture, EnsensoCam.name()).execute(); // Without parameters, most commands just operate on all open cameras
            NxLibCommand(cmdRectifyImages, EnsensoCam.name()).execute();
            NxLibCommand(cmdComputeDisparityMap, EnsensoCam.name()).execute();
            NxLibCommand(cmdComputePointMap, EnsensoCam.name()).execute();

            int width, height, channels, bytesPerElement;
            double timestamp;
            EnsensoCam[itmImages][itmRectified].getBinaryDataInfo(&width, &height, &channels, &bytesPerElement, 0, &timestamp);
            cv::Mat cvColorFR; //undistorted
            if (channels == 1 && bytesPerElement == 1)
            {
                std::vector<uint8_t> colorData;
                EnsensoCam[itmImages][itmRectified].getBinaryData(colorData, 0);
                cvColorFR = cv::Mat(height, width, CV_8UC1, colorData.data()).clone();
                cv::cvtColor(cvColorFR, cvColorFR, cv::ColorConversionCodes::COLOR_GRAY2RGB);
            }
            else
            {
                cout << "WARNING: Unexpected ensenso color image type." << endl;
            }
            FRCalibrationImages.push_back(cvColorFR);

            // depth image, color resize and camera pcd
            // NOTE for Ensenso S10: pointmap is 364x272, raw and rectified imgs are 1456x1088
            std::vector<float> pointMap;
            EnsensoCam[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, &channels, &bytesPerElement, 0, 0);
            EnsensoCam[itmImages][itmPointMap].getBinaryData(pointMap, 0);
            cv::Mat cvColor;
            cv::resize(cvColorFR, cvColor, cv::Size(width, height), 0, 0, cv::InterpolationFlags::INTER_LINEAR); //resize to depth dimensions
            cv::Mat cvDepth = cv::Mat(height, width, CV_32F, cv::Scalar(0));
            auto CamPcd = make_shared<open3d::geometry::PointCloud>();
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    // Get X,Y,Z coordinates of the point at image pixel (x,y)
                    float px = pointMap[(y * width + x) * 3];
                    float py = pointMap[(y * width + x) * 3 + 1];
                    float pz = pointMap[(y * width + x) * 3 + 2];

                    if (isnan(px) || isnan(py) || isnan(pz))
                        continue; // NaN values indicate missing pixels

                    cvDepth.at<float>(y, x) = pz;
                    Eigen::Vector3d point = Eigen::Vector3d(px, py, pz);
                    point /= 1000;
                    CamPcd->points_.push_back(point);
                    Eigen::Vector3d color = Eigen::Vector3d(cvColor.at<cv::Vec3b>(y, x)(0), cvColor.at<cv::Vec3b>(y, x)(1), cvColor.at<cv::Vec3b>(y, x)(2));
                    color /= 255;
                    CamPcd->colors_.push_back(color);
                }
            }

            // convert images to open3d
            std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
            rgbdImg->color_ = *OpenCVToOpen3D(cvColor);
            rgbdImg->depth_ = *OpenCVToOpen3D(cvDepth);
            rgbdImg->depth_ = *rgbdImg->depth_.ConvertDepthToFloatImage(1000, 100);

            // lastly push back pcd and rgbdImg
            Pcds.push_back(CamPcd);
            rgbdImages.push_back(rgbdImg);
            FrameTimestamps.push_back(timestamp);
        }
        catch (NxLibException &ex)
        {
            cout << "Warning: Dropped frame " << i << ": " << ex.getItemPath() << " " << ex.getErrorText() << endl;
            if (nFail < nFailMax)
            {
                cout << "Recording another.." << endl;
                nFail++;
            }
        }
    }
    return rgbdImages.size() > nFramesOld ? true : false;
}

bool EnsensoCamera::combineRecordedFrames(Eigen::Vector3d beltDirection, double beltSpeed)
{
    int n_frames = Pcds.size();
    if (n_frames < 2)
        return false;
    beltDirection.normalize();

    // cout << "combining " << n_frames << " frames" << endl;
    // cout << "beltdirection \n" << beltDirection << endl;
    // cout << "beltspeed " << beltSpeed << endl;

    //start with first pcd
    auto combinedPcd = make_shared<open3d::geometry::PointCloud>();
    *combinedPcd = *Pcds[0];
    //translate all pcds to first
    for (int i = 1; i < n_frames; i++)
    {
        double dt = FrameTimestamps[i] - FrameTimestamps[0];
        double s = beltSpeed * dt;
        // cout << "dt " << dt << endl;

        auto pcd = *Pcds[i];
        pcd.Translate(-beltDirection * s); //should now be aligned, todo also do icp as beltspeed may not be perfect
        *combinedPcd += pcd;
    }
    // open3d::visualization::DrawGeometries({combinedPcd, getOrigin()});

    //convert pcd to rgbd image
    auto combinedRgbd = getRGBDImageFromPCD(*combinedPcd);
    // open3d::visualization::DrawGeometries({combinedRgbd});

    //set combined frame to camera's current
    Pcds.clear();
    Pcds.push_back(combinedPcd);
    rgbdImages.clear();
    rgbdImages.push_back(combinedRgbd);
    FRCalibrationImages.resize(1);
    return true;
}

bool EnsensoCamera::disconnect()
{
    try
    {
        if (Connected)
        {
            NxLibCommand close(cmdClose);
            close.parameters()[itmCameras] = SerialNumber;
            close.execute();
            Connected = false;
        }
    }
    catch (NxLibException &ex)
    {
        cout << "Error: Disconnect failed (Closing Camera) " << ex.getItemPath() << " " << ex.getErrorText() << endl;
    }
    return !Connected;
}

template <typename T>
bool EnsensoCamera::setEnsensoSetting(const NxLibItem &settingNode, const std::string &itmString, T value)
{
    int retVal;
    settingNode[itmString].set(&retVal, value);
    if (retVal == 0)
        return true;
    else
    {
        cout << "Warning: Failed to set " << itmString << " (probably not supported)" << endl;
        return false;
    }
}

std::shared_ptr<EnsensoCamera> EnsensoCamera::CreateCameraPointer()
{
    return make_shared<EnsensoCamera>();
}

