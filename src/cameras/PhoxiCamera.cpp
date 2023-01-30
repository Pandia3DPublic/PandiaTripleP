#include "PhoxiCamera.h"
#include "utils/PhoxiUtil.h"
#include "utils/conversions.h"
#include "utils/jsonUtil.h"
using namespace std;

pho::api::PhoXiFactory PhoxiCamera::PhoxiFactory;
std::vector<pho::api::PhoXiDeviceInformation> PhoxiCamera::PhoxiDeviceList;

PhoxiCamera::PhoxiCamera(/* args */)
{
    CameraType = CameraTypes::Phoxi;
}

PhoxiCamera::~PhoxiCamera()
{
    disconnect();
}

std::vector<std::string> PhoxiCamera::getAvailableCameras()
{
    std::vector<std::string> availableCams;
    PhoxiCamera::PhoxiDeviceList.clear();
    try
    {
        //Check if the PhoXi Control Software is running
        if (!PhoxiCamera::PhoxiFactory.isPhoXiControlRunning())
        {
            cout << "PhoXi Control Software is not running" << endl;
            return availableCams;
        }
        //Get List of available devices on the network
        auto DeviceList = PhoxiCamera::PhoxiFactory.GetDeviceList();
        if (DeviceList.empty())
        {
            cout << "PhoXi Factory has found 0 devices" << endl;
            return availableCams;
        }
        auto json = readJsonFromDisk("./CameraSettings.json");
        bool includeFileCams = true;
        setValueFromJson(json, "Phoxi_IncludeFileCams", includeFileCams);
        for (int i = 0; i < DeviceList.size(); i++)
        {
            if (DeviceList[i].Status.Ready) {
                if (includeFileCams && DeviceList[i].IsFileCamera)
                {
                    PhoxiCamera::PhoxiDeviceList.push_back(DeviceList[i]);
                }
                else if (!DeviceList[i].IsFileCamera)
                {
                    PhoxiCamera::PhoxiDeviceList.push_back(DeviceList[i]);
                }
            }
        }
        printDeviceInfoList(PhoxiCamera::PhoxiDeviceList);
        for (int i = 0; i < PhoxiCamera::PhoxiDeviceList.size(); i++)
        {
            availableCams.push_back(PhoxiCamera::PhoxiDeviceList[i].HWIdentification);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in getNumberofConnectedDevices: " << e.what() << '\n';
    }
    return availableCams;
}

bool PhoxiCamera::connectBySerialNumber(const std::string &serialNumber)
{
    try
    {
        int camIndex = -1;
        for (int i = 0; i < PhoxiDeviceList.size(); i++)
        {
            if (PhoxiDeviceList.at(i).HWIdentification == serialNumber)
            {
                camIndex = i;
                break;
            }
        }
        if (camIndex == -1)
        {
            cout << "Did not find Phoxi Camera with serialnumber " << serialNumber << endl;
            return false;
        }
        if (!PhoxiDeviceList[camIndex].Status.Ready)
        {
            cout << "Connect failed as camera is busy..." << endl;
            return false;
        }

        cout << "Trying to connect to " << serialNumber << endl;
        PhoxiDevice = PhoxiFactory.CreateAndConnect(serialNumber, 5000); //5s timeout
        if (PhoxiDevice && PhoxiDevice->isConnected())
        {
            cout << "Successfully connected!" << endl;
            SerialNumber = PhoxiDeviceList.at(camIndex).HWIdentification;
            Connected = true;
        }
        else
        {
            cout << "Connect failed..." << endl;
            return false;
        }
        if (PhoxiDevice->Profiles.isEnabled() && !PhoxiDeviceList.at(camIndex).IsFileCamera)
        {
            std::vector<pho::api::PhoXiProfileDescriptor> ProfilesList = PhoxiDevice->Profiles;
            if (!PhoxiDevice->Profiles.isLastOperationSuccessful())
            {
                cout << "Can not get profile list: " << PhoxiDevice->Profiles.GetLastErrorMessage() << endl;
                return false;
            }
            // Available profiles: See PhoxiControl
            std::string NewActiveProfile = "DEFAULT";
            auto json = readJsonFromDisk("./CameraSettings.json");
            setValueFromJson(json, "Phoxi_Profile", NewActiveProfile);
            for (const pho::api::PhoXiProfileDescriptor &Profile : ProfilesList)
            {
                if (Profile.Name == NewActiveProfile)
                {
                    PhoxiDevice->ActiveProfile = NewActiveProfile;
                    if (!PhoxiDevice->ActiveProfile.isLastOperationSuccessful())
                    {
                        cout << "Can not set active profile: " << PhoxiDevice->ActiveProfile.GetLastErrorMessage() << endl;
                        return false;
                    }
                    break;
                }
            }
            cout << "Using Profile: " << PhoxiDevice->ActiveProfile.GetStoredValue() << endl;
        }
        if (PhoxiDevice->CapturingSettings.isEnabled() && !PhoxiDeviceList.at(camIndex).IsFileCamera)
        {
            pho::api::PhoXiCapturingSettings NewCapturingSettings = PhoxiDevice->CapturingSettings;
            if (!PhoxiDevice->CapturingSettings.isLastOperationSuccessful())
            {
                cout << "Can not get capturing settings: " << PhoxiDevice->CapturingSettings.GetLastErrorMessage() << endl;
                return false;
            }
            // NewCapturingSettings.CodingQuality = pho::api::PhoXiCodingQuality::Ultra;
            PhoxiDevice->MotionCamCameraMode->TextureSource = pho::api::PhoXiTextureSource::LED; //use grayscale texture
            PhoxiDevice->CapturingSettings = NewCapturingSettings;
            if (!PhoxiDevice->CapturingSettings.isLastOperationSuccessful())
            {
                cout << "Can not set new capturing settings: " << PhoxiDevice->CapturingSettings.GetLastErrorMessage() << endl;
                return false;
            }
        }
        if (PhoxiDevice->MotionCamCameraMode.isEnabled() && !PhoxiDeviceList.at(camIndex).IsFileCamera)
        {
            PhoxiDevice->MotionCamCameraMode->TextureSource = pho::api::PhoXiTextureSource::Laser; //use grayscale texture
            if (!PhoxiDevice->MotionCamCameraMode.isLastOperationSuccessful())
            {
                cout << "Can not set new MotionCamCameraMode: " << PhoxiDevice->MotionCamCameraMode.GetLastErrorMessage() << endl;
                return false;
            }
        }
        if (PhoxiDevice->MotionCamScannerMode.isEnabled() && !PhoxiDeviceList.at(camIndex).IsFileCamera)
        {
            PhoxiDevice->MotionCamScannerMode->TextureSource = pho::api::PhoXiTextureSource::LED; //use grayscale texture
            if (!PhoxiDevice->MotionCamScannerMode.isLastOperationSuccessful())
            {
                cout << "Can not set new MotionCamScannerMode: " << PhoxiDevice->MotionCamScannerMode.GetLastErrorMessage() << endl;
                return false;
            }
        }
        if (PhoxiDevice->OutputSettings.isEnabled() && !PhoxiDeviceList.at(camIndex).IsFileCamera)
        {
            pho::api::FrameOutputSettings NewOutputSettings = PhoxiDevice->OutputSettings;
            if (!PhoxiDevice->OutputSettings.isLastOperationSuccessful())
            {
                cout << "Can not get output settings: " << PhoxiDevice->OutputSettings.GetLastErrorMessage() << endl;
                return false;
            }
            NewOutputSettings.SendPointCloud = true; //needed
            NewOutputSettings.SendNormalMap = true; //needed, not working??
            NewOutputSettings.SendDepthMap = true; //needed
            NewOutputSettings.SendTexture = true; //needed
            NewOutputSettings.SendConfidenceMap = false;
            NewOutputSettings.SendEventMap = false;
            NewOutputSettings.SendColorCameraImage = false;
            PhoxiDevice->OutputSettings = NewOutputSettings;
            if (!PhoxiDevice->OutputSettings.isLastOperationSuccessful())
            {
                cout << "Can not set new output settings: " << PhoxiDevice->OutputSettings.GetLastErrorMessage() << endl;
                return false;
            }
        }


        auto &Calibration = PhoxiDevice->CalibrationSettings;
        // PrintCalibrationSettings(Calibration);
        auto DepthIntr = Calibration->CameraMatrix;
        DepthLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(PhoxiDevice->Resolution->Width, PhoxiDevice->Resolution->Height,
                                                                   DepthIntr.At(0, 0), DepthIntr.At(1, 1), DepthIntr.At(0, 2), DepthIntr.At(1, 2));
        ColorLDTIntrinsic = DepthLDTIntrinsic;
        CalibrationIntrinsic = DepthLDTIntrinsic;
        BrownianCalibrationCamDistortionCoeffs = {0, 0, 0, 0, 0};

        ApproxVoxelSize = 1/std::sqrt(DepthLDTIntrinsic.width_*DepthLDTIntrinsic.height_);
        
        PhoxiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
        PhoxiDevice->StartAcquisition();
        int n_wait = 1;
        int n_wait_max = 5;
        while (!PhoxiDevice->isAcquiring() && n_wait <= n_wait_max)
        {
            cout << "Waiting for camera to start acquisition (" << n_wait << "s / " << n_wait_max << "s)" << endl;
            std::this_thread::sleep_for(1s);
            n_wait++;
        }
        if (!PhoxiDevice->isAcquiring())
        {
            cout << "Your device could not start acquisition!" << endl;
            return false;
        }
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in connect: " << e.what() << endl;
        return false;
    }
}

bool PhoxiCamera::record(int n, int n_wait)
{
    int nFramesOld = rgbdImages.size();
    PhoxiDevice->ClearBuffer();
    for (int i = 0; i < n; ++i)
    {
        try
        {
            int FrameID = PhoxiDevice->TriggerFrame();
            if (FrameID < 0)
            {
                cout << "Trigger was unsuccessful!" << endl;
                continue;
            }
            pho::api::PFrame Frame = PhoxiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Default);
            if (!Frame || Frame->Empty())
            {
                cout << "Failed to retrieve the frame!" << endl;
                continue;
            }
            // printFrameInfo(Frame);
            // printFrameData(Frame);

            //images to OpenCV
            cv::Mat cvDepth;
            Frame->DepthMap.ConvertTo(cvDepth);  //distorted
            cv::Mat cvColorRaw;
            cv::Mat cvColor;
            if (!Frame->Texture.Empty()) // grayscale texture case
            {
                Frame->Texture.ConvertTo(cvColorRaw); //distorted
                cvColor = cv::Mat(cvColorRaw.rows, cvColorRaw.cols, CV_8UC3);
                double minVal;
                double maxVal;
                minMaxLoc(cvColorRaw, &minVal, &maxVal);
                cv::Mat cvGray = cv::Mat(cvColorRaw.rows, cvColorRaw.cols, CV_8UC1);
                for (int i = 0; i < cvColorRaw.rows; i++)
                {
                    for (int j = 0; j < cvColorRaw.cols; j++)
                    {
                        float tmp = cvColorRaw.at<float>(i, j) / maxVal * 255;
                        cvGray.at<uint8_t>(i, j) = (uint8_t)tmp;
                    }
                }
                cv::equalizeHist(cvGray, cvGray);
                cv::cvtColor(cvGray, cvColor, cv::COLOR_GRAY2RGB);
            }
            if (!Frame->TextureRGB.Empty()) // color texture case
            {
                Frame->TextureRGB.ConvertTo(cvColorRaw); //distorted
                cvColor = cv::Mat(cvColorRaw.rows, cvColorRaw.cols, CV_8UC3);
                double minVal = 0;
                double maxVal = 0;
                for (int i = 0; i < cvColorRaw.rows; i++)
                {
                    for (int j = 0; j < cvColorRaw.cols; j++)
                    {
                        if (cvColorRaw.at<cv::Vec3s>(i, j)(0) > maxVal)
                            maxVal = cvColorRaw.at<cv::Vec3s>(i, j)(0);
                        if (cvColorRaw.at<cv::Vec3s>(i, j)(1) > maxVal)
                            maxVal = cvColorRaw.at<cv::Vec3s>(i, j)(1);
                        if (cvColorRaw.at<cv::Vec3s>(i, j)(2) > maxVal)
                            maxVal = cvColorRaw.at<cv::Vec3s>(i, j)(2);
                    }
                }
                for (int i = 0; i < cvColorRaw.rows; i++)
                {
                    for (int j = 0; j < cvColorRaw.cols; j++)
                    {
                        cvColor.at<cv::Vec3b>(i, j)(0) = (uint8_t)cvColorRaw.at<cv::Vec3s>(i, j)(0) / maxVal * 255;
                        cvColor.at<cv::Vec3b>(i, j)(1) = (uint8_t)cvColorRaw.at<cv::Vec3s>(i, j)(1) / maxVal * 255;
                        cvColor.at<cv::Vec3b>(i, j)(2) = (uint8_t)cvColorRaw.at<cv::Vec3s>(i, j)(2) / maxVal * 255;
                    }
                }
                cv::Mat cvTemp;
                cv::cvtColor(cvColor, cvTemp, cv::COLOR_RGB2YCrCb);
                std::vector<cv::Mat> channels;
                cv::split(cvTemp, channels);
                cv::equalizeHist(channels[0], channels[0]);
                cv::merge(channels, cvTemp);
                cv::cvtColor(cvTemp, cvColor, cv::COLOR_YCrCb2RGB);
            }
            FRCalibrationImages.push_back(cvColor.clone());

            // phoxi pcd
            cv::Mat cvPcd;
            cv::Mat cvNormals;
            Frame->PointCloud.ConvertTo(cvPcd); //undistorted
            Frame->NormalMap.ConvertTo(cvNormals);
            auto PhoxiPcd = make_shared<open3d::geometry::PointCloud>();
            for (int i = 0; i < cvPcd.rows; i++)
            {
                for (int j = 0; j < cvPcd.cols; j++)
                {
                    Eigen::Vector3d p = Eigen::Vector3d(cvPcd.at<cv::Point3f>(i, j).x, cvPcd.at<cv::Point3f>(i, j).y, cvPcd.at<cv::Point3f>(i, j).z);
                    p /= 1000;
                    // Eigen::Vector3d n = Eigen::Vector3d(cvNormals.at<cv::Point3f>(i, j).x, cvNormals.at<cv::Point3f>(i, j).y, cvNormals.at<cv::Point3f>(i, j).z);
                    // n.normalize();
                    Eigen::Vector3d rgb = Eigen::Vector3d(cvColor.at<cv::Vec3b>(i, j)(0), cvColor.at<cv::Vec3b>(i, j)(1), cvColor.at<cv::Vec3b>(i, j)(2));
                    rgb /= 255;
                    if (p != Eigen::Vector3d::Zero())// && n != Eigen::Vector3d::Zero())
                    {
                        PhoxiPcd->points_.push_back(p);
                        // PhoxiPcd->normals_.push_back(n);
                        PhoxiPcd->colors_.push_back(rgb);
                    }
                }
            }
            // open3d rgbd image
            std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
            rgbdImg->color_ = *OpenCVToOpen3D(cvColor);
            rgbdImg->depth_ = *OpenCVToOpen3D(cvDepth);
            rgbdImg->depth_ = *rgbdImg->depth_.ConvertDepthToFloatImage(1000,100);

            // lastly push back pcd and rgbdImg
            Pcds.push_back(PhoxiPcd);
            rgbdImages.push_back(rgbdImg);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in record: " << e.what() << endl;
        }
    }
    // PhoxiDevice->StopAcquisition();
    return rgbdImages.size() > nFramesOld ? true : false;
}

bool PhoxiCamera::disconnect()
{
    return PhoxiDevice->Disconnect(true, true);
}

std::shared_ptr<PhoxiCamera> PhoxiCamera::CreateCameraPointer()
{
    return make_shared<PhoxiCamera>();
}

