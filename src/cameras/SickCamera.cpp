#include "SickCamera.h"
#include "SickCamera/PointXYZ.h"
#include "SickCamera/PointCloudPlyWriter.h"

#include "utils/vishelper.h"
#include "utils/conversions.h"
#include "utils/fileIO.h"
#include "utils/jsonUtil.h"
#include "utils/basicMath.h"

using namespace std;
using namespace visionary;

std::vector<SickCamera::PandiaSickDeviceInfo> MakeDeviceList()
{
    Json::Value json = readJsonFromDisk("./CameraSettings.json");
    auto ipAddresses = json["Sick_IPAddresses"];
    std::vector<SickCamera::PandiaSickDeviceInfo> v;
    for (auto &ip : ipAddresses)
    {
        SickCamera::PandiaSickDeviceInfo camInfo;
        camInfo.IpAddress = ip.asString();
        v.push_back(camInfo);
    }
    return v;
}

std::vector<SickCamera::PandiaSickDeviceInfo> SickCamera::DeviceInfoList = MakeDeviceList();

SickCamera::SickCamera(/* args */)
{
    CameraType = CameraTypes::Sick;
}

SickCamera::~SickCamera()
{
    disconnect();
}

bool SickCamera::disconnect()
{
    try
    {
        if (Connected)
        {
            for (auto &d : DeviceInfoList)
            {
                if (d.IpAddress == SerialNumber)
                {
                    d.busy = false;
                }
            }
            visionaryControl->stopAcquisition();
            visionaryControl->logout();
            visionaryControl->close();
            dataStream->close();
            Connected = false;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in disconnect: " << Zivid::toString(e) << std::endl;
    }
    return !Connected;
}

std::shared_ptr<SickCamera> SickCamera::CreateCameraPointer()
{
    return make_shared<SickCamera>();
}

std::vector<std::string> SickCamera::getAvailableCameras()
{
    // in case new entries are added to the json file in runtime, add these to DeviceInfoList
    auto tmpDeviceList = MakeDeviceList();
    for (auto &tmpDev: tmpDeviceList)
    {
        bool found = false;
        for (auto &dev: DeviceInfoList)
        {
            if (tmpDev.IpAddress == dev.IpAddress)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            DeviceInfoList.push_back(tmpDev);
        }
    }
    std::vector<std::string> availableCams; //ip adresses of cameras
    try
    {
        // // todo this unfortunately doesn't find the devices
        // unsigned int timeout = 5000; // The time how long to wait for a response from the devices.
        // VisionaryAutoIPScan ipScan;
        // SickCamera::deviceList = ipScan.doScan(timeout, "192.168.2.255", 2114u); // scan for devices
        // printf("Number of found devices: %u \n", deviceList.size());
        
        //push back serial numbers of cameras that are actually connectable
        for (auto &dev: DeviceInfoList)
        {
            if (!dev.busy)
            {
                auto pdataHandler_tmp = make_shared<VisionaryTMiniData>();
                auto dataStream_tmp = make_shared<VisionaryDataStream>(pdataHandler_tmp);
                if (dataStream_tmp->open(dev.IpAddress, htons(dev.IpPort)))
                {
                    availableCams.push_back(dev.IpAddress);
                    dataStream_tmp->close();
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in getAvailableCameras: " << e.what() << std::endl;
    }
    return availableCams;
}

//note for sick cameras atm we use ip address of camera as serialnumber
bool SickCamera::connectBySerialNumber(const std::string &serialNumber)
{
    try
    {
        if (Connected)
        {
            cout << "Info: Camera is already connected" << endl;
            return true;
        }
        pDataHandler = make_shared<VisionaryTMiniData>();
        dataStream = make_shared<VisionaryDataStream>(pDataHandler);
        visionaryControl = make_shared<VisionaryControl>();

        int deviceIndex = -1;
        for (int i = 0; i < DeviceInfoList.size(); i++)
        {
            if (DeviceInfoList[i].IpAddress == serialNumber)
            {
                deviceIndex = i;
                SerialNumber = serialNumber;
                break;
            }
        }
        if (deviceIndex == -1)
        {
            cout << "Connect failed as camera with requested serialnumber was not found" << endl;
        }

        //-----------------------------------------------
        // Connect to devices data stream
        // if (!dataStream->open(DeviceInfoList[camIndex].IpAddress, stoi(DeviceInfoList[camIndex].Port)))
        if (!dataStream->open(DeviceInfoList[deviceIndex].IpAddress, htons(DeviceInfoList[deviceIndex].IpPort)))
        {
            std::printf("Failed to open data stream connection to device.\n");
            return false; // connection failed
        }

        //-----------------------------------------------
        // Connect to devices control channel
        if (!visionaryControl->open(VisionaryControl::ProtocolType::COLA_2, DeviceInfoList[deviceIndex].IpAddress, 5000 /*ms*/))
        {
            std::printf("Failed to open control connection to device.\n");
            return false; // connection failed
        }

        //-----------------------------------------------
        // read Device Ident
        std::printf("DeviceIdent: '%s'\n", visionaryControl->getDeviceIdent().c_str());

        //-----------------------------------------------
        // Login as authorized client
        if (visionaryControl->login(IAuthentication::UserLevel::AUTHORIZED_CLIENT, "CLIENT"))
        {
            //-----------------------------------------------
            // An example of reading an writing device parameters is shown here.
            // Use the "SOPAS Communication Interface Description" PDF to determine data types for other variables
            //-----------------------------------------------
            CoLaCommand setEnDepthMaskCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "enDepthMask").parameterBool(true).build();
            CoLaCommand setEnDepthMaskResponse = visionaryControl->sendCommand(setEnDepthMaskCommand);
            if (setEnDepthMaskResponse.getError() == CoLaError::OK)
            {
                std::printf("Set enDepthMask\n");
            }
            //note: many filters are already enabled by default
            CoLaCommand command = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "enIntFilter").parameterBool(true).build();
            CoLaCommand response = visionaryControl->sendCommand(command);
            if (response.getError() == CoLaError::OK)
            {
                std::printf("Set enIntFilter\n");
            }

            //-----------------------------------------------
            // Read humidity parameter
            CoLaCommand getHumidity = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "humidity").build();
            CoLaCommand humidityResponse = visionaryControl->sendCommand(getHumidity);
            const double humidity = CoLaParameterReader(humidityResponse).readLReal();
            // std::printf("Read humidity = %f\n", humidity);

            //-----------------------------------------------
            // Read info messages variable
            CoLaCommand getMessagesCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "MSinfo").build();
            CoLaCommand messagesResponse = visionaryControl->sendCommand(getMessagesCommand);

            //-----------------------------------------------
        }

        //-----------------------------------------------
        // Logout from device after reading variables.
        if (!visionaryControl->logout())
        {
            std::printf("Failed to logout\n");
        }

        //-----------------------------------------------
        // Start image acquisiton and continously receive frames
        if (!visionaryControl->startAcquisition())
        {
            std::printf("Failed to start Acquisition\n");
        }

        //aquire a frame so camera parameters are set
        int nFramesMax = 60;
        int nFrames = 0;
        while(!dataStream->getNextFrame() && nFrames < nFramesMax) {
            nFrames++;
        }
        if (nFrames >= nFramesMax) {
            cout << "Connect failed: Couldn't aquire a frame for initialization. Please try again." << endl;
            return false;
        }

        auto params = pDataHandler->getCameraParameters();
        DepthLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(params.width, params.height, params.fx, params.fy, params.cx, params.cy);
        ColorLDTIntrinsic = DepthLDTIntrinsic;
        CalibrationIntrinsic = DepthLDTIntrinsic;
        BrownianCalibrationCamDistortionCoeffs = {0, 0, 0, 0, 0};
        ApproxVoxelSize = 1/std::sqrt(DepthLDTIntrinsic.width_*DepthLDTIntrinsic.height_);
        DeviceInfoList[deviceIndex].busy = true;
        Connected = true;

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in connect: " << e.what() << std::endl;
        return false;
    }
}

bool SickCamera::record(int n, int n_wait)
{
    int nFramesOld = rgbdImages.size();
    int nInvalidMax = 60;
    int nInvalid = 0;
    int width = DepthLDTIntrinsic.width_;
    int height = DepthLDTIntrinsic.height_;
    for (int i = 0; i < n + nInvalid && nInvalid < nInvalidMax; i++)
    {
        try
        {
            if (!dataStream->getNextFrame())
            {
                cout << "Dropped frame..." << endl;
                nInvalid++;
                continue; // No valid frame received
            }
            // std::printf("Frame received in continuous mode, frame #%d \n", pDataHandler->getFrameNum());
            std::vector<uint16_t> distanceMap = pDataHandler->getDistanceMap(); //unit is 1/4 mm
            std::vector<uint16_t> intensityMap = pDataHandler->getIntensityMap();
            // std::vector<uint16_t> stateMap = pDataHandler->getStateMap();

            // ################ images to opencv ####################
            if (distanceMap.size() != width*height || intensityMap.size() != width*height){
                cout << "Error: Image size does not match in record!" << endl;
                return false;
            }
            //depth image (gets generated by pcd as distance map is radial depth)
            // cv::Mat cvDepth = cv::Mat(height, width, CV_16UC1, distanceMap.data()).clone(); //actual copy
            // cvDepth.convertTo(cvDepth, CV_32F, 4); //now float and in mm

            //color image (grayscale)
            cv::Mat cvGrayRaw = cv::Mat(height, width, CV_16UC1, intensityMap.data()).clone(); //actual copy
            double minVal;
            double maxVal;
            minMaxLoc(cvGrayRaw, &minVal, &maxVal);
            cv::Mat cvGray = cv::Mat(cvGrayRaw.rows, cvGrayRaw.cols, CV_8UC1);
            for (int i = 0; i < cvGrayRaw.rows; i++)
            {
                for (int j = 0; j < cvGrayRaw.cols; j++)
                {
                    float tmp = (float)cvGrayRaw.at<uint16_t>(i, j) / maxVal * 255.0;
                    cvGray.at<uint8_t>(i, j) = (uint8_t)tmp;
                }
            }
            cv::equalizeHist(cvGray, cvGray);
            cv::Mat cvColor = cv::Mat(cvGrayRaw.rows, cvGrayRaw.cols, CV_8UC3);
            cv::cvtColor(cvGray, cvColor, cv::COLOR_GRAY2RGB);
            FRCalibrationImages.push_back(cvColor);

            //################ pcd ####################
            std::vector<PointXYZ> pointCloud; //undistorted, unit is meters
            pDataHandler->generatePointCloud(pointCloud);
            // pDataHandler->transformPointCloud(pointCloud);
            if (pointCloud.empty())
                cout << "Warning pointcloud is empty" << endl;
            auto o3dPcd = make_shared<open3d::geometry::PointCloud>();
            for (int i = 0; i < pointCloud.size(); i++)
            {
                auto &p = pointCloud[i];
                if (isnan(p.x) || isnan(p.y) || isnan(p.z) || p.z == 0)
                    continue; // NaN values indicate missing pixels
                o3dPcd->points_.push_back(Eigen::Vector3d(p.x, p.y, p.z));
            }
            o3dPcd->Transform(getRz(M_PI));
            o3dPcd->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
            // open3d::visualization::DrawGeometries({o3dPcd, getOrigin()});

            // ################### to open3d ##########################
            std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
            rgbdImg->color_ = *OpenCVToOpen3D(cvColor);
            rgbdImg->depth_ = *getDepthImageFromPCD(*o3dPcd);
            // rgbdImg->depth_ = *OpenCVToOpen3D(cvDepth);
            // rgbdImg->depth_ = *rgbdImg->depth_.ConvertDepthToFloatImage(1000, 100);

            // lastly push back pcd and rgbdImg
            Pcds.push_back(o3dPcd);
            rgbdImages.push_back(rgbdImg);
            // open3d::visualization::DrawGeometries({make_shared<open3d::geometry::Image>(rgbdImg->depth_)});
            // open3d::visualization::DrawGeometries({rgbdImg});
            
        }
        catch (const std::exception &e)
        {
            cout << "Warning: Dropped frame due to: " << e.what()  << endl;
            nInvalid++;
        }
    }
    if (nInvalid >= nInvalidMax)
    {
        cout << "Warning in record: Camera " << SerialNumber << " reached nInvalidMax" << endl;
    }
    return rgbdImages.size() > nFramesOld ? true : false;
}

