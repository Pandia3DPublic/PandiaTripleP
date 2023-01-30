#include "ZividCamera.h"
#include "utils/cameraFunctions.h"
#include "utils/conversions.h"
#include "utils/miscFunctions.h"
#include "utils/jsonUtil.h"
using namespace std;

Zivid::Application ZividCamera::ZividApp;
std::vector<Zivid::Camera> ZividCamera::ZividCameraList;

ZividCamera::ZividCamera(/* args */)
{
    CameraType = CameraTypes::Zivid;
}

ZividCamera::~ZividCamera()
{
    disconnect();
}

std::vector<std::string> ZividCamera::getAvailableCameras()
{
    std::vector<std::string> availableCams;
    try
    {
        ZividCamera::ZividCameraList = ZividCamera::ZividApp.cameras();
        for (int i = 0; i < ZividCamera::ZividCameraList.size(); i++)
        {
            auto &cam = ZividCamera::ZividCameraList[i];
            if (cam.state().isAvailable())
            {
                availableCams.push_back(cam.info().serialNumber().value());
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in getAvailableCameras: " << Zivid::toString(e) << std::endl;
    }
    return availableCams;
}

bool ZividCamera::connectBySerialNumber(const std::string &serialNumber)
{
    try
    {
        int camIndex = -1;
        ZividCameraList = ZividCamera::ZividApp.cameras();
        for (int i = 0; i < ZividCameraList.size(); i++)
        {
            if (ZividCameraList[i].info().serialNumber().value() == serialNumber)
            {
                camIndex = i;
                break;
            }
        }
        if (camIndex == -1)
        {
            cout << "Connect failed: Did not find Zivid Camera with serialnumber " << serialNumber << endl;
            return false;
        }
        if (!ZividCameraList[camIndex].state().isAvailable())
        {
            cout << "Connect failed: Zivid Camera is unavailable" << endl;
            return false;
        }

        cout << "Trying to connect to " << ZividCameraList.at(camIndex).info() << endl;
        ZividCam = ZividCameraList.at(camIndex).connect();
        if (ZividCam.state().isConnected())
        {
            cout << "Successfully connected!" << endl;
            Connected = true;
            SerialNumber = ZividCam.info().serialNumber().value();
        }
        else
        {
            cout << "Failed to connect to camera..." << endl;
            return false;
        }

        const auto suggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{std::chrono::milliseconds{1200}}};

        std::cout << "Running Capture Assistant with parameters:\n"
                  << suggestSettingsParameters << std::endl;
        ZividSettings = Zivid::CaptureAssistant::suggestSettings(ZividCam, suggestSettingsParameters);

        std::cout << "Settings suggested by Capture Assistant:" << std::endl;
        std::cout << ZividSettings.acquisitions() << std::endl;

        std::cout << "Manually configuring processing settings (Capture Assistant only suggests acquisition settings)" << std::endl;
        const auto BasicProcessing =
            Zivid::Settings::Processing{Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
                                        Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                                        Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{1.5}};
        ZividSettings.set(BasicProcessing);

        // use stripe engine for reflective objects, this overwrites BasicProcessing settings
        auto json = readJsonFromDisk("./CameraSettings.json");
        bool useStripeEngine = true;
        bool useDistortionCoeffs = false;
        setValueFromJson(json, "Zivid_UseStripeEngine", useStripeEngine);
        setValueFromJson(json, "Zivid_UseDistortionCoeffs", useDistortionCoeffs);
        if (useStripeEngine)
        {
            cout << "USING STRIPE ENGINE" << endl;
            ZividSettings.set(Zivid::Settings::Experimental::Engine::stripe);
            const auto AdvancedProcessing = Zivid::Settings::Processing{
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{1.5},
                Zivid::Settings::Processing::Filters::Noise::Removal::Enabled::yes,
                Zivid::Settings::Processing::Filters::Noise::Removal::Threshold{10.0}, // 7
                Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
                Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{7.0}, // 5
                Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{0.4},
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold{0.5},
                Zivid::Settings::Processing::Color::Balance::Red{1.0},
                Zivid::Settings::Processing::Color::Balance::Green{1.0},
                Zivid::Settings::Processing::Color::Balance::Blue{1.0},
                Zivid::Settings::Processing::Color::Gamma{1.0},
                Zivid::Settings::Processing::Color::Experimental::Mode::toneMapping};
            ZividSettings.set(AdvancedProcessing);
        }
        // cout << "All Zivid Settings used: \n" << ZividSettings << endl;

        auto intrinsics = Zivid::Experimental::Calibration::intrinsics(ZividCam);
        auto resolution = Zivid::Experimental::SettingsInfo::resolution(ZividCam.info(), ZividSettings);
        auto intr = intrinsics.cameraMatrix();
        auto dc = intrinsics.distortion();
        DepthLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(resolution.width(), resolution.height(), intr.fx().value(), intr.fy().value(), intr.cx().value(), intr.cy().value());
        ColorLDTIntrinsic = DepthLDTIntrinsic;
        CalibrationIntrinsic = DepthLDTIntrinsic;
        ApproxVoxelSize = 1/std::sqrt(DepthLDTIntrinsic.width_*DepthLDTIntrinsic.height_);
        if (useDistortionCoeffs)
        {
            cout << "USING ZIVID DISTORTION COEFFICIENTS" << endl;
            BrownianCalibrationCamDistortionCoeffs = {(float)intrinsics.distortion().k1().value(),
            (float)intrinsics.distortion().k2().value(), 
            (float)intrinsics.distortion().p1().value(), 
            (float)intrinsics.distortion().p2().value(), 
            (float)intrinsics.distortion().k3().value()};
        }
        else
        {
            BrownianCalibrationCamDistortionCoeffs = {0, 0, 0, 0, 0}; // todo better for zivid 2 (maybe)
        }
  
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in connect: " << Zivid::toString(e) << std::endl;
        return false;
    }
}

bool ZividCamera::record(int n, int n_wait)
{
    int nFramesOld = rgbdImages.size();
    for (int i = 0; i < n; ++i)
    {
        try
        {
            auto frame = ZividCam.capture(ZividSettings);
            auto pcd = frame.pointCloud();
            if (pcd.isEmpty())
            {
                cout << "Warning: Zivid Pcd is empty" << endl;
                continue;
            }
            auto rgba = pcd.copyImageRGBA();
            // The cast for colors.data() is required because the cv::Mat constructor requires non-const void *.
            // It does not actually mutate the data, it only adds an OpenCV header to the matrix. We then protect
            // our own instance with const.
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            auto cvRGBA = cv::Mat(rgba.height(), rgba.width(), CV_8UC4, const_cast<void *>(static_cast<const void *>(rgba.data()))); // wrapping data in opencv matrix, no copying occurs here
            cv::Mat cvColor;
            cv::cvtColor(cvRGBA, cvColor, cv::COLOR_RGBA2RGB);
            FRCalibrationImages.push_back(cvColor.clone());

            // note: invalid values in depth and cvDepth are nan, not 0
            auto depthImg = make_shared<open3d::geometry::Image>();
            depthImg->Prepare(DepthLDTIntrinsic.width_, DepthLDTIntrinsic.height_, 1, 4);

            // camera pcd (undistorted)
            auto pcdPoints = pcd.copyPointsXYZ();
            auto pcdNormals = pcd.copyNormalsXYZ();
            auto ZividPcd = make_shared<open3d::geometry::PointCloud>();
            for (int i = 0; i < pcdPoints.height(); i++)
            {
                for (int j = 0; j < pcdPoints.width(); j++)
                {
                    Eigen::Vector3d n(pcdNormals(i, j).x, pcdNormals(i, j).y, pcdNormals(i, j).z);
                    if (!pcdPoints(i, j).isNaN() && !pcdNormals(i, j).isNaN() && abs(n.dot(Eigen::Vector3d(0, 0, 1))) > 0.34) // inbuild normal filter
                    {
                        Eigen::Vector3d p = Eigen::Vector3d(pcdPoints(i, j).x, pcdPoints(i, j).y, pcdPoints(i, j).z);
                        p /= 1000;
                        *depthImg->PointerAt<float>(j, i) = pcdPoints(i, j).z / 1000.0;
                        Eigen::Vector3d n = Eigen::Vector3d(pcdNormals(i, j).x, pcdNormals(i, j).y, pcdNormals(i, j).z);
                        n.normalize();
                        Eigen::Vector3d rgb = Eigen::Vector3d(cvColor.at<cv::Vec3b>(i, j)(0), cvColor.at<cv::Vec3b>(i, j)(1), cvColor.at<cv::Vec3b>(i, j)(2));
                        rgb /= 255;
                        ZividPcd->points_.push_back(p);
                        ZividPcd->normals_.push_back(n);
                        ZividPcd->colors_.push_back(rgb);
                    }
                    else
                    {
                        *depthImg->PointerAt<float>(j, i) = 0;
                    }
                }
            }

            // convert to open3d
            std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
            rgbdImg->color_ = *OpenCVToOpen3D(cvColor);
            rgbdImg->depth_ = *getDepthImageFromPCD(*ZividPcd);

            // rgbdImg->depth_ = *depthImg;

            // lastly push back pcd and rgbdImg
            Pcds.push_back(ZividPcd);
            rgbdImages.push_back(rgbdImg);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in record: " << Zivid::toString(e) << std::endl;
        }
    }
    return rgbdImages.size() > nFramesOld ? true : false;
}

bool ZividCamera::disconnect()
{
    try
    {
        ZividCam.disconnect();
        Connected = false;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in disconnect: " << Zivid::toString(e) << std::endl;
    }
    return !Connected;
}

std::shared_ptr<ZividCamera> ZividCamera::CreateCameraPointer()
{
    return make_shared<ZividCamera>();
}



