#include "CalibrationSingleton.h"
#include "qualityUtil.h"
#include "utils/fileIO.h"
#include "utils/conversions.h"
#include "utils/3DFunctions.h"
#include "utils/basicMath.h"
#include "cameras/EnsensoCamera.h"
#include "cameras/IntelCamera.h"
#include "cameras/KinectCamera.h"
#include "cameras/PhoxiCamera.h"
#include "cameras/ZividCamera.h"
using namespace cv;
using namespace std;

CalibrationSingleton::CalibrationSingleton()
{
}

CalibrationSingleton::~CalibrationSingleton()
{
}

cv::Mat CalibrationSingleton::getImagewithCorners(std::shared_ptr<CameraWrapper> cam)
{
    cv::Size CalibrationPattern{Pattern_w, Pattern_h};
    // auto &img = cam.rgbdImages.front()->color_;
    // auto color = Open3DToOpencv(img);
    if (cam->FRCalibrationImages.empty()) {
        cout << "Warning: Empty image in getImagewithCorners!" << endl;
        return cv::Mat();
    }
    auto color = cam->FRCalibrationImages.front();

    vector<cv::Point2f> main_chessboard_corners;
    vector<cv::Point2f> secondary_chessboard_corners;

    bool got_corners = cv::findChessboardCorners(color,
                                                 CalibrationPattern,
                                                 main_chessboard_corners, 
                                                 cv::CALIB_CB_FAST_CHECK);

    cv::Mat out = color.clone();
    if (out.channels() == 1) 
        cv::cvtColor(out, out, cv::COLOR_GRAY2RGB);
    else
        cv::cvtColor(out, out, cv::COLOR_BGR2RGB); //for some reason drawChessboardcorners reverses rgb to bgr

    if (got_corners)
        cv::drawChessboardCorners(out, CalibrationPattern, main_chessboard_corners, true);
    else
        return out;
    // cv::cvtColor(out,out,cv::COLOR_BGR2RGB);
    return out;
}

bool CalibrationSingleton::DetectandSaveCorners(std::vector<std::shared_ptr<CameraWrapper>> &cameras, bool fastEnd)
{
    std::vector<std::shared_ptr<CameraWrapper>> badCamPointers;
    return DetectandSaveCorners(cameras, badCamPointers, fastEnd);
}

//badcam is an optional parameter that gets filled with the failing camera, if a cam fails
//fastend indicates that the function should return of one corner detection in one picture fails
bool CalibrationSingleton::DetectandSaveCorners(std::vector<std::shared_ptr<CameraWrapper>> &cameras, std::vector<std::shared_ptr<CameraWrapper>> &badCams, bool fastEnd)
{
    cv::Size CalibrationPattern{Pattern_w, Pattern_h};
    for (auto &cam : cameras)
    {
        cam->CornersListofLists.clear();
        bool allgood = true;
        for (int i = 0; i < cam->FRCalibrationImages.size(); i++)
        {
            auto color = cam->FRCalibrationImages[i];
            // cv::imshow("color full ", cam->FRCalibrationImages[i]);
            // cv::waitKey(0);
            cam->CornersListofLists.push_back(vector<cv::Point2f>());
            // cv::Size np(CalibrationPattern.height,CalibrationPattern.width);
            // bool got_corners = cv::findChessboardCorners(color, np, cam->CornersListofLists[i]);
            bool got_corners = cv::findChessboardCorners(color,
                                                         CalibrationPattern, cam->CornersListofLists[i],
                                                         CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FILTER_QUADS);
            if (got_corners)
            {
                Size winSize = Size(5, 5);
                Size zeroZone = Size(-1, -1);
                TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
                cv::Mat grey_img;
                if(color.channels() == 3){
                    cv::cvtColor(color, grey_img, cv::COLOR_RGB2GRAY);
                } else{
                    grey_img = color;
                }
                cv::cornerSubPix(grey_img, cam->CornersListofLists[i], winSize, zeroZone, criteria);
            }
            else
            {
                // cv::imshow("color full ", cam->FRCalibrationImages[i]);
                // cv::waitKey(0);
                allgood = false;
                if (fastEnd)
                {
                    return false;
                }
            }
        }
        if (!allgood)
        {
            badCams.push_back(cam);
        }
    }
    if (badCams.size() > 0)
    {
        return false;
    }
    cout << "Detected corners in all images! \n";
    return true;
}

void CalibrationSingleton::CalibrateCameras(std::vector<std::shared_ptr<CameraWrapper>> &cameras)
{
    cout << "Starting Calibration \n";
    open3d::pipelines::registration::PoseGraph PoseGraph;
    PoseGraph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode()); //Push back identity node
    for (int i = 0; i < cameras.size() - 1; i++)
    {
        auto cam1 = cameras[i];
        for (int j = i + 1; j < cameras.size(); j++)
        {
            auto cam2 = cameras.at(j);
            bool emptyCornerList = false;
            for (auto &cl : cam1->CornersListofLists)
            {
                if (cl.size() == 0)
                {
                    emptyCornerList = true;
                }
            }
            for (auto &cl : cam2->CornersListofLists)
            {
                if (cl.size() == 0)
                    emptyCornerList = true;
            }
            if (emptyCornerList)
            {
                cout << "Error, could not detect markers in all images, aborting calibration \n";
                return;
            }
            Eigen::Matrix4d edgeTransform = getTransformBetween(cam1, cam2);
            PoseGraph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(i, j, edgeTransform));
            //initialize node poses
            if (i == 0)
            {
                cam2->Extrinsic = edgeTransform.inverse();
                PoseGraph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(cam2->Extrinsic));
            }
        }
    }

    open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria criteria;
    open3d::pipelines::registration::GlobalOptimizationOption option;
    open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt optimization_method;
    open3d::pipelines::registration::GlobalOptimization(PoseGraph, optimization_method, criteria, option);
    // transfer to Cameras
    for (int i = 0; i < cameras.size(); i++)
    {
        cameras[i]->Extrinsic = PoseGraph.nodes_[i].pose_;
        cout << "Extrinsic in pose graph for camera " << cameras[i]->SerialNumber << "\n"
             << cameras[i]->Extrinsic << endl;
        // cameras[i]->SaveCameraPositionToDisk(ResourceFolder + "config/");
    }
    // saveMatrixToDisc(ResourceFolder + "config/", "correctionMatrix.txt", Eigen::Matrix4d::Identity());
}

Eigen::Matrix4d CalibrationSingleton::getTransformBetween(std::shared_ptr<CameraWrapper> sourceCam, std::shared_ptr<CameraWrapper> targetCam)
{
    cv::Size CalibrationPattern{Pattern_w, Pattern_h};
    vector<cv::Point3f> chessboard_corners_world;
    for (int h = 0; h < CalibrationPattern.height; h++)
    {
        for (int w = 0; w < CalibrationPattern.width; w++)
        {
            chessboard_corners_world.emplace_back(
                cv::Point3f{w * SquareLength, h * SquareLength, 0.0});
        }
    }

    vector<vector<cv::Point3f>> chessboard_corners_world_nested_for_cv(sourceCam->CornersListofLists.size(),
                                                                       chessboard_corners_world);

    cv::Matx33d target_calibration_matrix = eigenToOpenCv33(targetCam->CalibrationIntrinsic.intrinsic_matrix_);
    cv::Matx33d source_calibration_matrix = eigenToOpenCv33(sourceCam->CalibrationIntrinsic.intrinsic_matrix_);

    //#########################################################

    cv::Vec3d rt;
    cv::Vec3d tt;
    cv::Matx33d Rt;
    bool success = cv::solvePnP(chessboard_corners_world,
                                targetCam->CornersListofLists.front(),
                                target_calibration_matrix,
                                targetCam->BrownianCalibrationCamDistortionCoeffs,
                                rt,
                                tt, false, 0);
    cv::Rodrigues(rt, Rt); //convert rodriguez vector to rotation matrix
    Eigen::Matrix4d Tt = OpencvTransformationToEigen(Rt, tt);

    cv::Vec3d rs;
    cv::Vec3d ts;
    cv::Matx33d Rs;
    success = cv::solvePnP(chessboard_corners_world,
                           sourceCam->CornersListofLists.front(),
                           source_calibration_matrix,
                           sourceCam->BrownianCalibrationCamDistortionCoeffs,
                           rs,
                           ts, false, 0);
    cv::Rodrigues(rs, Rs); //convert rodriguez vector to rotation matrix
    Eigen::Matrix4d Ts = OpencvTransformationToEigen(Rs, ts);


    //################## kabsch algorithm #########################
    Eigen::MatrixXd ptarget(3, chessboard_corners_world.size());
    Eigen::MatrixXd psource(3, chessboard_corners_world.size());
    for (int i = 0; i < chessboard_corners_world.size(); i++)
    {
        ptarget.block<3, 1>(0, i) = Eigen::Vector3d(chessboard_corners_world[i].x, chessboard_corners_world[i].y, chessboard_corners_world[i].z);
        psource.block<3, 1>(0, i) = Eigen::Vector3d(chessboard_corners_world[i].x, chessboard_corners_world[i].y, chessboard_corners_world[i].z);
        ptarget.block<3, 1>(0, i) = Tt.block<3, 3>(0, 0) * ptarget.block<3, 1>(0, i) + Tt.block<3, 1>(0,3);
        psource.block<3, 1>(0, i) = Ts.block<3, 3>(0, 0) * psource.block<3, 1>(0, i) + Ts.block<3, 1>(0,3);
    }

    // Find the centroids then shift to the origin
    Eigen::Vector3d target_ctr = Eigen::Vector3d::Zero();
    Eigen::Vector3d source_ctr = Eigen::Vector3d::Zero();
    for (int col = 0; col < ptarget.cols(); col++)
    {
        target_ctr += ptarget.col(col);
        source_ctr += psource.col(col);
    }
    target_ctr /= ptarget.cols();
    source_ctr /= ptarget.cols();
    for (int col = 0; col < ptarget.cols(); col++)
    {
        ptarget.col(col) -= target_ctr;
        psource.col(col) -= source_ctr;
    }

    Eigen::Matrix3d Rk = kabsch(psource, ptarget);

    Eigen::Matrix4d Tst_k = Eigen::Matrix4d::Identity();
    Tst_k.block<3,3>(0,0) = Rk;
    Tst_k =  getTrans(target_ctr)* Tst_k* getTrans(source_ctr).inverse();

    //############# debug #################

 
    // auto pcdtarget = make_shared<open3d::geometry::PointCloud>();
    // auto pcdsource = make_shared<open3d::geometry::PointCloud>();

    // for (int i = 0; i < chessboard_corners_world.size(); i++)
    // {
    //     pcdtarget->points_.push_back(Eigen::Vector3d(chessboard_corners_world[i].x, chessboard_corners_world[i].y, chessboard_corners_world[i].z));
    //     pcdsource->points_.push_back(Eigen::Vector3d(chessboard_corners_world[i].x, chessboard_corners_world[i].y, chessboard_corners_world[i].z));
    // }
    // pcdtarget->Transform(Tt);
    // pcdsource->Transform(Ts);
    // pcdtarget->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    // pcdsource->PaintUniformColor(Eigen::Vector3d(0, 0, 1));


    // auto pcdt = open3d::geometry::PointCloud::CreateFromRGBDImage(*targetCam.rgbdImages[0], targetCam.DepthLDTIntrinsic);
    // auto pcds = open3d::geometry::PointCloud::CreateFromRGBDImage(*sourceCam.rgbdImages[0], sourceCam.DepthLDTIntrinsic);

    // open3d::visualization::DrawGeometries({getOrigin(), pcdtarget, pcdt});
    // open3d::visualization::DrawGeometries({getOrigin(), pcdsource, pcds});

    // cout << "Tst * Ts \n" << Tst_k * Ts << endl;
    // cout << "Tt \n" << Tt << endl;
    // cout << "Tst_stereo \n" << Tst_stereo << endl;

    //#################################################################

    if (sourceCam->CameraType == CameraTypes::Kinect && targetCam->CameraType == CameraTypes::Kinect)
    {
        KinectCamera *kinectSource = dynamic_cast<KinectCamera*>(sourceCam.get());
        KinectCamera *kinectTarget = dynamic_cast<KinectCamera*>(targetCam.get());
        if (kinectSource != nullptr && kinectTarget != nullptr)
        {
            //Depth camera is reference camera, so add transformation between depth and color cameras
            auto sourceDepthToColor = kinectSource->KinectCalibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
            Eigen::Matrix4d sourceDepthToColorEigen = arraysToEigenRow(sourceDepthToColor.rotation, sourceDepthToColor.translation);
            sourceDepthToColorEigen.block<3, 1>(0, 3) /= 1000.0;

            auto targetColorToDepth = kinectTarget->KinectCalibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH];
            Eigen::Matrix4d targetColorToDepthEigen = arraysToEigenRow(targetColorToDepth.rotation, targetColorToDepth.translation);
            targetColorToDepthEigen.block<3, 1>(0, 3) /= 1000.0;

            Tst_k = targetColorToDepthEigen * Tst_k * sourceDepthToColorEigen;
        }
    }
    //for intel calibration is done from the infrared cam
    return Tst_k;
}


//for intel cams do auto exposure stuff
// note: this is very specific for intel atm and not multicam ready, also evil pointer casting
void CalibrationSingleton::AdjustCameraExposure(std::vector<std::shared_ptr<CameraWrapper>> &cameras)
{
    //############## start with one image from each camera and adjust exposure if necessary #####################
    int max_it = 8;
    for (int i = 0; i < max_it; i++)
    {
        for (auto &cam : cameras)
        {
            if (cam->CameraType != CameraTypes::Intel){
                return;
            }
            IntelCamera *intelCam = dynamic_cast<IntelCamera*>(cam.get());
            intelCam->clearBuffers();
            intelCam->recordWithoutChecks(1, 0);
        }
        std::vector<std::shared_ptr<CameraWrapper>> badCamPointers;
        if (DetectandSaveCorners(cameras, badCamPointers, false))
        {
            break;
        }
        else
        {
            for (auto &badCam : badCamPointers)
            {
                IntelCamera *badIntelCam = dynamic_cast<IntelCamera*>(badCam.get());
                if (badIntelCam == nullptr)
                {
                    cout << "Error: Dynamic cast to intel camera failed (badcam)." << endl;
                    return;
                }
                //############# actual exposure change ###################
                if (!badIntelCam->IntelDepthSensor.supports(RS2_OPTION_EXPOSURE))
                {
                    cout << "Warning: Intel Sensor doesn't support exposure settings!" << endl;
                    return;
                }
                auto optionRange = badIntelCam->IntelDepthSensor.get_option_range(RS2_OPTION_EXPOSURE);
                int maxStepCount = (optionRange.max - optionRange.min) / optionRange.step;
                int step = (float)maxStepCount / (float)max_it * optionRange.step;
                cout << "Adjusting Exposure for camera " << badIntelCam->SerialNumber << endl;
                if (i == 0) // first run set to min exposure
                {
                    cout << "Info: Setting exposure to " << optionRange.min << endl;
                    badIntelCam->setIntelSensorOption(badIntelCam->IntelDepthSensor, RS2_OPTION_EXPOSURE, optionRange.min);
                }
                else
                {
                    float newExposure = badIntelCam->IntelDepthSensor.get_option(RS2_OPTION_EXPOSURE) + step;
                    if (newExposure <= optionRange.max)
                    {
                        cout << "Info: Setting exposure to " << newExposure << endl;
                        badIntelCam->setIntelSensorOption(badIntelCam->IntelDepthSensor, RS2_OPTION_EXPOSURE, newExposure);
                    }
                    else
                    {
                        cout << "Warning: New Exposure would be higher than max! aborting change." << endl;
                    }
                }
            }
        }
    }
}

// intel specific like AdjustCameraExposure
void CalibrationSingleton::ResetCamerasToAutoExposure(std::vector<std::shared_ptr<CameraWrapper>> &cameras)
{
    if (cameras.front()->CameraType != CameraTypes::Intel) 
    {
        return;
    }

    //##################### reset to auto exposure ###################################
    for (auto &cam : cameras)
    {
        IntelCamera *intelCam = dynamic_cast<IntelCamera*>(cam.get());
        if (intelCam == nullptr)
        {
            cout << "Error: Dynamic cast to intel camera failed." << endl;
            return;
        }
        intelCam->setIntelSensorOption(intelCam->IntelDepthSensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, true);
    }
}

void CalibrationSingleton::SetCameraEmitterOnOff(std::vector<std::shared_ptr<CameraWrapper>> &cameras, bool setOn)
{
        for (auto &cam : cameras)
        {
            if (cam->CameraType == CameraTypes::Intel){
                IntelCamera* intelCam = dynamic_cast<IntelCamera*>(cam.get());
                if (intelCam == nullptr)
                {
                    cout << "Error: Dynamic cast to intel camera failed." << endl;
                    break;
                }
                intelCam->setIntelSensorOption(intelCam->IntelDepthSensor, RS2_OPTION_EMITTER_ENABLED, setOn);
            }
            else if (cam->CameraType == CameraTypes::Ensenso)
            {
                EnsensoCamera* ensensoCam = dynamic_cast<EnsensoCamera*>(cam.get());
                if (ensensoCam == nullptr)
                {
                    cout << "Error: Dynamic cast to ensenso camera failed." << endl;
                    break;
                }
                ensensoCam->setEnsensoSetting(ensensoCam->CaptureSettings, itmProjector, setOn);
            }
        }
}

void CalibrationSingleton::setGravityVectors(std::vector<std::shared_ptr<CameraWrapper>> &cameras)
{
    cv::Size CalibrationPattern{Pattern_w, Pattern_h};
    vector<cv::Point3d> chessboard_corners_world;
    for (int h = 0; h < CalibrationPattern.height; h++)
    {
        for (int w = 0; w < CalibrationPattern.width; w++)
        {
            chessboard_corners_world.emplace_back(
                cv::Point3d{w * SquareLength, h * SquareLength, 0.0});
        }
    }

    for (auto &cam : cameras)
    {
        cv::Matx33f camera_matrix = CalibrationIntrToOpenCVMat(*cam);
        cv::Matx33d R;
        cv::Vec3d t;
        cv::Vec3d r;
        vector<cv::Point2d> cornersList;
        bool success = cv::solvePnP(chessboard_corners_world,
                                    cam->CornersListofLists.front(),
                                    camera_matrix,
                                    cam->BrownianCalibrationCamDistortionCoeffs,
                                    r,
                                    t, false, 0);
        cv::Rodrigues(r, R); //convert rodriguez vector to rotation matrix

        Eigen::Matrix4d trans = OpencvTransformationToEigen(R, t);
        if(cam->CameraType == CameraTypes::Kinect)
        {
            KinectCamera* kinectCam = dynamic_cast<KinectCamera*>(cam.get());
            if (kinectCam == nullptr)
            {
                cout << "Error: Dynamic cast to kinect camera failed." << endl;
                break;
            }
            auto colorToDepth = kinectCam->KinectCalibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH];
            Eigen::Matrix4d colorToDepthEigen = arraysToEigenRow(colorToDepth.rotation, colorToDepth.translation);
            cam->GravityVector = colorToDepthEigen.block<3, 3>(0, 0) * trans.block<3, 3>(0, 0) * Eigen::Vector3d(0, 0, 1);
        }
        else
        {
            cam->GravityVector = trans.block<3, 3>(0, 0) * Eigen::Vector3d(0, 0, 1);
        }
        t = R*t;
        cam->CameraHeight = t(2);
        cam->GravityVector.normalize();
        // auto ls = std::make_shared<open3d::geometry::LineSet>();
        // Eigen::Vector3d zero(0.0, 0.0, 0.0);
        // ls->points_.push_back(zero);
        // ls->points_.push_back(cam->GravityVector);
        // ls->lines_.push_back(Eigen::Vector2i(0, 1));
        // auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*cam->rgbdImages.front(),cam->DepthLDTIntrinsic);
        // open3d::visualization::DrawGeometries({getOrigin(),ls,pcd});
    }
    // for (auto &cam : cameras)
    // {
    //     cam->SaveGravityVectorToDisk(ResourceFolder + "config/");
    // }
}