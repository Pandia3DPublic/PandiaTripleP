#include "ScanningSingleton.h"
#include "qualityUtil.h"
// #include <teaser/registration.h>
#include "utils/3DFunctions.h"
#include "utils/miscFunctions.h"
#include "utils/imageFunctions.h"
#include "utils/conversions.h"
#include "utils/basicMath.h"
#include "utils/PandiaTimer.h"
#include "utils/fileIO.h"
#include "utils/vishelper.h"
#include "utils/jsonUtil.h"
#include "cameras/IntelCamera.h"

using namespace std;
ScanningSingleton::ScanningSingleton()
{
    CurrentPCD = make_shared<open3d::geometry::PointCloud>();
    CurrentDownsampledPCD = make_shared<open3d::geometry::PointCloud>();
    CurrentVoxelGrid = make_shared<open3d::pipelines::integration::ScalableTSDFVolume>(0.004, 0.015, open3d::pipelines::integration::TSDFVolumeColorType::RGB8, 16, 1);
    ReferenceScanPCD = make_shared<open3d::geometry::PointCloud>();
    OldReferenceScanPCD = make_shared<open3d::geometry::PointCloud>();
    ReferencePCD = make_shared<open3d::geometry::PointCloud>();
    SmallReferencePCD = make_shared<open3d::geometry::PointCloud>();
    ReferenceMesh = make_shared<open3d::geometry::TriangleMesh>();
    CalibrationManager = make_shared<CalibrationSingleton>();
    OldPCD = make_shared<open3d::geometry::PointCloud>();
    OldDownsampledPCD = make_shared<open3d::geometry::PointCloud>();
}

ScanningSingleton::~ScanningSingleton()
{
}

void ScanningSingleton::clearCurrentStructures()
{
    CurrentPCD->Clear();
    CurrentDownsampledPCD->Clear();
    CurrentVoxelGrid->Reset();
}

void ScanningSingleton::clearReferenceModel()
{
    ReferenceMesh->Clear();
    ReferencePCD->Clear();
    SmallReferencePCD->Clear();
}

bool ScanningSingleton::ReferenceModelSet()
{
    if (ReferencePCD->IsEmpty() || SmallReferencePCD->IsEmpty())
    {
        return false;
    }
    return true;
}

// WARNING: THIS DOES NOT WORK FOR OCMI
std::vector<std::vector<Eigen::Vector3d>> ScanningSingleton::getLatestDistanceColorsGradient()
{
    // return getDistanceColorsGradientGeneral(distances, ColorThresholdMin, ColorThresholdMax);
    std::vector<std::vector<Eigen::Vector3d>> pcdColorVector;
    for (auto &cam : Cameras)
    {
        auto transpcd = *cam->CroppedPcds.back();
        transpcd.Transform(cam->T_camToRef.back());
        auto distances = transpcd.ComputePointCloudDistance(*ReferencePCD);
        // auto camcolor = getDistanceColorsGradientGeneral(distances, cam->ColorThresholdMin, cam->ColorThresholdMax);
        pcdColorVector.emplace_back(getDistanceColorsGradientGeneral(distances, cam->ColorThresholdMin, cam->ColorThresholdMax));
    }
    return pcdColorVector;
}

// the following points on the object surface can be marked as missing:
// 1. At least one camera gets data for the point. This can be checked by projecting the point on a
// camera screen and checking if the pixel depht value is non zero.
// 2. The depth value strongly diverges from the should be value.
// Tactic: For each cam generate a reference pcd depth buffer. For each non zero pixel in the
// buffer check if the corresponding scannedrgbd pixel is non zero and diverges above a certain
// threshold. If it does, mark the corresponding point on the reference pcd as missing / add the
// corresponding point to the missing pcd.

// WARNING: depth images from pcds can be ambigous! some depth values will depend
// on the order in which points were projected, if multiple points hit the same image!
shared_ptr<open3d::geometry::PointCloud> ScanningSingleton::getMissingPoints()
{
    PandiaTimer timer;
    auto missingPoints = make_shared<open3d::geometry::PointCloud>();
    // vector<double> distances;

    for (auto &cam : Cameras)
    {
        for (int k = 0; k < cam->rgbdImages.size(); k++)
        {
            auto missingScanPoints = make_shared<open3d::geometry::PointCloud>();
            Eigen::MatrixXd depth_buffer;
            if (!ReferenceMesh->IsEmpty())
            {
                auto transformedRefMesh = *ReferenceMesh;
                Eigen::Matrix4d refToCamTrans = cam->T_camToRef[k].inverse();
                transformedRefMesh.Transform(refToCamTrans);
                depth_buffer = cam->getFOVDepthBuffer(transformedRefMesh);
            }
            else
            {
                auto transformedRefPcd = *ReferencePCD;
                Eigen::Matrix4d refToCamTrans = cam->T_camToRef[k].inverse();
                transformedRefPcd.Transform(refToCamTrans);
                depth_buffer = cam->getDepthBufferFromPCD(transformedRefPcd);
            }

            // auto bufferpcd = cam->pcdFromDepth(*EigenToO3DDepthImage(depth_buffer));
            // open3d::visualization::DrawGeometries({EigenToO3DDepthImage(depth_buffer)});
            // open3d::visualization::DrawGeometries({bufferpcd, cam->CroppedPcds[k], getOrigin()});
            auto rgbd = cam->getCroppedRGBDImageWithGround(cam->rgbdImages[k]);
            open3d::geometry::KDTreeFlann croppedPcdTree(*cam->CroppedPcds[k]);
            for (int i = 0; i < depth_buffer.rows(); i++)
            {
                for (int j = 0; j < depth_buffer.cols(); j++)
                {
                    double &d = depth_buffer(i, j);
                    double dm = *rgbd->depth_.PointerAt<float>(j, i);
                    if (d != 0 && dm != 0)
                    {
                        if (dm - d > cam->ColorThresholdMax) // the measured value must be behind the db value
                        {
                            Eigen::Vector3d p = cam->PixeltoPoint(i, j, d);
                            // p = cam->Extrinsic.block<3, 3>(0, 0) * p + cam->Extrinsic.block<3, 1>(0, 3);
                            // since we look diagonally, we get larger than reasonable values for some pixel pairs
                            //###normal filtering, mathematically meaningfull ###
                            //  Eigen::Vector3d z(0, 0, 1);
                            //  Eigen::Vector3d n(cam->nx(i,j), cam->ny(i,j), cam->nz(i,j));
                            //  double alpha = std::acos(z.dot(n));
                            //  if (alpha > M_PI_2) {alpha=M_PI - alpha;}
                            //  double l = ColorThresholdMax / std::cos(alpha);
                            //  if(dm -d > l){
                            //      missingScanPoints->points_.push_back(p);
                            //  }
                            //### alternative radius filtering #####
                            std::vector<int> inds;
                            std::vector<double> dists;
                            int n = croppedPcdTree.SearchKNN(p, 1, inds, dists);
                            if (dists.front() * 100 > cam->ColorThresholdMax)
                            {
                                missingScanPoints->points_.push_back(p);
                            }
                        }
                    }
                }
            }

            missingScanPoints->EstimateNormals();
            applyNormalFilter(missingScanPoints);
            double radius = 5 * cam->ApproxVoxelSize;
            auto res = missingScanPoints->RemoveRadiusOutliers((int)((radius * M_PI * M_PI / cam->ApproxVoxelSize) / 5.0), radius, false);
            missingScanPoints = std::get<0>(res);

            missingScanPoints->Transform(cam->T_camToRef[k]);
            *missingPoints = *missingPoints + *missingScanPoints;
        }
    }
    //###debug func
    // missingPoints->colors_ = getDistanceColorsGradientGeneral(distances,10* ColorThresholdMin,10*ColorThresholdMax);
    // auto transmesh = make_shared<open3d::geometry::TriangleMesh>(*ReferenceMesh);
    // Eigen::Matrix4d refToCamTrans = Cameras.front()->T_camToRef.inverse();
    // transmesh->Transform(refToCamTrans);
    // transmesh->ComputeTriangleNormals();
    // transmesh->ComputeVertexNormals();
    // open3d::visualization::DrawGeometries({getOrigin(), transmesh, missingPoints});

    missingPoints->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
    cout << "Time in getMissingPoints: " << timer.seconds() << endl;
    return missingPoints;
}

// note that at high angles we get missing point coloration due to depth image resolution (2 points one bucket)
std::shared_ptr<open3d::geometry::PointCloud> ScanningSingleton::getVisibilityPCD()
{
    auto out = make_shared<open3d::geometry::PointCloud>(*ReferencePCD);
    // out->PaintUniformColor(Eigen::Vector3d(0, 0, 1)); // blue
    // for (auto &cam : Cameras)
    // {

    //     auto transformedRefMesh = *ReferenceMesh;
    //     Eigen::Matrix4d refToCamTrans = cam->T_camToRef.inverse();
    //     transformedRefMesh.Transform(refToCamTrans);
    //     Eigen::MatrixXd depth_buffer = cam->getFOVDepthBuffer(transformedRefMesh);

    //     auto depth_image = EigenToO3DDepthImage(depth_buffer);
    //     depth_buffer = O3DDepthtoEigen(depth_image);
    //     // open3d::visualization::DrawGeometries({depth_image});
    //     // auto d2 = EigenToO3DDepthImage(depth_buffer);
    //     // open3d::visualization::DrawGeometries({d2});
    //     // open3d::visualization::DrawGeometries({EigenDepthToPCD(depth_buffer,cam.DepthLDTIntrinsic), getOrigin()});

    //     double epsilon = 1e-3; // in case point lies exactly on mesh
    //     for (int i = 0; i < ReferencePCD->points_.size(); i++)
    //     {
    //         Eigen::Vector3d pt = refToCamTrans.block<3, 3>(0, 0) * ReferencePCD->points_[i] + refToCamTrans.block<3, 1>(0, 3);
    //         Eigen::Vector2i ind = cam->PointtoPixel(pt);
    //         if (cam->InsideDepthImage(ind))
    //         {
    //             if (pt(2) <= depth_buffer(ind(0), ind(1)) + epsilon)
    //             {
    //                 out->colors_[i] = Eigen::Vector3d(0, 1, 0); // green
    //             }
    //         }
    //     }
    // }
    return out;
}

std::shared_ptr<open3d::geometry::PointCloud> ScanningSingleton::getCalibrationPCD()
{
    auto out = make_shared<open3d::geometry::PointCloud>();
    for (auto &cam : Cameras)
    {
        auto tmp = make_shared<open3d::geometry::PointCloud>();
        cam->clearBuffers();
        cam->record(1);
        // want about 100k points each
        int samplerate = round((float)cam->Pcds.front()->points_.size() / 100000.0f);
        if (samplerate >= 1)
            tmp = cam->Pcds.front()->UniformDownSample(samplerate);
        else
            *tmp = *cam->Pcds.front();
        tmp->Transform(cam->Extrinsic);
        *out = *out + *tmp;
    }
    return out;
}

void ScanningSingleton::RecolorCurrentPCD(const std::vector<Eigen::Vector3d> &newColors)
{
    if (CurrentPCD->colors_.size() == newColors.size())
        CurrentPCD->colors_ = newColors;
    else
    {
        cout << "Recolor failed as color vector size differs." << endl;
    }
}

bool ScanningSingleton::ComputeCameraPositions()
{
    try
    {
        bool success = false;
        CalibrationManager->SetCameraEmitterOnOff(Cameras, false);
        CalibrationManager->AdjustCameraExposure(Cameras);
        for (auto &cam : Cameras)
        {
            cam->clearBuffers();
            if (cam->CameraType == CameraTypes::Zivid || cam->CameraType == CameraTypes::Phoxi)
            {
                cam->record(1, 0);
            }
            else if (cam->CameraType == CameraTypes::Intel)
            {
                IntelCamera *intelCam = dynamic_cast<IntelCamera *>(cam.get());
                if (intelCam == nullptr)
                {
                    cout << "Error in casting to intel camera" << endl;
                    break;
                }
                intelCam->recordWithoutChecks(5, 0);
            }
            else
            {
                cam->record(5, 5);
            }
            // cam->AlignUndistortandConvertO3D();
        }
        // perform stereo calibration
        // 1.Search for corners in all images and save them inside the cameras
        bool foundAllCorners = CalibrationManager->DetectandSaveCorners(Cameras, true);
        // 2.Pass these to the actual calibrate function and save the extrinsic inside the cameras
        if (foundAllCorners)
        {
            CalibrationManager->CalibrateCameras(Cameras);
            CalibrationManager->setGravityVectors(Cameras);
            if (!CalibrationManager->StereoErrors.empty())
            {
                double err = 0;
                for (double &e : CalibrationManager->StereoErrors)
                {
                    err += e;
                }
                err /= CalibrationManager->StereoErrors.size();
                cout << "Stereo calibrate avg error: " << err << endl;
            }
            success = true;
        }
        else
        {
            cout << "Calibration failed \n";
            success = false;
        }
        CalibrationManager->StereoErrors.clear();
        CalibrationManager->ResetCamerasToAutoExposure(Cameras);
        CalibrationManager->SetCameraEmitterOnOff(Cameras, true);
        // record again with emitter for pcd
        // for (auto &cam : Cameras)
        // {
        //     cam->clearBuffers();
        //     cam->record(1, 0);
        //     // cam->AlignUndistortandConvertO3D();
        // }
        return success;
    }
    catch (const std::exception &e)
    {
        cout << "Exception caught in ComputeCameraPositions" << endl;
        // std::cerr << e.what() << '\n';
        return false;
    }
}

bool ScanningSingleton::RecordAndFilterFrames()
{
    // record images
    // WARNING IF WE RECORD MULTIPLE TIMES WE NEED TO CHANGE PERFORMREFINEALINGCAD
    bool recordingSuccess = false;
    for (auto cam : Cameras)
    {
        recordingSuccess = cam->record(1);
        cam->CroppedPcds.push_back(cam->getCroppedPcd(cam->Pcds.back()));
        if (!cam->CroppedPcds.back()->HasNormals())
        {
            cam->CroppedPcds.back()->EstimateNormals();
        }
        // if (cam->CameraType == CameraTypes::Zivid)
        // {
        //     double r = 0.005;
        //     double nr = 2e6 * M_PI * r * r; // 2 million points per square meter
        //     auto res = cam->Pcds.back()->RemoveRadiusOutliers(nr / 4, r, false); //note: this can take up to 9 seconds!
        //     cam->Pcds.back() = std::get<0>(res);
        // }

        // open3d::visualization::DrawGeometries({getOrigin(),cam->Pcds.back()});
        // open3d::visualization::DrawGeometries({cam->rgbdImages.back()});
        // applyNormalFilter(cam->rgbdImages.back()->depth_,*cam,cam->Pcds.back());
        // applyNormalFilter(cam->Pcds.back());
        // todo more pictures for ensenso would be better
        //     if (cam->CameraType == CameraTypes::Phoxi ||
        //         cam->CameraType == CameraTypes::Zivid ||
        //         cam->CameraType == CameraTypes::Ensenso) //todo more images would be better
        //     {
        //         recordingSuccess = cam->record(1);
        //     }
        //     if (cam->CameraType == CameraTypes::Intel ||
        //         cam->CameraType == CameraTypes::Kinect)
        //     {
        //         recordingSuccess = cam->record(10);
        //     }
    }
    if (!recordingSuccess)
    {
        cout << "Error: Recording failed \n";
        return false;
    }

    return true;
}

void ScanningSingleton::PerformRefineAlignWithCAD()
{
    // cout << "before realign \n";
    // open3d::visualization::DrawGeometries({getOrigin(), CurrentPCD,ReferencePCD});
    clearCurrentStructures();
    double downsample_voxel_size = 0.005;
    int icp_maxIterations = 60;
    double icp_maxCorsDistance = 0.015;

    auto json = readJsonFromDisk("functionConfigs/PerformRefineAlignWithCAD.json");
    setValueFromJson(json, "icp_maxIterations", icp_maxIterations);
    setValueFromJson(json, "icp_maxCorsDistance", icp_maxCorsDistance);
    json = readJsonFromDisk("functionConfigs/StartScan.json");
    setValueFromJson(json, "downsample_voxel_size", downsample_voxel_size);

    // note: here we realing only the new pcds. WARNING NOT WORKING FOR multiple frames in once record
    for (auto &cam : Cameras)
    {
        for (int i = 0; i < cam->CroppedPcds.size(); i++)
        {

            Eigen::MatrixXd depth_buffer;
            if (!ReferenceMesh->IsEmpty())
            {
                auto transformedRefMesh = *ReferenceMesh;
                Eigen::Matrix4d refToCamTrans = cam->T_camToRef[i].inverse();
                transformedRefMesh.Transform(refToCamTrans);
                depth_buffer = cam->getFOVDepthBuffer(transformedRefMesh);
            }
            else
            {
                auto transformedRefPcd = *ReferencePCD;
                Eigen::Matrix4d refToCamTrans = cam->T_camToRef[i].inverse();
                transformedRefPcd.Transform(refToCamTrans);
                depth_buffer = cam->getDepthBufferFromPCD(transformedRefPcd);
            }
            auto referenceProjectionPcd = cam->EigenDepthToPCD(depth_buffer);
            referenceProjectionPcd->Transform(cam->T_camToRef[i]);
            cam->T_camToRef[i] = refineRegistrationICP(cam->CroppedPcds[i], referenceProjectionPcd, cam->T_camToRef[i], icp_maxIterations, icp_maxCorsDistance);
            auto transformedPCD = *cam->CroppedPcds[i];
            transformedPCD.Transform(cam->T_camToRef[i]);
            *CurrentPCD = *CurrentPCD + transformedPCD;
        }
    }
    CurrentDownsampledPCD = CurrentPCD->VoxelDownSample(downsample_voxel_size);
    // Correct the final fitness by running one iteartion of icp
    refineRegistrationICP(CurrentDownsampledPCD, ReferencePCD, Eigen::Matrix4d::Identity(), FinalFitness, 1, icp_maxCorsDistance);
}

bool ScanningSingleton::StartScan()
{
    try
    {
        PandiaTimer ScanTimer;
        // json variables
        double downsample_voxel_size = 0.005;
        int icp_CAD_maxIterations = 100;
        double icp_CAD_maxCorsDistance = 0.01;
        double PCA_EarlyOutFitnessThres = 0.8;

        auto json = readJsonFromDisk("functionConfigs/StartScan.json");
        setValueFromJson(json, "downsample_voxel_size", downsample_voxel_size);
        setValueFromJson(json, "icp_CAD_maxIterations", icp_CAD_maxIterations);
        setValueFromJson(json, "icp_CAD_maxCorsDistance", icp_CAD_maxCorsDistance);
        setValueFromJson(json, "PCA_EarlyOutFitnessThres", PCA_EarlyOutFitnessThres);

        // reset ocmi variables
        OCMILastAngle = 0;
        OCMIScanIteration = 0;
        FinalFitness = 0;
        OldFinalFitness = 0;
        if (!ReferenceModelSet())
        {
            cout << "Warning: Referencemodel is not set." << endl;
            return false;
        }
        clearCurrentStructures();
        OldPCD->Clear();
        OldDownsampledPCD->Clear();
        T_OCMIs.clear();

        //######## Record images #############
        for (auto &cam : Cameras)
        {
            cam->clearBuffers();
        }
        if (!RecordAndFilterFrames())
            return false;

        cout << "StartScan: after record " << ScanTimer.seconds() << endl;
        for (auto &cam : Cameras)
        {
            auto transformedPCD = *cam->CroppedPcds.back();
            transformedPCD.Transform(cam->Extrinsic);
            *CurrentPCD = *CurrentPCD + transformedPCD;
        }
        CurrentDownsampledPCD = CurrentPCD->VoxelDownSample(downsample_voxel_size);
        cout << "StartScan: after currentpcd building " << ScanTimer.seconds() << endl;

        //############ Align Scan with CAD Modell ##########################
        // Fitness is given by #cors/#source points. Unlike in the documentation!

        Eigen::Matrix4d T_init = getIterativePCABasedAlignment(CurrentPCD, ReferencePCD, FinalFitness);
        // Eigen::Matrix4d T_init = getIterativePCABasedAlignment(CurrentDownsampledPCD, ReferencePCD, FinalFitness);
        cout << "StartScan: after icp pca " << ScanTimer.seconds() << endl;
        // early out if pca align is bad
        if (FinalFitness < PCA_EarlyOutFitnessThres)
        {
            cout << "Info: Speeding up further alignment as PCA fitness is too low: " << FinalFitness << endl;
            icp_CAD_maxIterations = 1;
        }
        Eigen::Matrix4d T_FirstCamToRef = refineRegistrationICP(CurrentDownsampledPCD, ReferencePCD, T_init, FinalFitness, icp_CAD_maxIterations, icp_CAD_maxCorsDistance);
        cout << "StartScan: after icp T_FirstCamToRef " << ScanTimer.seconds() << endl;
        CurrentPCD->Transform(T_FirstCamToRef);
        CurrentDownsampledPCD->Transform(T_FirstCamToRef);
        //############# set camera transformations #################
        T_OCMIs.push_back(Eigen::Matrix4d::Identity());
        for (auto &cam : Cameras)
        {
            cam->T_camToRef.push_back(T_FirstCamToRef * T_OCMIs.front() * cam->Extrinsic);
        }
        //########### further options #####################
        if (RefineAlignWithCAD)
        {
            PerformRefineAlignWithCAD();
            cout << "StartScan: after icp PerformRefineAlignWithCAD " << ScanTimer.seconds() << endl;
        }

        cout << "Initial scan time was " << ScanTimer.seconds() << " seconds." << endl;
    }
    catch (const std::exception &e)
    {
        cout << "Exception caught in StartScan" << endl;
        // std::cerr << e.what() << '\n';
        return false;
    }
    return true;
}

bool ScanningSingleton::AddScan()
{
    std::cout << "in add scan \n";
    try
    {
        PandiaTimer ScanTimer;
        // json variables
        double downsample_voxel_size = 0.005;
        int icp_Pcd_maxIterations = 60;
        double icp_Pcd_maxCorsDistance = 0.015;
        int icp_CAD_maxIterations = 100;
        double icp_CAD_maxCorsDistance = 0.01;
        double PCA_EarlyOutFitnessThres = 0.8;

        auto json = readJsonFromDisk("functionConfigs/AddScan.json");
        setValueFromJson(json, "icp_Pcd_maxIterations", icp_Pcd_maxIterations);
        setValueFromJson(json, "icp_Pcd_maxCorsDistance", icp_Pcd_maxCorsDistance);
        setValueFromJson(json, "icp_CAD_maxIterations", icp_CAD_maxIterations);
        setValueFromJson(json, "icp_CAD_maxCorsDistance", icp_CAD_maxCorsDistance);
        setValueFromJson(json, "PCA_EarlyOutFitnessThres", PCA_EarlyOutFitnessThres);

        json = readJsonFromDisk("functionConfigs/StartScan.json");
        setValueFromJson(json, "downsample_voxel_size", downsample_voxel_size);

        if (!ReferenceModelSet())
        {
            cout << "Warning: Referencemodel is not set." << endl;
            return false;
        }
        if (CurrentPCD->IsEmpty())
        {
            cout << "Warning: You must start a scan first before adding a scan!" << endl;
            return false;
        }
        //######## Record and filter rgbd images #############
        if (!RecordAndFilterFrames())
            return false;

        auto lastScanPcd = make_shared<open3d::geometry::PointCloud>();
        for (auto &cam : Cameras)
        {
            auto transformedPCD = *cam->CroppedPcds.back();
            transformedPCD.Transform(Cameras.front()->T_camToRef.front() * cam->Extrinsic);
            *lastScanPcd = *lastScanPcd + transformedPCD;
        }

        auto lastScanPcdDownsampled = lastScanPcd->VoxelDownSample(downsample_voxel_size);

        *OldPCD = *CurrentPCD;
        *OldDownsampledPCD = *CurrentDownsampledPCD;

        //############ Align new scan with previous pcd ##########################
        // auto rotationOffset = getRotationMatrixFromVectorAndAngle(gravity, toRadians(-30));
        // Eigen::Matrix4d T_init = Tcenter * rotationOffset * Tcenter.inverse();
        Eigen::Matrix4d T_init = getIterativeAlignmentforOCMI(lastScanPcdDownsampled, CurrentDownsampledPCD, Cameras.front()->GravityVector, OCMILastAngle);
        Eigen::Matrix4d T_OCMI = refineRegistrationICP(lastScanPcdDownsampled, CurrentDownsampledPCD, T_init, icp_Pcd_maxIterations, icp_Pcd_maxCorsDistance);

        lastScanPcd->Transform(T_OCMI);
        lastScanPcdDownsampled->Transform(T_OCMI);

        //############ Add to CurrentPCD ############
        *CurrentPCD = *CurrentPCD + *lastScanPcd;
        *CurrentDownsampledPCD = *CurrentDownsampledPCD + *lastScanPcdDownsampled;

        //########## Align combined scan to cad model #############
        OldFinalFitness = FinalFitness;
        Eigen::Matrix4d T_PCA = getIterativePCABasedAlignment(CurrentDownsampledPCD, SmallReferencePCD, FinalFitness);
        if (FinalFitness < PCA_EarlyOutFitnessThres)
        {
            cout << "Info: Speeding up further alignment as PCA fitness is too low: " << FinalFitness << endl;
            icp_CAD_maxIterations = 1;
        }
        Eigen::Matrix4d T_AlginCorrection = refineRegistrationICP(CurrentDownsampledPCD, ReferencePCD, T_PCA, FinalFitness, icp_CAD_maxIterations, icp_CAD_maxCorsDistance);
        //############# set camera transformations #################
        T_OCMIs.push_back(T_OCMI);
        for (auto &cam : Cameras)
        {
            cam->T_camToRef.push_back(Eigen::Matrix4d::Identity());
            for (int i = 0; i < cam->T_camToRef.size(); i++)
            {
                cam->T_camToRef[i] = T_AlginCorrection * T_OCMIs[i] * Cameras.front()->T_camToRef.front() * cam->Extrinsic;
            }
        }
        // ########### further options #####################
        if (RefineAlignWithCAD)
        {
            PerformRefineAlignWithCAD(); // further align all camera scanned pcds to cad
        }

        // if (OCMIScanIteration == 0)
        //     LatestPCD->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
        // if (OCMIScanIteration == 1)
        //     LatestPCD->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
        // if (OCMIScanIteration == 2)
        //     LatestPCD->PaintUniformColor(Eigen::Vector3d(1, 1, 0));
        // if (OCMIScanIteration == 3)
        //     LatestPCD->PaintUniformColor(Eigen::Vector3d(0, 1, 1));
        // if (OCMIScanIteration == 4)
        //     LatestPCD->PaintUniformColor(Eigen::Vector3d(1, 0, 1));
        // if (OCMIScanIteration == 5)
        //     LatestPCD->PaintUniformColor(Eigen::Vector3d(1, 0, 0));

        // auto debugpcd = make_shared<open3d::geometry::PointCloud>();
        // for (auto &cam: Cameras)
        // {
        //     *debugpcd = *cam->CroppedPcds.back();
        //     debugpcd->Transform(cam->T_camToRef.back());
        //     open3d::visualization::DrawGeometries({debugpcd, ReferencePCD, getOrigin()});
        // }

        OCMIScanIteration++;
        OCMIScanRevertable = true;
        cout << "Scan time was " << ScanTimer.seconds() << " seconds." << endl;
    }
    catch (const std::exception &e)
    {
        cout << "Exception caught in AddScan" << endl;
        // std::cerr << e.what() << '\n';
        return false;
    }
    return true;
}

bool ScanningSingleton::RevertLastScan()
{
    try
    {
        if (OCMIScanRevertable)
        {
            OCMIScanIteration--;
            OCMIScanRevertable = false;
            *CurrentPCD = *OldPCD;
            *CurrentDownsampledPCD = *OldDownsampledPCD;
            FinalFitness = OldFinalFitness;
            T_OCMIs.pop_back();
            for (auto &cam : Cameras)
            {
                cam->rgbdImages.pop_back();
                cam->Pcds.pop_back();
                cam->CroppedPcds.pop_back();
                cam->T_camToRef.pop_back();
            }
            return true;
        }
        else
        {
            return false;
        }
    }
    catch (const std::exception &e)
    {
        cout << "Exception caught in RevertLastScan" << endl;
        // std::cerr << e.what() << '\n';
        return false;
    }
}

bool ScanningSingleton::StartReferenceScan()
{
    try
    {
        ReferenceScanRevertable = false;
        ReferenceScanPCD->Clear();
        //######## Record and filter rgbd images #############
        for (auto &cam : Cameras)
        {
            cam->clearBuffers();
            cam->T_AddedToRef.push_back(Eigen::Matrix4d::Identity());
        }
        if (!RecordAndFilterFrames())
            return false;

        for (auto &cam : Cameras)
        {
            auto transformedPCD = *cam->CroppedPcds.back();
            transformedPCD.Transform(cam->Extrinsic);
            *ReferenceScanPCD = *ReferenceScanPCD + transformedPCD;
        }
        // todo this is a hack
        if (Cameras.size() == 2)
        {
            Eigen::Matrix4d T = refineRegistrationICP(Cameras.back()->CroppedPcds.back(), Cameras.front()->CroppedPcds.back(), Cameras.back()->Extrinsic);
            *ReferenceScanPCD = *Cameras.front()->CroppedPcds.back();
            auto transformedPCD = *Cameras.back()->CroppedPcds.back();
            transformedPCD.Transform(T);
            *ReferenceScanPCD = *ReferenceScanPCD + transformedPCD;
        }

        // ReferenceScanPCD->Translate(-ReferenceScanPCD->GetCenter());

    }
    
    catch (const std::exception &e)
    {
        cout << "Exception caught in StartReferenceScan" << endl;
        // std::cerr << e.what() << '\n';
        return false;
    }
    return true;
}

bool ScanningSingleton::AddReferenceScan()
{
    
    if (ReferenceScanPCD->points_.size() == 0)
    {
        return StartReferenceScan();
    }
    //back up for last scan revert
    *OldReferenceScanPCD = *ReferenceScanPCD;

    // implementation for one camera, without graph optimization
    auto &cam = Cameras.front();
    RecordAndFilterFrames();
    // cam->record(1);
    // icp
    // Eigen::Matrix4d T =  getIterativeAlignmentforOCMI(cam->CroppedPcds.back(), ReferenceScanPCD, cam->GravityVector,LastAddReferenceScanAngle);
    Eigen::Matrix4d T = getIterativePCABasedAlignment(cam->CroppedPcds.back(), ReferenceScanPCD);
    cam->T_AddedToRef.push_back(T);
    // add scan
    auto tmppcd = make_shared<open3d::geometry::PointCloud>(*cam->CroppedPcds.back());
    tmppcd->Transform(T);
    *ReferenceScanPCD = *ReferenceScanPCD + *tmppcd;
    // voxel downsample to native camera resolution
    ReferenceScanPCD->VoxelDownSample(cam->ApproxVoxelSize);
    ReferenceScanRevertable = true;
    return true;
}

bool ScanningSingleton::RevertLastReferenceScan()
{
    if (ReferenceScanRevertable)
    {
        ReferenceScanRevertable = false;
        *ReferenceScanPCD = *OldReferenceScanPCD;
        for (auto &cam : Cameras)
        {
            cam->rgbdImages.pop_back();
            cam->Pcds.pop_back();
            cam->CroppedPcds.pop_back();
            cam->T_AddedToRef.pop_back();
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool ScanningSingleton::RefineReferenceScan()
{
    open3d::pipelines::registration::PoseGraph PoseGraph;
    PoseGraph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode()); // Push back identity node
    auto &cam = Cameras.front();
    for (int i = 0; i < cam->CroppedPcds.size() - 1; i++)
    {
        for (int j = i + 1; j < cam->CroppedPcds.size(); j++)
        {
            auto pcdi = cam->CroppedPcds[i];
            auto pcdj = cam->CroppedPcds[j];
            // transform j to i
            Eigen::Matrix4d T_j_to_i = cam->T_AddedToRef[i].inverse() * cam->T_AddedToRef[j];
            T_j_to_i = refineRegistrationICP(pcdj, pcdi, T_j_to_i, 30, 3 * cam->ApproxVoxelSize);
            PoseGraph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(j, i, T_j_to_i));
            // initialize node poses
            if (i == 0)
            {
                PoseGraph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(T_j_to_i));
            }
        }
    }

    open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria criteria;
    open3d::pipelines::registration::GlobalOptimizationOption option;
    open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt optimization_method;
    open3d::pipelines::registration::GlobalOptimization(PoseGraph, optimization_method, criteria, option);


    ReferenceScanPCD->Clear();
    for (int i = 0; i < cam->CroppedPcds.size(); i++)
    {
        auto tmppcd = make_shared<open3d::geometry::PointCloud>(*cam->CroppedPcds[i]);
        tmppcd->Transform(PoseGraph.nodes_[i].pose_);
        *ReferenceScanPCD = *ReferenceScanPCD + *tmppcd;
    }
    ReferenceScanPCD->VoxelDownSample(2*cam->ApproxVoxelSize);
    return true;
}

bool ScanningSingleton::SetReferenceModel(std::shared_ptr<open3d::geometry::TriangleMesh> mesh)
{
    // open3d::visualization::DrawGeometries({mesh, getOrigin()});

    double pointsPercm2 = 1e-4 / (ReferenceVoxelResolution * ReferenceVoxelResolution);
    double pointsPercm2Small = pointsPercm2 / 10; // todo remove
    double uniqueModelNumber = 0;
    *ReferenceMesh = *mesh;
    for (int i = 0; i < ReferenceMesh->vertices_.size() && i < 100; i++)
    {
        uniqueModelNumber = uniqueModelNumber + std::pow(ReferenceMesh->vertices_[i](0), 2) + std::pow(ReferenceMesh->vertices_[i](1), 2) + std::pow(ReferenceMesh->vertices_[i](2), 2);
    }
    uniqueModelNumber /= 1000;
    uniqueModelNumber *= 1000; // round for numeric stability

    ReferenceMesh->Translate(-ReferenceMesh->GetCenter());
    ReferenceMesh->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    // mesh->Scale(0.001, mesh->GetCenter()); //todo do not scale
    if (!ReferenceMesh->HasTriangleNormals())
    {
        ReferenceMesh->ComputeTriangleNormals();
    }
    ReferenceMesh->NormalizeNormals();

    string pcdstring = ResourceFolder + to_string(uniqueModelNumber) + ".pcd";
    string pcdstring_small = ResourceFolder + to_string(uniqueModelNumber) + "_small" + ".pcd";
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Error); // get rid of read pcd warnings
    ReferencePCD = readPcd(pcdstring);
    SmallReferencePCD = readPcd(pcdstring_small);
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Warning);

    if (ReferencePCD->IsEmpty())
    {
        cout << "Sampling points for pcd..." << endl;
        auto pcd = ReferenceMesh->SamplePointsPoissonDisk(pointsPercm2 * 10000 * ReferenceMesh->GetSurfaceArea());
        ReferencePCD = getScannablePcd(ReferenceMesh, pcd);
        ReferencePCD->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
        open3d::io::WritePointCloudOption params;
        open3d::io::WritePointCloudToPCD(pcdstring, *ReferencePCD, params);
        cout << "Done." << endl;
    }
    if (SmallReferencePCD->IsEmpty())
    {
        cout << "Sampling points for pcd small..." << endl;
        auto pcd = ReferenceMesh->SamplePointsPoissonDisk(pointsPercm2Small * 10000 * ReferenceMesh->GetSurfaceArea());
        SmallReferencePCD = getScannablePcd(ReferenceMesh, pcd);
        SmallReferencePCD->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
        open3d::io::WritePointCloudOption params;
        open3d::io::WritePointCloudToPCD(pcdstring_small, *SmallReferencePCD, params);
        cout << "Done." << endl;
    }

    // small pcd should give same result as large
    ReferencePCAVectors = getPCAVectors(SmallReferencePCD);
    SmallReferencePCDCenter = SmallReferencePCD->GetCenter();

    return true;
}

bool ScanningSingleton::SetReferenceModel(std::shared_ptr<open3d::geometry::PointCloud> pcd)
{
    ReferenceMesh = make_shared<open3d::geometry::TriangleMesh>();
    double pointsPercm2 = 9; // todo read from config
    double pointsPercm2Small = 0.9;
    double uniqueModelNumber = 0;
    for (int i = 0; i < pcd->points_.size() && i < 100; i++)
    {
        uniqueModelNumber = uniqueModelNumber + std::pow(pcd->points_[i](0), 2) + std::pow(pcd->points_[i](1), 2) + std::pow(pcd->points_[i](2), 2);
    }
    uniqueModelNumber /= 1000;
    uniqueModelNumber *= 1000; // round for numeric stability

    *ReferencePCD = *pcd;
    if (ReferencePCD->IsEmpty())
    {
        cout << "Error: Reference Pcd is empty!" << endl;
        return false;
    }
    ReferencePCD->Translate(-ReferencePCD->GetCenter());
    // ReferencePCD->Scale(0.001, ReferencePCD->GetCenter());
    if (!ReferencePCD->HasNormals())
    {
        ReferencePCD->EstimateNormals();
    }
    ReferencePCD->NormalizeNormals();

    string pcdstring = ResourceFolder + to_string(uniqueModelNumber) + ".pcd";
    string pcdstring_small = ResourceFolder + to_string(uniqueModelNumber) + "_small" + ".pcd";
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Error); // get rid of read pcd warnings
    SmallReferencePCD = readPcd(pcdstring_small);
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Warning);

    if (SmallReferencePCD->IsEmpty())
    {
        cout << "Sampling points for pcd small..." << endl;
        SmallReferencePCD = ReferencePCD->VoxelDownSample(0.01);
        open3d::io::WritePointCloudOption params;
        open3d::io::WritePointCloudToPCD(pcdstring_small, *SmallReferencePCD, params);
        cout << "Done." << endl;
    }

    // small pcd should give same result as large
    ReferencePCAVectors = getPCAVectors(SmallReferencePCD);
    SmallReferencePCDCenter = SmallReferencePCD->GetCenter();

    return true;
}

Eigen::Matrix4d ScanningSingleton::getIterativePCABasedAlignment(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target)
{
    double fitness = 0;
    return getIterativePCABasedAlignment(source, target, fitness);
}

Eigen::Matrix4d ScanningSingleton::getIterativePCABasedAlignment(std::shared_ptr<open3d::geometry::PointCloud> source_large, std::shared_ptr<open3d::geometry::PointCloud> target_large, double &alignmentFitness)
{

    // create uniformly downsized pcds
    // largest size of the current pcd
    double lm = (source_large->GetMaxBound() - source_large->GetMinBound()).norm();
    double ns = 100; // todo json
    double rv_rough = lm / ns;
    auto source = source_large->VoxelDownSample(rv_rough);
    auto target = target_large->VoxelDownSample(rv_rough);
    // icp inlinier threshold for pca icp
    double tm_rough = 4 * rv_rough;

    // the choice of rv_fine here is not trivial.
    // By using ReferenceVoxelResolution we hit a compromise.
    // Alternatively we could take the Voxelresolution of the worst camera.
    // by taking r_v of the referencepcd, low res cameras contribution in the final
    // fitness gets smaller
    double rv_fine = ReferenceVoxelResolution;
    double tm_fine = 1.5 * rv_fine; // 1.5 here is to account for noise in the data
    int nfine = 2;

    // usually target is refernce model and source is current scan
    Eigen::Matrix3d targetPCAVectors = getPCAVectors(target);

    // find center
    Eigen::Vector3d centerSource = source->GetCenter();
    Eigen::Vector3d centerTarget = target->GetCenter();
    // translate both pcds to center
    auto source_c = std::make_shared<open3d::geometry::PointCloud>(*source);
    auto target_c = std::make_shared<open3d::geometry::PointCloud>(*target);
    source_c->Translate(-centerSource);
    target_c->Translate(-centerTarget);

    // translate both pcds to center
    auto source_large_c = source_large->VoxelDownSample(rv_fine);
    auto target_large_c = target_large->VoxelDownSample(rv_fine);
    source_large_c->Translate(-centerSource);
    target_large_c->Translate(-centerTarget);
    //####################################################

    // 1. Algin source to target using the initial pca rotation
    Eigen::Matrix4d Rpca = getPCABasedRotation(source_c, target_c);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    int max_angle_it = 12;
    double angelInc = 2 * M_PI / (max_angle_it - 1);
    double fitness = 0;
    double thres_fitness = 0.90;
    int icp_max_iterations = 30;

    auto json = readJsonFromDisk("functionConfigs/IterativePCABasedAlignment.json");
    setValueFromJson(json, "max_angle_it", max_angle_it);
    setValueFromJson(json, "thres_fitness", thres_fitness);
    setValueFromJson(json, "thres_icp_inlier", tm_rough);
    setValueFromJson(json, "icp_max_iterations", icp_max_iterations);

    double bestFitness = 0;
    Eigen::Matrix4d bestT = Eigen::Matrix4d::Identity();

    // axis1 is a smaller axis. Just do a flip around that axis.
    for (int axis1 = 0; axis1 < 2; axis1++)
    {
        Eigen::Matrix4d Raxis1 = getRotationMatrixFromVectorAndAngle(targetPCAVectors.block<3, 1>(0, 1), axis1 * M_PI);
        for (int axis2 = 0; axis2 < max_angle_it; axis2++) // biggest axis
        {

            Eigen::Matrix4d Raxis2 = getRotationMatrixFromVectorAndAngle(targetPCAVectors.block<3, 1>(0, 2), axis2 * angelInc);
            T = refineRegistrationICPCenteredPCDs(source_c, target_c, fitness, Raxis2 * Raxis1 * Rpca, icp_max_iterations, tm_rough);
            T = refineRegistrationICPCenteredPCDs(source_large_c, target_large_c, fitness, T, nfine, tm_fine);
            cout << "pca fitness: " << fitness << endl;
            // auto sourceTrans = make_shared<open3d::geometry::PointCloud>(*source_large_c);
            // sourceTrans->Transform(T);
            // sourceTrans->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
            // open3d::visualization::DrawGeometries({getOrigin(), sourceTrans, target_large_c});

            if (fitness > thres_fitness)
            {
                alignmentFitness = fitness;
                return getTrans(centerTarget) * T * getTrans(-centerSource);
            }
            if (fitness > bestFitness)
            {
                alignmentFitness = fitness;
                bestT = T;
                bestFitness = fitness;
            }
        }
    }

    // return best matrix here in case non was really good
    cout << "Did not find a great alignment candidate in pca align, using best candidate\n";
    return getTrans(centerTarget) * bestT * getTrans(-centerSource);
}
