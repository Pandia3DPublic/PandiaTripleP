#include "VolumeSingleton.h"
#include "utils/fileIO.h"
#include "frustum.h"
#include "../utils/3DFunctions.h"
#include "../utils/imageFunctions.h"
#include "../utils/miscFunctions.h"
#include "../utils/conversions.h"
#include "../utils/vishelper.h"
#include "../utils/basicMath.h"

using namespace std;

VolumeSingleton::VolumeSingleton(/* args */)
{
    VolumePcd = make_shared<open3d::geometry::PointCloud>();
    SiloModelPCD = make_shared<open3d::geometry::PointCloud>();
    SiloModel = make_shared<open3d::geometry::TriangleMesh>();
    FirstDirectionPcd = make_shared<open3d::geometry::PointCloud>();
    SecondDirectionPcd = make_shared<open3d::geometry::PointCloud>();
}

VolumeSingleton::~VolumeSingleton()
{
}

double VolumeSingleton::getCurrentVolume()
{
    CurrentVolumeMaxPointHeight = 0;
    CurrentVolumeAvgPointHeight = 0;
    if (!SiloModelSet())
    {
        return -1;
    }
    if (FOVDepthBuffer.size() == 0)
    {
        if (!SiloModel->IsEmpty())
        {
            auto transformedSiloModel = *SiloModel;
            transformedSiloModel.Transform(SiloToFirstCamTransform);
            FOVDepthBuffer = Cameras.front()->getFOVDepthBuffer(transformedSiloModel);
        }
        else
        {
            auto transformedSiloModel = *SiloModelPCD;
            transformedSiloModel.Transform(SiloToFirstCamTransform);
            FOVDepthBuffer = Cameras.front()->getDepthBufferFromPCD(transformedSiloModel);
        }
    }
    auto &cam = *Cameras.front();
    // for both pcds build z-buffers. Keep a pixel only if the recorded value is some minimum value smaller
    // than the reference value.
    Eigen::Matrix3d &intr = cam.DepthLDTIntrinsic.intrinsic_matrix_;
    auto croppedRGBD = cam.getCroppedRGBDImage(cam.rgbdImages.front());
    auto recordedDepth = croppedRGBD->depth_;
    auto rcolor = croppedRGBD->color_;

    // open3d::visualization::DrawGeometries({croppedRGBD});

    // open3d::visualization::DrawGeometries({EigenToO3DDepthImage(FOVDepthBuffer)});
    // auto filteredRecordedDepth = make_shared<open3d::geometry::Image>();
    // filteredRecordedDepth->Prepare(recordedDepth.width_, recordedDepth.height_, 1, 4);
    // auto filteredReferenceDepth = make_shared<open3d::geometry::Image>();
    // filteredReferenceDepth->Prepare(recordedDepth.width_, recordedDepth.height_, 1, 4);

    // Possbile improvements:
    // 1. adjust for wallnoise volume loses, since data gets cut
    // 2. try to filter small extra clusters
    // 3. If there are large parts with no data, we neglect those parts in volume calculation, try 2d convex hull (optional)
    double volume = 0;
    VolumeImage = cv::Mat(recordedDepth.height_, recordedDepth.width_, CV_8UC3, cv::Scalar(0));

    double d_sum = 0;
    unsigned int n_measured = 0;
    for (int i = 0; i < FOVDepthBuffer.rows(); i++)
    {
        for (int j = 0; j < FOVDepthBuffer.cols(); j++)
        {
            double d = *recordedDepth.PointerAt<float>(j, i) - FOVDepthBuffer(i, j);
            double d_abs = std::abs(d);
            if (d<VolumeNoise && * recordedDepth.PointerAt<float>(j, i)> 1e-3 && FOVDepthBuffer(i, j) > 1e-3 && d_abs <= MaxProductHeight)
            {
                if (CreateOverlayinImage)
                {
                    VolumeImage.at<cv::Vec3b>(i, j) = cv::Vec3b(153, 255, 102);
                }
                // the actual images
                //  *filteredRecordedDepth->PointerAt<float>(j, i) = *recordedDepth.PointerAt<float>(j, i);
                //  *filteredReferenceDepth->PointerAt<float>(j, i) = FOVDepthBuffer(i, j);

                // construct frustum per image, near plane is recorded, far plane is reference
                // i is for height, j is for width
                Frustum f(d_abs, *recordedDepth.PointerAt<float>(j, i) / intr(1, 1),
                          *recordedDepth.PointerAt<float>(j, i) / intr(0, 0),
                          FOVDepthBuffer(i, j) / intr(1, 1),
                          FOVDepthBuffer(i, j) / intr(0, 0));
                volume += f.getVolume();
                if (d_abs > CurrentVolumeMaxPointHeight)
                    CurrentVolumeMaxPointHeight = d_abs;
                d_sum += d_abs;
                n_measured++;
            }
        }
    }
    CurrentVolumeAvgPointHeight = d_sum / (n_measured == 0 ? 1 : n_measured);

    float alpha = 0.5;
    float beta = 1 - alpha;
    auto cvColor = Open3DToOpenCV(cam.rgbdImages.front()->color_);
    if (cam.CameraType == CameraTypes::Ensenso)
    {
        cv::resize(VolumeImage, VolumeImage, cam.FRCalibrationImages.front().size());
        cvColor = cam.FRCalibrationImages.front();
    }
    for (int i = 0; i < VolumeImage.rows; i++)
    {
        for (int j = 0; j < VolumeImage.cols; j++)
        {
            if (VolumeImage.at<cv::Vec3b>(i, j)(0) != 0) // volume data is measured here, blend colors
            {
                if (cvColor.channels() == 3)
                {
                    VolumeImage.at<cv::Vec3b>(i, j) = alpha * VolumeImage.at<cv::Vec3b>(i, j) + beta * cvColor.at<cv::Vec3b>(i, j);
                }
                if (cvColor.channels() == 1)
                {
                    float s = cvColor.at<uchar>(i, j);
                    cv::Vec3b frcolor(s, s, s);
                    VolumeImage.at<cv::Vec3b>(i, j) = alpha * VolumeImage.at<cv::Vec3b>(i, j) + beta * frcolor;
                }
            }
            else
            {
                if (cvColor.channels() == 1)
                {
                    float s = cvColor.at<uchar>(i, j);
                    cv::Vec3b frcolor(s, s, s);
                    VolumeImage.at<cv::Vec3b>(i, j) = frcolor;
                }
                if (cvColor.channels() == 3)
                {
                    VolumeImage.at<cv::Vec3b>(i, j) = cvColor.at<cv::Vec3b>(i, j);
                }
            }
        }
    }
    if (CreateOverlayinImage)
    {
        // fov rectangle
        float halfWidth = std::floor(VolumeImage.cols / 2.0);
        float halfHeight = std::floor(VolumeImage.rows / 2.0);
        int left_thres = cam.crop_left / 100 * halfWidth;
        int right_thres = (1 - cam.crop_right / 100) * halfWidth + halfWidth;
        int top_thres = cam.crop_top / 100 * halfHeight;
        int bottom_thres = (1 - cam.crop_bottom / 100) * halfHeight + halfHeight;
        cv::rectangle(VolumeImage, cv::Point(left_thres, top_thres), cv::Point(right_thres, bottom_thres), cv::Scalar(0, 255, 239), 2);

        // belt measurement area, todo needs testing
        // if (!BeltAreaImage.empty())
        // {
        //     cv::resize(BeltAreaImage, BeltAreaImage, VolumeImage.size());
        //     std::vector<std::vector<cv::Point>> contours;
        //     cv::findContours(BeltAreaImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        //     cv::drawContours(VolumeImage, contours, -1, cv::Scalar(255, 255, 0), 2);
        // }
    }

    // auto debugimage = OpenCVToOpen3D(VolumeImage);
    // open3d::visualization::DrawGeometries({debugimage});
    // auto refpcd = open3d::geometry::PointCloud::CreateFromDepthImage(*filteredReferenceDepth, cam.DepthLDTIntrinsic);
    // auto recpcd = open3d::geometry::PointCloud::CreateFromDepthImage(*filteredRecordedDepth, cam.DepthLDTIntrinsic);
    // open3d::visualization::DrawGeometries({refpcd,recpcd,getOrigin()});
    // cout << "d " << DensityFactor << endl;
    // cout << "v0 " << volume << endl;
    // cout << "v1 " << volume * DensityFactor << endl;
    return volume * DensityFactor;
}

// Generally we need to align the silo model to the camerapcd, since the silo model center is
// constant while the silo model center is not! If we want to have consistency when rerunning
// the setup with a silo with a different fill level, we need to save the pcd center at weird
// intervals if we want to do the alignment in this way.
bool VolumeSingleton::ProcessCenterButton()
{
    if (!SiloModelSet())
        return false;
    try
    {
        auto cam = Cameras.front(); // first cam
        x_offset = 0;
        y_offset = 0;
        z_offset = 0;
        auto RecordedAndCroppedPcd = cam->getCroppedPcd(cam->Pcds.front());
        RecordedAndCroppedPcd = RecordedAndCroppedPcd->VoxelDownSample(0.01);
        TranslationOffset = RecordedAndCroppedPcd->GetCenter() - SiloModelPCD->GetCenter();
        setSiloToFirstCamTransform();

        // auto TransSiloModel = make_shared<open3d::geometry::TriangleMesh>(*SiloModel);
        // TransSiloModel->Transform(SiloToFirstCamTransform);
        // open3d::visualization::DrawGeometries({RecordedAndCroppedPcd, getOrigin(), TransSiloModel});
        return true;
    }
    catch (const std::exception &e)
    {
        cout << "C++ Exception caught in ProcessCenterButton." << endl;
        cout << e.what() << endl;
        return false;
    }
}

bool VolumeSingleton::ProcessICPButton()
{
    if (!SiloModelSet())
        return false;
    try
    {
        auto cam = Cameras.front(); // first cam
        auto RecordedAndCroppedPcd = cam->getCroppedPcd(cam->Pcds.front());
        RecordedAndCroppedPcd = RecordedAndCroppedPcd->VoxelDownSample(0.01);
        SiloToFirstCamTransform = refineRegistrationICP_Volume(SiloModelPCD, RecordedAndCroppedPcd, SiloToFirstCamTransform, 220, 0.04);
        
        if (!SiloModel->IsEmpty())
        {
            auto transformedSiloModel = *SiloModel;
            transformedSiloModel.Transform(SiloToFirstCamTransform);
            FOVDepthBuffer = Cameras.front()->getFOVDepthBuffer(transformedSiloModel);
        }
        else
        {
            auto transformedSiloModel = *SiloModelPCD;
            transformedSiloModel.Transform(SiloToFirstCamTransform);
            FOVDepthBuffer = Cameras.front()->getDepthBufferFromPCD(transformedSiloModel);
        }
        
        // auto TransSiloModel = make_shared<open3d::geometry::PointCloud>(*SiloModelPCD);
        // TransSiloModel->Transform(SiloToFirstCamTransform);
        // open3d::visualization::DrawGeometries({RecordedAndCroppedPcd, getOrigin(), TransSiloModel});
        /* code */
        return true;
    }
    catch (const std::exception &e)
    {
        cout << "C++ Exception caught in ProcessICPButton." << endl;
        cout << e.what() << endl;
        return false;
    }
}

bool VolumeSingleton::setSiloToFirstCamTransform()
{
    if (!SiloModelSet())
        return false;

    Eigen::Vector3d siloCenter = Eigen::Vector3d(0,0,0);
    // silomodel is already centered in setsilomodel, so this is just zeros
    if (!SiloModel->IsEmpty())
        siloCenter = SiloModel->GetCenter();
    else
        siloCenter = SiloModelPCD->GetCenter();

    Eigen::Matrix4d Rx = getTrans(siloCenter) * getRx(alpha) * getTrans(-siloCenter);
    Eigen::Matrix4d Ry = getTrans(siloCenter) * getRy(beta) * getTrans(-siloCenter);
    Eigen::Matrix4d Rz = getTrans(siloCenter) * getRz(gamma) * getTrans(-siloCenter);
    SiloToFirstCamTransform = getTrans(Eigen::Vector3d(x_offset + TranslationOffset(0),
                                                       y_offset + TranslationOffset(1), z_offset + TranslationOffset(2))) *
                              Rz * Ry * Rx;

    if (!SiloModel->IsEmpty())
    {
        auto transformedSiloModel = *SiloModel;
        transformedSiloModel.Transform(SiloToFirstCamTransform);
        FOVDepthBuffer = Cameras.front()->getFOVDepthBuffer(transformedSiloModel);
    }
    else
    {
        auto transformedSiloModel = *SiloModelPCD;
        transformedSiloModel.Transform(SiloToFirstCamTransform);
        FOVDepthBuffer = Cameras.front()->getDepthBufferFromPCD(transformedSiloModel);
    }
    // SaveAlignmentData(ConfigFolder);
    // saveMatrixToDisc(ConfigFolder, "SiloToFirstCamTransform.txt",SiloToFirstCamTransform);
    return true;
}

void VolumeSingleton::setSiloModel(std::shared_ptr<open3d::geometry::TriangleMesh> mesh)
{
    clearSiloModel();
    double pointsPercm2 = 0.9;
    double uniqueModelNumber = 0;
    *SiloModel = *mesh;
    for (int i = 0; i < SiloModel->vertices_.size() && i < 100; i++)
    {
        uniqueModelNumber = uniqueModelNumber + std::pow(SiloModel->vertices_[i](0), 2) + std::pow(SiloModel->vertices_[i](1), 2) + std::pow(SiloModel->vertices_[i](2), 2);
    }
    uniqueModelNumber /= 1000;
    uniqueModelNumber *= 1000; // round for numeric stability

    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Error); // get rid of read pcd warnings

    SiloModel->Translate(-SiloModel->GetCenter());
    string pcdstring = ResourceFolder + to_string(uniqueModelNumber) + ".pcd";
    SiloModelPCD = readPcd(pcdstring);

    if (SiloModelPCD->IsEmpty())
    {
        cout << "Sampling points for silo pcd..." << endl;
        SiloModelPCD = SiloModel->SamplePointsPoissonDisk(pointsPercm2 * 10000 * SiloModel->GetSurfaceArea());
        open3d::io::WritePointCloudOption params;
        open3d::io::WritePointCloudToPCD(pcdstring, *SiloModelPCD, params);
        cout << "Done." << endl;
    }

    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Warning);
}

void VolumeSingleton::setSiloModel(std::shared_ptr<open3d::geometry::PointCloud> pcd)
{
    clearSiloModel();
    *SiloModelPCD = *pcd;
    SiloModelPCD->Translate(-SiloModelPCD->GetCenter());
}

bool VolumeSingleton::SiloModelSet()
{
    if (SiloModelPCD->IsEmpty())
        return false;
    else
        return true;
}

void VolumeSingleton::clearSiloModel()
{
    SiloModel->Clear();
    SiloModelPCD->Clear();
    FOVDepthBuffer = Eigen::MatrixXd();
}

void VolumeSingleton::CalculateDirectionVector()
{
    if (FOVDepthBuffer.size() == 0)
    {
        auto transformedSiloModel = *SiloModel;
        transformedSiloModel.Transform(SiloToFirstCamTransform);
        FOVDepthBuffer = Cameras.front()->getFOVDepthBuffer(transformedSiloModel);
    }

    // get pcd in front of background of both direction pcds.
    auto firstFilteredPcd = make_shared<open3d::geometry::PointCloud>();
    auto secondFilteredPcd = make_shared<open3d::geometry::PointCloud>();
    auto &cam = Cameras.front();
    double averagedepth;
    for (int k = 0; k < FirstDirectionPcd->points_.size(); k++)
    {
        Eigen::Vector3d &p = FirstDirectionPcd->points_[k];
        averagedepth += p(2);
        Eigen::Vector2i ind = cam->PointtoPixel(p);
        if (cam->InsideDepthImage(ind))
        {
            int &i = ind(0);
            int &j = ind(1);

            double d = p(2) - FOVDepthBuffer(i, j);
            if (d < VolumeNoise && p(2) > 1e-3 && FOVDepthBuffer(i, j) > 1e-3)
            {
                firstFilteredPcd->points_.push_back(p);
            }
        }
    }
    averagedepth /= FirstDirectionPcd->points_.size();

    for (int k = 0; k < SecondDirectionPcd->points_.size(); k++)
    {
        Eigen::Vector3d &p = SecondDirectionPcd->points_[k];
        Eigen::Vector2i ind = cam->PointtoPixel(p);
        if (cam->InsideDepthImage(ind))
        {

            int &i = ind(0);
            int &j = ind(1);
            double d = p(2) - FOVDepthBuffer(i, j);
            if (d < VolumeNoise && p(2) > 1e-3 && FOVDepthBuffer(i, j) > 1e-3)
            {
                secondFilteredPcd->points_.push_back(p);
            }
        }
    }

    Eigen::Vector3d p1 = cam->PixeltoPoint(0, 0, averagedepth);
    Eigen::Vector3d p2 = cam->PixeltoPoint(cam->DepthLDTIntrinsic.height_, 0, averagedepth);
    // auto labels = firstFilteredPcd->ClusterDBSCAN(((p2-p1).norm()/cam->DepthLDTIntrinsic.height_)/4,20);
    auto labels = firstFilteredPcd->ClusterDBSCAN(0.02, 20);
    auto tmp = make_shared<open3d::geometry::PointCloud>();
    // int firstcount = 0;
    // int secondcount = 0;
    for (int k = 0; k < firstFilteredPcd->points_.size(); k++)
    {
        if (labels[k] != -1)
        {
            tmp->points_.push_back(firstFilteredPcd->points_[k]);
        } // else{
        //     firstcount++;
        // }
    }
    *firstFilteredPcd = *tmp;
    tmp->Clear();
    labels = secondFilteredPcd->ClusterDBSCAN(0.02, 20);
    for (int k = 0; k < secondFilteredPcd->points_.size(); k++)
    {
        if (labels[k] != -1)
        {
            tmp->points_.push_back(secondFilteredPcd->points_[k]);
        } // else{
        //     secondcount++;
        // }
    }
    *secondFilteredPcd = *tmp;
    // cout << "firstcount " << firstcount << endl;
    // cout << "secondcount " << secondcount << endl;
    // open3d::visualization::DrawGeometries({getOrigin(), firstFilteredPcd,secondFilteredPcd});
    // open3d::visualization::DrawGeometries({getOrigin(), secondFilteredPcd});
    DirectionVector = secondFilteredPcd->GetCenter() - firstFilteredPcd->GetCenter();
    DirectionVector.normalize();

    //###### setting the base plane #######
    Eigen::Vector3d vn = DirectionVector.normalized();
    double l = 0;
    // this follows by setting the derivative of the square of plane point distances to zero
    for (auto &p : SecondDirectionPcd->points_)
    {
        l += p.dot(vn);
    }
    l /= SecondDirectionPcd->points_.size();
    BasePlanePoint = vn * l;
    BasePlane = PandiaPlane(vn, BasePlanePoint);
}

void VolumeSingleton::setBasePlane()
{
    BasePlane = PandiaPlane(DirectionVector.normalized(), BasePlanePoint);
}

double VolumeSingleton::getVolumeIncrement(double dt)
{
    auto &cam = *Cameras.front();
    // for both pcds build z-buffers. Keep a pixel only if the recorded value is some minimum value smaller
    // than the reference value.
    auto croppedRGBD = cam.getCroppedRGBDImage(cam.rgbdImages.front());
    Eigen::Vector3d v = DirectionVector * BeltSpeed;
    Eigen::Vector3d vn = v.normalized();
    // second plane of p1 and vn
    Eigen::Vector3d p1 = BasePlane.p0 + v * dt;
    PandiaPlane plane1(vn, p1);
    // only take the points between the two planes and measure the volume of those

    auto depth = croppedRGBD->depth_;
    for (int i = 0; i < depth.height_; i++)
    {
        for (int j = 0; j < depth.width_; j++)
        {
            Eigen::Vector3d p = cam.PixeltoPoint(i, j, *depth.PointerAt<float>(j, i));
            if (!(BasePlane.getMinDistance(p) > 0 && plane1.getMinDistance(p) < 0))
            {
                *depth.PointerAt<float>(j, i) = 0;
            }
        }
    }
    BeltAreaImage = Open3DToOpenCV(depth);
    BeltAreaImage.convertTo(BeltAreaImage, CV_8UC1);

    //########### now we can calculate the actual volume!!!! #####################
    Eigen::Matrix3d &intr = cam.DepthLDTIntrinsic.intrinsic_matrix_;
    double volume = 0;
    for (int i = 0; i < FOVDepthBuffer.rows(); i++)
    {
        for (int j = 0; j < FOVDepthBuffer.cols(); j++)
        {
            double d = *depth.PointerAt<float>(j, i) - FOVDepthBuffer(i, j);
            if (d<VolumeNoise && *depth.PointerAt<float>(j, i)> 1e-3 && FOVDepthBuffer(i, j) > 1e-3 && abs(d) <= MaxProductHeight)
            {
                // construct frustum per image, near plane is recorded, far plane is reference
                // i is for height, j is for width
                Frustum f(std::abs(d), *depth.PointerAt<float>(j, i) / intr(1, 1),
                          *depth.PointerAt<float>(j, i) / intr(0, 0),
                          FOVDepthBuffer(i, j) / intr(1, 1),
                          FOVDepthBuffer(i, j) / intr(0, 0));
                volume += f.getVolume();
            }
        }
    }

    // cutPcd->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    // cout << "v " << v << endl;
    // cout << "dt " << dt << endl;
    // cout << "DirectionVector " << DirectionVector.norm() << endl;
    // cout << "BeltSpeed " << BeltSpeed << endl;
    // auto pl1 = plane1.getPlaneTriangleMesh();
    // auto base = BasePlane.getPlaneTriangleMesh();
    // base->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
    // pl1->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
    // open3d::visualization::DrawGeometries({make_shared<open3d::geometry::Image>(depth)});
    // open3d::visualization::DrawGeometries({getOrigin(), cam.pcdFromDepth(*croppedRGBD), cutPcd, base, pl1},
    //                                       "Test", 1000, 100, 50, 50, false, true, true);

    return volume * DensityFactor;
}

void VolumeSingleton::clearBeltAreaImage()
{
    BeltAreaImage = cv::Mat();
}