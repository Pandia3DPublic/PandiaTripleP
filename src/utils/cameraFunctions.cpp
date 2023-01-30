#include "cameraFunctions.h"
using namespace std;
open3d::camera::PinholeCameraIntrinsic getScaledIntrinsic(const open3d::camera::PinholeCameraIntrinsic &intr, int newWidth, int newHeight)
{
    double factorWidth = (double)newWidth / (double)intr.width_;
    double factorHeight = (double)newHeight / (double)intr.height_;
    if (factorWidth != factorHeight)
    {
        cout << "Warning: scaled intrinsic has different aspect ratio" << endl;
    }
    double fx = intr.intrinsic_matrix_(0, 0) * factorWidth;
    double fy = intr.intrinsic_matrix_(1, 1) * factorHeight;
    double cx = intr.intrinsic_matrix_(0, 2) * factorWidth;
    double cy = intr.intrinsic_matrix_(1, 2) * factorHeight;
    return open3d::camera::PinholeCameraIntrinsic(newWidth, newHeight, fx, fy, cx, cy);
}

CameraWrapper *getCameraBySerialNumber(std::shared_ptr<std::vector<CameraWrapper>> cameras, std::string SerialNumber)
{
    for (auto &cam : *cameras)
    {
        if (cam.SerialNumber == SerialNumber)
        {
            return &cam;
        }
    }
    return nullptr;
}

bool SortbySerialNumber(const CameraWrapper &c1, const CameraWrapper &c2)
{
    return (c1.SerialNumber < c2.SerialNumber);
}



//gets the nearby points in 2d
vector<Eigen::Vector4d> getNearbyPoints(const Eigen::Vector2d &p_q, const vector<Eigen::Vector4d> &points, const open3d::geometry::KDTreeFlann& kdtree)
{

    vector<Eigen::Vector4d> out;


    Eigen::Vector3d p_q3d(p_q(0),p_q(1),0);
    std::vector<int> indices(1);
    std::vector<double> dists(1);
    double search_radius = 2;
    kdtree.SearchRadius(p_q3d, search_radius, indices, dists);

    for (auto& ind:indices){
        out.push_back(points[ind]);
    }
    // cout << out.size() << endl;
    return out;
}


cv::Mat getUndistortionMap(const cv::Mat DistortionMap){


    cv::Mat out(cv::Size(DistortionMap.cols, DistortionMap.rows), DistortionMap.type());
    //1. get 2d pointmap
    vector<Eigen::Vector4d> pointmap;

    for (int i = 0; i < DistortionMap.rows; i++)
    {
        for (int j = 0; j < DistortionMap.cols; j++)
        {
            pointmap.push_back(Eigen::Vector4d(DistortionMap.at<cv::Vec2f>(i, j)(0), DistortionMap.at<cv::Vec2f>(i, j)(1),
            DistortionMap.at<cv::Vec2f>(i, j)(0)-j, DistortionMap.at<cv::Vec2f>(i, j)(1)-i));
        }
    }

    // construct open3d pcd for easy kdtree search
    auto pcd = make_shared<open3d::geometry::PointCloud>();
    for (auto &p : pointmap)
    {
        pcd->points_.push_back(Eigen::Vector3d(p(0), p(1), 0));
    }

    open3d::geometry::KDTreeFlann pointmapKdtree;
    pointmapKdtree.SetGeometry(*pcd);

int c = 0;
    //interpolate nearby points
    for (int i = 0; i < out.rows; i++)
    {
        for (int j = 0; j < out.cols; j++)
        {
            Eigen::Vector2d ind(j,i);
            auto points = getNearbyPoints(ind, pointmap, pointmapKdtree);
            double weight = 0;
            // if (!points.empty()){
            //     ind = ind - points.front().tail(2);
            // }
            Eigen::Vector2d dir = Eigen::Vector2d::Zero();
            for (auto& p:points){
                // cout << p.tail(2) << endl;
                double w = 1/(ind-p.head(2)).norm() ;
                dir = dir - w* p.tail(2);
                weight += w;
            }

            if (dir.norm() != 0)
            {
                dir /= weight;
                // dir /= points.size();
                ind += dir;
                out.at<cv::Vec2f>(i, j)(0) = ind(0);
                out.at<cv::Vec2f>(i, j)(1) = ind(1);
                // cout << out_eigen << endl;
            }
            else
            {
                out.at<cv::Vec2f>(i, j) = cv::Vec2f(0,0);
                c++;
            }
        }
    }
    if (c> 0){
        cout << "Warning: zero pixels in distortion map: " <<  c << endl;
    }


    return out;

}