#pragma once
#include "../cameras/CameraWrapper.h"

cv::Matx33f CalibrationIntrToOpenCVMat(CameraWrapper& cam);
Eigen::Matrix4d OpencvTransformationToEigen(cv::Matx33d R, cv::Vec3d t);
Eigen::Matrix4d refineRegistrationICPCenteredPCDs(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target, double &goodness, const Eigen::Matrix4d &init = Eigen::Matrix4d::Identity(), int maxIterations = 30, double maxCorsDistance = 0.02);

std::vector<Eigen::Vector2i> getEdgePixels(std::shared_ptr<open3d::geometry::RGBDImage> img);
std::vector<Eigen::Vector2i> getBackgroundEdgePixels(std::shared_ptr<open3d::geometry::RGBDImage> img, std::vector<Eigen::Vector2i> &edges);
void applyEdgeColorPersistenceFilter(std::shared_ptr<open3d::geometry::RGBDImage> rgbd);

Eigen::Matrix3d getPCAVectors(std::shared_ptr<open3d::geometry::PointCloud> pcd);
Eigen::Matrix4d getPCABasedRotation(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target);

Eigen::Matrix4d getIterativeAlignmentforOCMI(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target, const Eigen::Vector3d& rotationAxis, double &lastAngle);
