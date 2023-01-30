#pragma once

//images
cv::Mat Open3DToOpenCV(const open3d::geometry::Image &img);
cv::Matx33d eigenToOpenCv33(const Eigen::Matrix3d &m);
cv::Matx33f eigenToOpenCv33f(const Eigen::Matrix3d &m);
Eigen::Matrix3d opencv33ToEigen(const cv::Matx33d m);
std::shared_ptr<open3d::geometry::Image> OpenCVToOpen3D(const cv::Mat &img);
cv::Mat KinectImageToOpenCV(const k4a::image &im);
std::shared_ptr<open3d::geometry::Image> KinectImageToOpen3D(const k4a::image &im);
k4a::image Open3DToKinectImage(const open3d::geometry::Image &img);
cv::Mat FrametoMat(const rs2::frame &f);
cv::Mat FrametoMatMeters(const rs2::depth_frame &f);
cv::Mat FrametoMatMillimeters(const rs2::depth_frame &f);
std::shared_ptr<open3d::geometry::Image> EigenToO3DDepthImage(const Eigen::MatrixXd &depth);
Eigen::MatrixXd O3DDepthtoEigen(std::shared_ptr<open3d::geometry::Image> depth);

//pcds
std::shared_ptr<open3d::geometry::PointCloud> KinectPCDToOpen3D(const k4a::image &k4apcd, const k4a::image *color = nullptr);

//misc
double toDegrees(const double &radians);
double toRadians(const double &degrees);
Eigen::Matrix4d arraysToEigenRow(float *rot, float *trans);
Eigen::Matrix4d arraysToEigenColumn(float *rot, float *trans);
