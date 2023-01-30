#pragma once

std::string getStringforNumberedFile(int a);
void setFromIntrinsicFile(const std::string &filepath, open3d::camera::PinholeCameraIntrinsic &intrinsic);
void IntrinsicFileHelperFunction(std::string value, int lineCounter, open3d::camera::PinholeCameraIntrinsic &intrinsic);
bool fileExists(const std::string &filename);
bool saveMatrixToDisc(std::string path, std::string name, const Eigen::Matrix4d &matrix);
bool saveVectorToDisc(std::string path, std::string name, const std::vector<float> vector);
bool saveStringToDisc(std::string path, std::string name, const std::string& content);
Eigen::Matrix4d readMatrixFromDisc(const std::string &PathAndName);
std::string readStringFromDisc(const std::string& path,const std::string& name);
std::string getExtensionFromFilename(const std::string &filename);
std::shared_ptr<open3d::geometry::TriangleMesh> readMesh(const std::string &path);
std::shared_ptr<open3d::geometry::PointCloud> readPcd(const std::string &path);
std::vector<std::string> getFileNames(const std::string& folder);
