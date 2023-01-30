#pragma once



std::vector<double> getDistanceFromMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, std::shared_ptr<open3d::geometry::PointCloud> pcd,
                                        std::vector<Eigen::Vector3d> &nearestPoints);
void applyNormalFilter(std::shared_ptr<open3d::geometry::PointCloud> &pcd_in);
std::vector<double> getPointCloudPointToPlaneDistances(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2);
std::vector<double> getPointCloudPointToPlaneDistancesBidirectional(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2);
std::vector<Eigen::Vector3d> getHalfsphereVectors();
std::shared_ptr<open3d::geometry::PointCloud> getScannablePcd(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, std::shared_ptr<open3d::geometry::PointCloud> pcd);
Eigen::Matrix3d kabsch(const Eigen::Matrix3Xd &in, const Eigen::Matrix3Xd &out);
bool rayTriangleIntersect(const Eigen::Vector3d &orig, const Eigen::Vector3d &dir,
    const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2,
    float &t, float &u, float &v);

Eigen::Matrix4d refineRegistrationICP(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d &init, int maxIterations = 30, double maxCorsDistance = 0.02);
Eigen::Matrix4d refineRegistrationICP(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d &init, double &fitness, int maxIterations = 30, double maxCorsDistance = 0.02);
Eigen::Matrix4d refineRegistrationICP_Volume(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d &init, int maxIterations = 30, double maxCorsDistance = 0.02);
std::vector<Eigen::Vector3d> getDistanceColorsGradientGeneral(const std::vector<double>& distances, double min, double max);