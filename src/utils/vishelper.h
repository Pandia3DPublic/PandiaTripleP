#pragma once

std::shared_ptr<open3d::geometry::LineSet> getOrigin();
std::shared_ptr<open3d::geometry::LineSet> getLineSet(const Eigen::Matrix3d &axis);
std::shared_ptr<open3d::geometry::LineSet> getLineSet(const Eigen::Vector3d &vec);
std::shared_ptr<open3d::geometry::TriangleMesh> getCameraArrow(const Eigen::Matrix4d &trans);
void showEdges(cv::Mat img);
std::shared_ptr<open3d::geometry::LineSet> visualizeHalfsphereVectors(const std::vector<Eigen::Vector3d> &vecs, const Eigen::Vector3d &p = Eigen::Vector3d::Zero());
