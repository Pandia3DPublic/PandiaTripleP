#pragma once
#include "cameras/CameraWrapper.h"
bool SortbyAlphabet(const std::string &s1, const std::string &s2);
double edgeFunction(const Eigen::Vector2d &v0, const Eigen::Vector2d &v1, const Eigen::Vector2d &p);
std::vector<int> getInnerToOuterEdgeMapping(std::vector<Eigen::Vector2i> &innerEdge, std::vector<Eigen::Vector2i> &outerEdge);
bool IsVisible(const Eigen::Vector3d& p, const Eigen::MatrixXd& depth_buffer,CameraWrapper& cam);
