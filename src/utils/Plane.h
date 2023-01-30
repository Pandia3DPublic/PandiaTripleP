#pragma once

class PandiaRay;

class PandiaPlane
{
public:
    PandiaPlane(const Eigen::Vector3d &n_, const Eigen::Vector3d &point);
    Eigen::Vector3d n;
    double d;
    double l;
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    PandiaRay PlanePlaneIntersection(PandiaPlane &other);
    double getMinDistance(const Eigen::Vector3d &p);

    std::shared_ptr<open3d::geometry::TriangleMesh> getPlaneTriangleMesh();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class PandiaRay
{
public:
    PandiaRay();
    PandiaRay(const Eigen::Vector3d &t, const Eigen::Vector3d &point);
    Eigen::Vector3d Direction;
    Eigen::Vector3d Origin;
    Eigen::Vector3d RayPlaneIntersection(PandiaPlane &other);
    bool RayPlaneIntersects(PandiaPlane &other);
    bool RayRayIntersects(PandiaRay &other, Eigen::Vector3d planeN);
    std::shared_ptr<open3d::geometry::LineSet> getLineSet();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};