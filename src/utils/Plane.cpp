#include "Plane.h"


PandiaPlane::PandiaPlane(const Eigen::Vector3d &n_, const Eigen::Vector3d &point)
{
    n = n_.normalized();
    p0 = point;
    d = point.transpose() * n;
}

double PandiaPlane::getMinDistance(const Eigen::Vector3d &p)
{
    return n.dot(p - p0);
}
PandiaRay PandiaPlane::PlanePlaneIntersection(PandiaPlane &other)
{
    PandiaRay out;
    out.Direction = n.cross(other.n);
    const float l = out.Direction.squaredNorm();
    float eps = 1e-6;

    // If the length of the cross product is 0, that means parallel planes, no intersection.
    if (l > eps)
    {
        // calculate the final (point, normal)
        out.Origin = -((out.Direction.cross(other.n) * d) + (n.cross(out.Direction) * other.d)) / l;
        return out;
    }
    return PandiaRay();
}
std::shared_ptr<open3d::geometry::TriangleMesh> PandiaPlane::getPlaneTriangleMesh()
{
    //vector inside the plane 
    PandiaRay normalray;
    normalray.Direction = n;
    normalray.Origin = Eigen::Vector3d::Ones();
    Eigen::Vector3d pointOnPlane = normalray.RayPlaneIntersection(*this);
    Eigen::Vector3d vectorOnPlane = pointOnPlane - p0;
    // if (n.z() != 0)
    // {
    //     double vz = (-n.x() - n.y()) / n.z();
    //     v = Eigen::Vector3d(1, 1, vz);
    // }
    // else if (n.y() != 0)
    // {
    //     double vy = (-n.x() - n.z()) / n.y();
    //     v = Eigen::Vector3d(1, vy, 1);
    // }
    // else if (n.x() != 0)
    // {
    //     double vx = (-n.y() - n.z()) / n.x();
    //     v = Eigen::Vector3d(vx, 1, 1);
    // }
    // v.normalize();
    // v = v *2;
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    // mesh->vertices_.push_back(n * d);
    // mesh->vertices_.push_back(v + n * d);
    // Eigen::Vector3d u = v.cross(n);
    // mesh->vertices_.push_back(u + n * d);
    // mesh->vertices_.push_back(n * d + u + v);


    Eigen::Vector3d secondVectorOnPlane = vectorOnPlane.cross(n);
    mesh->vertices_.push_back(p0 );
    mesh->vertices_.push_back(p0 + vectorOnPlane);
    mesh->vertices_.push_back(p0 + secondVectorOnPlane);
    mesh->vertices_.push_back(p0 + secondVectorOnPlane + vectorOnPlane);
    mesh->triangles_.push_back(Eigen::Vector3i(2, 1, 0));
    mesh->triangles_.push_back(Eigen::Vector3i(1, 2, 3));
    mesh->ComputeTriangleNormals();
    return mesh;
}

PandiaRay::PandiaRay()
{
    Origin = Eigen::Vector3d::Zero();
    Direction = Eigen::Vector3d::Zero();
}
PandiaRay::PandiaRay(const Eigen::Vector3d &t, const Eigen::Vector3d &point)
{
    Direction = t.normalized();
    Origin = point;
}

Eigen::Vector3d PandiaRay::RayPlaneIntersection(PandiaPlane &other)
{
    float denom = other.n.dot(Direction);
    float eps = 1e-6;
    if (denom > eps)
    {
        float t = (other.d * other.n - Origin).dot(other.n) / denom;
        return Origin + t * Direction;
    }
    std::cout << "RayPlaneIntersection failed" << std::endl;
    return Eigen::Vector3d::Zero();
}

bool PandiaRay::RayPlaneIntersects(PandiaPlane &other)
{
    float denom = other.n.dot(Direction);
    float eps = 1e-6;
    if (denom > eps)
    {
        float t = (other.d * other.n - Origin).dot(other.n) / denom;
        return true;
    }
    return false;
}
std::shared_ptr<open3d::geometry::LineSet> PandiaRay::getLineSet()
{
    auto out = std::make_shared<open3d::geometry::LineSet>();
    out->points_.push_back(Origin + 5 * Direction);
    out->points_.push_back(Origin);
    out->lines_.push_back(Eigen::Vector2i(0, 1));
    out->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    return out;
}

bool PandiaRay::RayRayIntersects(PandiaRay &other, Eigen::Vector3d planeN)
{
    if (other.Direction.cross(Direction) == Eigen::Vector3d::Zero())
    {
        return false;
    }
    else if (Direction.cross(other.Direction).dot(planeN) >= 1 - 1e-6 && Direction.cross(other.Direction).dot(planeN) <= 1 + 1e-6)
    {
        return true;
    }
    else
    {
        return false;
    }
}