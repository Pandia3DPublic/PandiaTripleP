#include "3DFunctions.h"
#include "conversions.h"
#include "basicMath.h"
#include "vishelper.h"
using namespace std;

// headerless!
//  s = start
//  e = end
bool getDistanceToLineSegment(const Eigen::Vector3d &p, const Eigen::Vector3d &s, const Eigen::Vector3d &e, double &dist, Eigen::Vector3d &x)
{
    Eigen::Vector3d f = e - s;
    double fl = f.norm();
    double d = p.dot(f);
    double t = (d - s.dot(f)) / (fl * fl);
    if (t < 0 || t > 1)
        return false;

    // Eigen::Vector3d x = s + t * f;
    x = s + t * f;
    dist = (p - x).norm();
    return true;
}

// todo performance!
vector<double> getDistanceFromMesh(shared_ptr<open3d::geometry::TriangleMesh> mesh, shared_ptr<open3d::geometry::PointCloud> pcd,
                                   std::vector<Eigen::Vector3d> &nearestPoints)
{
    vector<double> out;
    out.resize(pcd->points_.size());
    nearestPoints.clear();
    nearestPoints.resize(pcd->points_.size());
    int maxcount = 0;
// do brute-force triangle ray intersection for all triangles.
#pragma omp parallel for
    for (int i = 0; i < pcd->points_.size(); i++)
    {
        double dmin = 1e6;
        Eigen::Vector3d np = pcd->points_[i]; // nearest point on mesh for debug
        // check for closest distance to all triangles
        for (int j = 0; j < mesh->triangles_.size(); j++)
        {

            Eigen::Vector3d po = pcd->points_[i];
            Eigen::Vector3d n = mesh->triangle_normals_[j];
            Eigen::Vector3d v0 = mesh->vertices_[mesh->triangles_[j](0)];
            Eigen::Vector3d v1 = mesh->vertices_[mesh->triangles_[j](1)];
            Eigen::Vector3d v2 = mesh->vertices_[mesh->triangles_[j](2)];
            float d;
            float u;
            float v;
            bool success = rayTriangleIntersect(po, n, v0, v1, v2, d, u, v);
            if (!success)
            {
                success = rayTriangleIntersect(po, -n, v0, v1, v2, d, u, v);
            }
            if (success && abs(d) < dmin)
            {
                dmin = abs(d);
                np = po + d * n;
            }
        }
        // check for closest distance to all line segments
        // no triangle has been found that is nearest
        for (int j = 0; j < mesh->triangles_.size(); j++)
        {
            for (int k = 0; k < 3; k++)
            {
                double d;
                int index = 0;
                if (k == 0)
                    index = 1;
                if (k == 1)
                    index = 2;
                Eigen::Vector3d s = mesh->vertices_[mesh->triangles_[j](index)];
                Eigen::Vector3d e = mesh->vertices_[mesh->triangles_[j](k)];
                Eigen::Vector3d px;
                bool success = getDistanceToLineSegment(pcd->points_[i], s, e, d, px);
                if (success && abs(d) < dmin)
                {
                    dmin = abs(d);
                    np = px;
                }
            }
        }
        if (dmin < 1e6)
        {
            out[i] = dmin;
            nearestPoints[i] = np;
        }
        else
        {
            // todo need closest point
            maxcount++;
        }
    }
    cout << maxcount << " points did not find the nearest triangle or line segment\n";
    return out;
}

vector<double> getPointCloudPointToPlaneDistances(shared_ptr<open3d::geometry::PointCloud> pcd1, shared_ptr<open3d::geometry::PointCloud> pcd2)
{
    using namespace open3d;
    std::vector<double> distances(pcd1->points_.size());
    geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*pcd2);
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)pcd1->points_.size(); i++)
    {
        std::vector<int> indices(1);
        std::vector<double> dists(1);
        if (kdtree.SearchKNN(pcd1->points_[i], 1, indices, dists) == 0)
        {
            utility::LogDebug(
                "[ComputePointCloudToPointCloudDistance] Found a point "
                "without neighbors.");
            distances[i] = 0.0;
        }
        else
        {
            Eigen::Vector3d pt = pcd2->points_[indices.front()];
            Eigen::Vector3d nt = pcd2->normals_[indices.front()];
            double rho = pcd1->points_[i].dot(nt);
            distances[i] = std::abs(pt.dot(nt) - rho);
            // distances[i] = std::sqrt(dists[0]);
        }
    }
    return distances;
}

vector<double> getPointCloudPointToPlaneDistancesBidirectional(shared_ptr<open3d::geometry::PointCloud> pcd1, shared_ptr<open3d::geometry::PointCloud> pcd2)
{
    using namespace open3d;
    cout << "Warning: Using untested point to plane distances" << endl;
    // cout << "pcd1 size " << pcd1->points_.size() << endl;
    std::vector<double> distances(pcd1->points_.size(), 1e6);
    std::vector<double> distances1(pcd1->points_.size(), 1e6);
    std::vector<double> distances2(pcd1->points_.size(), 1e6);
    geometry::KDTreeFlann kdtree2;
    kdtree2.SetGeometry(*pcd2);
    int count1 = 0;
    int count2 = 0;
    // open3d::visualization::DrawGeometries({pcd1, getOrigin()});

#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)pcd1->points_.size(); i++)
    {
        std::vector<int> indices2(1);
        std::vector<double> dists2(1);
        if (kdtree2.SearchKNN(pcd1->points_[i], 1, indices2, dists2) == 0)
        {
            utility::LogDebug("[ComputePointCloudToPointCloudDistance] Found a point without neighbors.");
        }
        else
        {
            Eigen::Vector3d pt2 = pcd2->points_[indices2.front()];
            Eigen::Vector3d nt2 = pcd2->normals_[indices2.front()];
            double rho1 = pcd1->points_[i].dot(nt2);
            distances1[i] = std::abs(pt2.dot(nt2) - rho1);

            Eigen::Vector3d pt1 = pcd1->points_[i];
            Eigen::Vector3d nt1 = pcd1->normals_[i];
            double rho2 = pcd2->points_[indices2.front()].dot(nt1);
            distances2[i] = std::abs(pt1.dot(nt1) - rho2);

            if (distances2[i] > 1.5 * distances1[i] || distances1[i] > 1.5 * distances2[i])
            {
                // // point to point dist
                Eigen::Vector3d diff = pcd1->points_[i] - pcd2->points_[indices2.front()];
                distances[i] = diff.norm();
                // count1++;
            }
            else
            {
                distances[i] = std::min(distances1[i], distances2[i]);
                // count2++;
            }
        }
    }
    // cout << "case 1 " << count1 << endl;
    // cout << "case 2 " << count2 << endl;
    return distances;
}

// assumes an untransformed pcd
void applyNormalFilter(shared_ptr<open3d::geometry::PointCloud> &pcd_in)
{
    if (!pcd_in->IsEmpty())
    {
        auto filteredPcd = make_shared<open3d::geometry::PointCloud>();
        double thres = std::cos(toRadians(70));
        for (int j = 0; j < pcd_in->points_.size(); j++)
        {
            double tmp = std::acos(pcd_in->normals_[j].dot(Eigen::Vector3d(0, 0, 1)));
            if (tmp > thres && tmp < M_PI_2 - thres)
            { // 70deg
                filteredPcd->points_.push_back(pcd_in->points_[j]);
                filteredPcd->normals_.push_back(pcd_in->normals_[j]);
                if (pcd_in->HasColors())
                {
                    filteredPcd->colors_.push_back(pcd_in->colors_[j]);
                }
            }
        }
        pcd_in = filteredPcd;
    }
    else
    {
        std::cout << "Warning: PCD is empty in normal filter!" << endl;
    }
}

// looking into the direction of the camera, x is left, y is up
vector<Eigen::Vector3d> getHalfsphereVectors()
{
    vector<Eigen::Vector3d> out;
    double minAngle = toRadians(15);
    // double maxAngle = 2* PANDIA_PI;
    double maxAngle = M_PI - minAngle;
    double stepsxy = 5;                   // actual step size is one more
    double alphaInc = maxAngle / stepsxy; // alpha for values in x-y plane
    double stepsyz = 5;                   // actual step size is one more
    double betaInc = maxAngle / stepsyz;  // beta for yz plane

    // length of all vectors is one
    for (float alpha = minAngle; alpha <= maxAngle; alpha += alphaInc)
    {
        for (float beta = minAngle; beta <= maxAngle; beta += betaInc)
        {
            out.push_back(Eigen::Vector3d(std::sin(beta) * std::cos(alpha), std::sin(alpha) * std::sin(beta), std::cos(beta)));
        }
    }
    return out;
}

// looking into the direction of the camera, x is left, y is up
vector<Eigen::Vector3d> getSphereVectors()
{
    vector<Eigen::Vector3d> out;
    double minAngle = toRadians(15);
    // double maxAngle = 2* PANDIA_PI;
    double maxAngle = 2 * M_PI - minAngle;
    double stepsxy = 10;                  // actual step size is one more
    double alphaInc = maxAngle / stepsxy; // alpha for values in x-y plane
    double stepsyz = 10;                  // actual step size is one more
    double betaInc = maxAngle / stepsyz;  // beta for yz plane

    // length of all vectors is one
    for (float alpha = minAngle; alpha <= maxAngle; alpha += alphaInc)
    {
        for (float beta = minAngle; beta <= maxAngle; beta += betaInc)
        {
            out.push_back(Eigen::Vector3d(std::sin(beta) * std::cos(alpha), std::sin(alpha) * std::sin(beta), std::cos(beta)));
        }
    }
    return out;
}

vector<Eigen::Vector3d> rotateHalfsphereVectors(const vector<Eigen::Vector3d> &vecs, const Eigen::Matrix3d &T)
{
    vector<Eigen::Vector3d> out;
    for (int i = 0; i < vecs.size(); i++)
    {
        out.push_back(T * vecs[i]);
    }
    return out;
}

// given a mesh and the associated pointcloud this function returns the part of the pcd thats acutally scannable.
shared_ptr<open3d::geometry::PointCloud> getScannablePcd(shared_ptr<open3d::geometry::TriangleMesh> mesh, shared_ptr<open3d::geometry::PointCloud> pcd)
{
    shared_ptr<open3d::geometry::PointCloud> out = make_shared<open3d::geometry::PointCloud>();
    auto rays = getSphereVectors();
    float thres = 1e-4;
    vector<Eigen::Vector3d> points(pcd->points_.size(), Eigen::Vector3d::Zero());
    vector<Eigen::Vector3d> normals(pcd->points_.size(), Eigen::Vector3d::Zero());
    pcd->EstimateNormals();
    pcd->NormalizeNormals();

#pragma omp parallel for
    for (int i = 0; i < pcd->points_.size(); i++)
    {
        // for every point cast rays
        // dont use straight y-axis since rodrigues will fail for parallel vectors
        // if cad model has straight y-axis aligned surface
        // Eigen::Matrix3d R = getRotationMatrixFromAtoB(pcd->normals_[i], Eigen::Vector3d(0.001, 0.999, 0));
        // auto rays = rotateHalfsphereVectors(basicrays, R);
        bool intersection = false;
        for (int j = 0; j < rays.size(); j++)
        {
            // for each ray iterate through all triangles
            for (int k = 0; k < mesh->triangles_.size(); k++)
            {
                float t = 1e6;
                float b, c;
                intersection = rayTriangleIntersect(pcd->points_[i], rays[j],
                                                    mesh->vertices_[mesh->triangles_[k](0)], mesh->vertices_[mesh->triangles_[k](1)],
                                                    mesh->vertices_[mesh->triangles_[k](2)], t, b, c);
                if (intersection && t < thres)
                { // threshold in case a point cuts the triangle it sits in, happens almost every time.
                    intersection = false;
                }
                if (intersection)
                {
                    break;
                }
            }
            if (!intersection) // the ray hit the sky, we do not need to check other rays for this point
            {
                break;
            }
        }

        if (!intersection)
        {
            // for parallel compute
            points[i] = pcd->points_[i];
            normals[i] = pcd->normals_[i];
        }
    }
    for (int i = 0; i < points.size(); i++)
    {
        if (points[i] != Eigen::Vector3d::Zero())
        {
            out->points_.push_back(points[i]);
            out->normals_.push_back(normals[i]);
        }
    }

    return out;
}

// The input 3D points are stored as columns.
Eigen::Matrix3d kabsch(const Eigen::Matrix3Xd &source, const Eigen::Matrix3Xd &target)
{

    if (source.cols() != target.cols())
        throw "Find3DAffineTransform(): input data mis-match";

    // SVD
    Eigen::MatrixXd Cov = source * target.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Find the rotation
    double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    if (d > 0) // why?
        d = 1.0;
    else
        d = -1.0;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
    I(2, 2) = d;
    Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

    return R;
}

// t is ray length parameter
// v0-v2 are vertices
// u and v are baycentric coordinates, which are written to
// taken from the internet without understanding whats going on
bool rayTriangleIntersect(
    const Eigen::Vector3d &orig, const Eigen::Vector3d &dir,
    const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2,
    float &t, float &u, float &v)
{
    float kEpsilon = 1e-6;
    Eigen::Vector3d v0v1 = v1 - v0;
    Eigen::Vector3d v0v2 = v2 - v0;
    Eigen::Vector3d pvec = dir.cross(v0v2);
    float det = v0v1.dot(pvec);
    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon)
        return false;
    float invDet = 1 / det;

    Eigen::Vector3d tvec = orig - v0;
    u = tvec.dot(pvec) * invDet;
    if (u < 0 || u > 1)
        return false;

    Eigen::Vector3d qvec = tvec.cross(v0v1);
    v = dir.dot(qvec) * invDet;
    if (v < 0 || u + v > 1)
        return false;

    t = v0v2.dot(qvec) * invDet;

    return true;
}

Eigen::Matrix4d refineRegistrationICP(std::shared_ptr<open3d::geometry::PointCloud> pcd1,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d &init, int maxIterations, double maxCorsDistance)
{
    double fitness;
    return refineRegistrationICP(pcd1, pcd2, init, fitness, maxIterations, maxCorsDistance);
}
Eigen::Matrix4d refineRegistrationICP(std::shared_ptr<open3d::geometry::PointCloud> pcd1,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d &init, double &fitness, int maxIterations, double maxCorsDistance)
{
    if (!pcd1->HasNormals())
        pcd1->EstimateNormals();
    if (!pcd2->HasNormals())
        pcd2->EstimateNormals();
    // if (!pcd1->HasColors())
    //     cout << "warning pcd1 has no colors!" << endl;
    // if (!pcd2->HasColors())
    //     cout << "warning pcd2 has no colors!" << endl;

    shared_ptr<open3d::geometry::PointCloud> pcd1f = make_shared<open3d::geometry::PointCloud>(*pcd1);
    shared_ptr<open3d::geometry::PointCloud> pcd2f = make_shared<open3d::geometry::PointCloud>(*pcd2);

    // move pcds to the center. ICP works MUCH better this way
    Eigen::Vector3d center1 = pcd1f->GetCenter();
    Eigen::Matrix4d Tcenter1 = Eigen::Matrix4d::Identity();
    Tcenter1.block<3, 1>(0, 3) = -center1;
    pcd1f->Transform(Tcenter1);
    Eigen::Vector3d center2 = pcd2f->GetCenter();
    Eigen::Matrix4d Tcenter2 = Eigen::Matrix4d::Identity();
    Tcenter2.block<3, 1>(0, 3) = -center2;
    pcd2f->Transform(Tcenter2);
    Eigen::Matrix4d newinit = Tcenter2 * init * Tcenter1.inverse();

    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    crit.max_iteration_ = maxIterations;
    bool pointToPlane = true;
    auto result = open3d::pipelines::registration::RegistrationResult();
    if (pointToPlane)
        result = open3d::pipelines::registration::RegistrationICP(*pcd1f, *pcd2f, maxCorsDistance, newinit, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
    else
        result = open3d::pipelines::registration::RegistrationICP(*pcd1f, *pcd2f, maxCorsDistance, newinit, open3d::pipelines::registration::TransformationEstimationPointToPoint(), crit);
    // double goodness = (result.fitness_ / result.inlier_rmse_) / (result.fitness_ + 1 / result.inlier_rmse_);
    fitness = result.fitness_;
    if (isnan(fitness))
    {
        cout << "Warning: Fitness in icp is nan \n";
    }
    return Tcenter2.inverse() * result.transformation_ * Tcenter1;
}

Eigen::Matrix4d refineRegistrationICP_Volume(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d &init, int maxIterations, double maxCorsDistance)
{
    if (!pcd1->HasNormals())
        pcd1->EstimateNormals();
    if (!pcd2->HasNormals())
        pcd2->EstimateNormals();
    // if (!pcd1->HasColors())
    //     cout << "warning pcd1 has no colors!" << endl;
    // if (!pcd2->HasColors())
    //     cout << "warning pcd2 has no colors!" << endl;

    shared_ptr<open3d::geometry::PointCloud> pcd1f = make_shared<open3d::geometry::PointCloud>(*pcd1);
    shared_ptr<open3d::geometry::PointCloud> pcd2f = make_shared<open3d::geometry::PointCloud>(*pcd2);

    // move pcds to the center. ICP works MUCH better this way
    Eigen::Vector3d center1 = pcd1f->GetCenter();
    Eigen::Matrix4d Tcenter1 = Eigen::Matrix4d::Identity();
    Tcenter1.block<3, 1>(0, 3) = -center1;
    pcd1f->Transform(Tcenter1);
    Eigen::Vector3d center2 = pcd2f->GetCenter();
    Eigen::Matrix4d Tcenter2 = Eigen::Matrix4d::Identity();
    Tcenter2.block<3, 1>(0, 3) = -center2;
    pcd2f->Transform(Tcenter2);
    Eigen::Matrix4d newinit = Tcenter2 * init * Tcenter1.inverse();

    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    crit.max_iteration_ = maxIterations;
    auto result = open3d::pipelines::registration::RegistrationICP(*pcd1f, *pcd2f, maxCorsDistance, newinit, open3d::pipelines::registration::TransformationEstimationPointToPoint(), crit);
    if (isnan(result.fitness_))
    {
        cout << "Warning: Fitness in icp is nan \n";
    }
    return Tcenter2.inverse() * result.transformation_ * Tcenter1;
}

std::vector<Eigen::Vector3d> getDistanceColorsGradientGeneral(const vector<double> &distances, double min, double max)
{
    std::vector<Eigen::Vector3d> out;
    float colordiff = max - min;
    for (int i = 0; i < distances.size(); i++)
    {
        if (distances[i] > max)
        {
            out.push_back(Eigen::Vector3d(1, 0, 0)); // red
        }
        else if (distances[i] < min)
        {
            out.push_back(Eigen::Vector3d(0, 1, 0)); // red
        }
        else if (distances[i] <= min + colordiff / 2)
        {
            out.push_back(Eigen::Vector3d(((distances[i] - min) * 2) / colordiff, 1, 0)); // green base, increase red
        }
        else if (distances[i] > min + colordiff / 2)
        {
            out.push_back(Eigen::Vector3d(1, 1 - (distances[i] - min - colordiff / 2) / colordiff, 0)); // red base, decrease green
        }
    }
    return out;
}
