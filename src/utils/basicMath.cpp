#include "basicMath.h"


using namespace std;
Eigen::Matrix4d getflip()
{
    Eigen::Matrix4d flip;
    flip << 1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    return flip;
}

Eigen::Matrix3d getflip3()
{
    Eigen::Matrix3d flip;
    flip << 1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    return flip;
}

// get the rotation matrix around an arbitrary vector u for an angle a.
Eigen::Matrix4d getRotationMatrixFromVectorAndAngle(const Eigen::Vector3d &v_ex, const double &a)
{
    Eigen::Vector3d k = v_ex;
    k.normalize();
    // 2.construct hte cross product matrix
    Eigen::Matrix3d K;
    K << 0, -k(2), k(1),
        k(2), 0, -k(0),
        -k(1), k(0), 0;
    // 4. get the rotation matrix using the rodrigues formula
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() + sin(a) * K + (1 - cos(a)) * K * K;
    return out;
}

// get the rotation matrix from a to b by using the Rodrigues rotation formular
// note: This might be the wrong way round and give the transform from b to a
Eigen::Matrix3d getRotationMatrixFromAtoB(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    if (a.dot(b) == -a.norm() * b.norm())
    {
        cout << "Warning axis of rotation not defined for parallel vectors \n";
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d out;
    // 1.construct axis of rotation
    Eigen::Vector3d k = -a.cross(b);
    k.normalize();
    // 2.construct hte cross product matrix
    Eigen::Matrix3d K;
    K << 0, -k(2), k(1),
        k(2), 0, -k(0),
        -k(1), k(0), 0;
    // 3.get the angle of roation
    double angle = acos(a.dot(b) / (a.norm() * b.norm()));
    // 4. get the rotation matrix using the rodrigues formula
    out = Eigen::Matrix3d::Identity() + sin(angle) * K + (1 - cos(angle)) * K * K;
    return out;
}

double bilinearInterpolation(const double &v_tl, const double &v_bl, const double &v_br, const double &v_tr,
                             const Eigen::Vector2d &p_query){

    Eigen::Vector2d p_tl(floor(p_query(0)),floor(p_query(1)));
    Eigen::Vector2d p_bl(ceil(p_query(0)),floor(p_query(1)));
    Eigen::Vector2d p_br(ceil(p_query(0)),ceil(p_query(1)));
    Eigen::Vector2d p_tr(floor(p_query(0)),ceil(p_query(1)));
    //4 times area times value
    Eigen::Vector2d tmp_vec = p_query-p_tl;
    tmp_vec(0) = std::abs(tmp_vec(0));
    tmp_vec(1) = std::abs(tmp_vec(1));
    double tl = tmp_vec(0) * tmp_vec(1) * v_tl;

    tmp_vec = p_query - p_bl;
    tmp_vec(0) = std::abs(tmp_vec(0));
    tmp_vec(1) = std::abs(tmp_vec(1));
    double bl = tmp_vec(0) * tmp_vec(1) * v_bl;

    tmp_vec = p_query - p_br;
    tmp_vec(0) = std::abs(tmp_vec(0));
    tmp_vec(1) = std::abs(tmp_vec(1));
    double br = tmp_vec(0) * tmp_vec(1) * v_br;

    tmp_vec = p_query - p_tr;
    tmp_vec(0) = std::abs(tmp_vec(0));
    tmp_vec(1) = std::abs(tmp_vec(1));
    double tr = tmp_vec(0) * tmp_vec(1) * v_tr;

    return tl + bl + br + tr;
}