#pragma once

Eigen::Matrix4d getflip();
Eigen::Matrix3d getflip3();

Eigen::Matrix4d getRotationMatrixFromVectorAndAngle(const Eigen::Vector3d &vec_in, const double &a);
Eigen::Matrix3d getRotationMatrixFromAtoB(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

double bilinearInterpolation(const double &v_tl, const double &v_bl, const double &v_br, const double &v_tr, 
                             const Eigen::Vector2d &p_query);

template <typename T>
Eigen::Matrix4d getRx(T a)
{
    Eigen::Matrix4d rx;
    rx << 1, 0, 0, 0,
        0, std::cos(a), -std::sin(a), 0,
        0, std::sin(a), std::cos(a), 0,
        0, 0, 0, 1;
    return rx;
}

template <typename T>
Eigen::Matrix4d getRy(T a)
{
    Eigen::Matrix4d ry;
    ry << std::cos(a), 0, std::sin(a), 0,
        0, 1, 0, 0,
        -std::sin(a), 0, std::cos(a), 0,
        0, 0, 0, 1;
    return ry;
}
template <typename T>
Eigen::Matrix4d getRz(T a)
{
    Eigen::Matrix4d rz;
    rz << std::cos(a), -std::sin(a), 0, 0,
        std::sin(a), std::cos(a), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return rz;
}

template <typename T>
Eigen::Matrix4d getTrans(const T &a)
{
    Eigen::Matrix4d t;
    t << 1, 0, 0, a(0),
        0, 1, 0, a(1),
        0, 0, 1, a(2),
        0, 0, 0, 1;
    return t;
}
