#pragma once
class Frustum
{
public:
    Frustum(const double &h_, const double &height_near_, const double &width_near_, const double &height_far_, const double &width_far_);
    ~Frustum(){};

    double h; //height between near and far plane
    double height_near; 
    double width_near; 
    double height_far; 
    double width_far;

    double getVolume();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};