#include "frustum.h"
//@height_near height of near plane
Frustum::Frustum(const double &h_, const double &height_near_,
                 const double &width_near_, const double &height_far_,
                 const double &width_far_) : h(h_), height_near(height_near_),
                                             width_near(width_near_), height_far(height_far_),
                                             width_far(width_far_)
{
}


double Frustum::getVolume(){

    double Anear = height_near * width_near;
    double Afar = height_far * width_far;
    return h / 3 * (Anear + Afar + sqrt(Anear * Afar));
}