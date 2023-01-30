#include "miscFunctions.h"
#include "3DFunctions.h"
#include "imageFunctions.h"
bool SortbyAlphabet(const std::string &s1, const std::string &s2)
{
    return (s1 < s2);
}

double edgeFunction(const Eigen::Vector2d &v0, const Eigen::Vector2d &v1, const Eigen::Vector2d &p)
{
    return (p(0) - v0(0)) * (v1(1) - v0(1)) - (p(1) - v0(1)) * (v1(0) - v0(0));
}

std::vector<int> getInnerToOuterEdgeMapping(std::vector<Eigen::Vector2i> &innerEdge, std::vector<Eigen::Vector2i> &outerEdge)
{
    std::vector<int> out;
    for (auto &edge : innerEdge)
    {
        float dmin = 1e6;
        int idx;
        for (int i = 0; i < outerEdge.size(); i++)
        {
            float d = (outerEdge[i] - edge).norm();
            if (d < dmin)
            {
                dmin = d;
                idx = i;
            }
        }
        out.push_back(idx);
    }
    return out;
}

bool IsVisible(const Eigen::Vector3d& p, const Eigen::MatrixXd& depth_buffer, CameraWrapper& cam){
    // open3d::visualization::DrawGeometries({EigenToO3DDepthImage(depth_buffer)});
    double epsilon = 0.01; // large for depth noise
    Eigen::Vector2i ind = cam.PointtoPixel(p);
    if (cam.InsideDepthImage(ind)){
        if(p(2) <= depth_buffer(ind(0), ind(1)) + epsilon){
            return true;
        }
    }
    return false;
}