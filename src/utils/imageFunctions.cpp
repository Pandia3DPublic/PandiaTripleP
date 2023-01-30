#include "imageFunctions.h"
#include "3DFunctions.h"
#include "conversions.h"
using namespace std;


//the pcd which corresponds to the depth image can be passed for performance reasons
void applyNormalFilter(open3d::geometry::Image &depth, CameraWrapper& cam, shared_ptr<open3d::geometry::PointCloud> pcd)
{
    double thres = cos(toRadians(70));

    if (pcd == nullptr){
        pcd = cam.pcdFromDepth(depth);
        pcd->EstimateNormals();
        pcd->OrientNormalsTowardsCameraLocation();
    }

    for (int i = 0; i < pcd->points_.size(); i++)
    {
        if (abs(pcd->normals_[i].dot(pcd->points_[i])) < thres)
        {
            Eigen::Vector2i idx = cam.PointtoPixel(pcd->points_[i]);
            *depth.PointerAt<float>(idx(1), idx(0)) = 0;
        }
    }
}



// get the average of multiple rgbd images
shared_ptr<open3d::geometry::RGBDImage> getAverageImg(vector<shared_ptr<open3d::geometry::RGBDImage>> &rgbds)
{
    if (rgbds.size() == 1)
    {
        return rgbds.front();
    }

    shared_ptr<open3d::geometry::RGBDImage> out = make_shared<open3d::geometry::RGBDImage>();
    out->color_.Prepare(rgbds.front()->color_.width_, rgbds.front()->color_.height_, 3, 1);
    out->depth_.Prepare(rgbds.front()->depth_.width_, rgbds.front()->depth_.height_, 1, 4);

    for (int i = 0; i < out->depth_.height_; i++)
    {
        for (int j = 0; j < out->depth_.width_; j++)
        {
            float d_out = 0;
            int count = 0;
            int rgbk = 0;
            for (int k = 0; k < rgbds.size(); k++)
            {
                if (*rgbds[k]->depth_.PointerAt<float>(j, i) != 0 && !isnan(*rgbds[k]->depth_.PointerAt<float>(j, i)))
                {
                    d_out += *rgbds[k]->depth_.PointerAt<float>(j, i);
                    count++;
                    rgbk = k;
                }
            }

            if (count != 0)
            {
                *out->depth_.PointerAt<float>(j, i) = d_out / count;
                *out->color_.PointerAt<u_char>(j, i, 0) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 0);
                *out->color_.PointerAt<u_char>(j, i, 1) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 1);
                *out->color_.PointerAt<u_char>(j, i, 2) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 2);
            }
        }
    }
    return out;
}
// get the average of multiple rgbd images. Depending in the implementation this also filters for inconsistent pixels.
shared_ptr<open3d::geometry::RGBDImage> getSoftAverageImgFilter(vector<shared_ptr<open3d::geometry::RGBDImage>> &rgbds)
{
    if (rgbds.size() == 1)
    {
        return rgbds.front();
    }

    shared_ptr<open3d::geometry::RGBDImage> out = make_shared<open3d::geometry::RGBDImage>();
    out->color_.Prepare(rgbds.front()->color_.width_, rgbds.front()->color_.height_, 3, 1);
    out->depth_.Prepare(rgbds.front()->depth_.width_, rgbds.front()->depth_.height_, 1, 4);

    for (int i = 0; i < out->depth_.height_; i++)
    {
        for (int j = 0; j < out->depth_.width_; j++)
        {
            float d_out = 0;
            int count = 0;
            int rgbk = 0;
            for (int k = 0; k < rgbds.size(); k++)
            {
                if (*rgbds[k]->depth_.PointerAt<float>(j, i) != 0)
                {
                    d_out += *rgbds[k]->depth_.PointerAt<float>(j, i);
                    count++;
                    rgbk = k;
                }
            }

            if (count == rgbds.size())
            {
                *out->depth_.PointerAt<float>(j, i) = d_out / count;
                *out->color_.PointerAt<u_char>(j, i, 0) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 0);
                *out->color_.PointerAt<u_char>(j, i, 1) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 1);
                *out->color_.PointerAt<u_char>(j, i, 2) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 2);
            }
        }
    }
    return out;
}
// get the average of multiple rgbd images. Depending in the implementation this also filters for inconsistent pixels.
shared_ptr<open3d::geometry::RGBDImage> getAverageImgFilter(vector<shared_ptr<open3d::geometry::RGBDImage>> &rgbds)
{
    if (rgbds.size() == 1)
    {
        return rgbds.front();
    }

    shared_ptr<open3d::geometry::RGBDImage> out = make_shared<open3d::geometry::RGBDImage>();
    out->color_.Prepare(rgbds.front()->color_.width_, rgbds.front()->color_.height_, 3, 1);
    out->depth_.Prepare(rgbds.front()->depth_.width_, rgbds.front()->depth_.height_, 1, 4);

    for (int i = 0; i < out->depth_.height_; i++)
    {
        for (int j = 0; j < out->depth_.width_; j++)
        {
            float d_out = 0;
            int count = 0;
            int rgbk = 0;
            for (int k = 0; k < rgbds.size(); k++)
            {
                if (*rgbds[k]->depth_.PointerAt<float>(j, i) != 0)
                {
                    d_out += *rgbds[k]->depth_.PointerAt<float>(j, i);
                    count++;
                    rgbk = k;
                }
            }
            bool deviatePixel = false;
            if (count == 0)
            {
                deviatePixel = true;
            }
            else
            {
                d_out /= count;
                for (int k = 0; k < rgbds.size(); k++)
                {
                    if (std::abs(*rgbds[k]->depth_.PointerAt<float>(j, i) - d_out) > 0.015)
                    {
                        deviatePixel = true;
                    }
                }
            }
            if (!deviatePixel)
            {
                // if (count == rgbds.size()) {
                // if(count != 0){
                *out->depth_.PointerAt<float>(j, i) = d_out;
                *out->color_.PointerAt<u_char>(j, i, 0) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 0);
                *out->color_.PointerAt<u_char>(j, i, 1) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 1);
                *out->color_.PointerAt<u_char>(j, i, 2) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 2);
            }
        }
    }
    return out;
}

bool isDepthImageValid(const open3d::geometry::Image &depth)
{
    int nInvalidPixels = 0;
// #pragma omp parallel for reduction(+ \
//                                    : nInvalidPixels) // this causes issues detected by valgrind
    for (int j = 0; j < depth.height_; j++)
    {
        for (int k = 0; k < depth.width_; k++)
        {
            // note the coordinate change
            float d = *depth.PointerAt<float>(k, j);
            if (d <= 0 || d > 1e6)
            {
                nInvalidPixels++;
            }
        }
    }
    if (nInvalidPixels > 0.75 * depth.height_ * depth.width_)
    {
        return false;
    }
    else
    {
        return true;
    }
}
