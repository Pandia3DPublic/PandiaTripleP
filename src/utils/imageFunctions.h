#pragma once
#include "cameras/CameraWrapper.h"

void applyNormalFilter(open3d::geometry::Image &depth, CameraWrapper& cam, std::shared_ptr<open3d::geometry::PointCloud> pcd = nullptr);

//collects all pixel and ignore zero values in single image
std::shared_ptr<open3d::geometry::RGBDImage> getAverageImg(std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> &rgbds);
//if one pixel is zero, set resulting pixel to zero
std::shared_ptr<open3d::geometry::RGBDImage> getSoftAverageImgFilter(std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> &rgbds);
//same as soft but with additional outlier detection
std::shared_ptr<open3d::geometry::RGBDImage> getAverageImgFilter(std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> &rgbds);

bool isDepthImageValid(const open3d::geometry::Image &depth);