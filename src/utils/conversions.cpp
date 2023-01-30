#include "conversions.h"

using namespace cv;
using namespace std;

// no copy, sets data pointer in cv mat
cv::Mat Open3DToOpenCV(const open3d::geometry::Image &img)
{
    if (img.num_of_channels_ == 3 && img.bytes_per_channel_ == 1)
    {
        cv::Mat out(img.height_, img.width_, CV_8UC3, (void *)img.data_.data());
        return out;
    }
    else if (img.num_of_channels_ == 1 && img.bytes_per_channel_ == 2)
    {
        cv::Mat out(img.height_, img.width_, CV_16U, (void *)img.data_.data());
        return out;
    }
    else if (img.num_of_channels_ == 1 && img.bytes_per_channel_ == 4)
    {
        cv::Mat out(img.height_, img.width_, CV_32F, (void *)img.data_.data());
        return out;
    }
    else
    {
        cout << "warning conversion not implemented for input image properties" << endl;
        cv::Mat out;
        return out;
    }
}

// memcopy to o3d
std::shared_ptr<open3d::geometry::Image> OpenCVToOpen3D(const cv::Mat &img)
{
    auto out = make_shared<open3d::geometry::Image>();
    out->Prepare(img.cols, img.rows, img.channels(), img.elemSize() / img.channels());
    if (img.isContinuous())
    {
        memcpy(out->data_.data(), img.data, img.total() * img.elemSize());
    }
    else
    {
        cout << "warning opencv image is not continious!" << endl;
    }
    return out;
}

cv::Mat KinectImageToOpenCV(const k4a::image &im)
{
    cv::Mat out;
    if (im.get_format() == k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32)
    {
        cv::Mat cv_image_with_alpha = cv::Mat(im.get_height_pixels(), im.get_width_pixels(), CV_8UC4, (void *)im.get_buffer());
        cv::cvtColor(cv_image_with_alpha, out, cv::COLOR_BGRA2RGB);
    }
    else if (im.get_format() == k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16)
    {
        out = cv::Mat(im.get_height_pixels(), im.get_width_pixels(), CV_16U, (void *)im.get_buffer(), cv::Mat::AUTO_STEP).clone();
    }
    else
    {
        cout << "Warning: KinectImageToOpenCV for this image format is not supported" << endl;
    }
    return out;
}

std::shared_ptr<open3d::geometry::Image> KinectImageToOpen3D(const k4a::image &im)
{
    auto out = make_shared<open3d::geometry::Image>();
    if (im.get_format() == k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32)
    {
        out->Prepare(im.get_width_pixels(), im.get_height_pixels(), 3, sizeof(uint8_t));
        const uint8_t *ch = im.get_buffer();
        for (int i = 0; i < im.get_height_pixels(); i++)
        {
            for (int j = 0; j < im.get_width_pixels(); j++)
            {
                *out->PointerAt<uint8_t>(j, i, 2) = *ch++; // b
                *out->PointerAt<uint8_t>(j, i, 1) = *ch++; // g
                *out->PointerAt<uint8_t>(j, i, 0) = *ch++; // r
                ch++;                                      // skip alpha channel
            }
        }
    }
    else if (im.get_format() == k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16)
    {
        out->Prepare(im.get_width_pixels(), im.get_height_pixels(), 1, sizeof(uint16_t));
        const uint16_t *d = (uint16_t *)(void *)im.get_buffer();
        for (int i = 0; i < im.get_height_pixels(); i++)
        {
            for (int j = 0; j < im.get_width_pixels(); j++)
            {
                *out->PointerAt<uint16_t>(j, i) = *d++;
            }
        }
    }
    else
    {
        cout << "Warning: KinectImageToOpen3D for this image format is not supported" << endl;
    }
    return out;
}


k4a::image Open3DToKinectImage(const open3d::geometry::Image &img)
{
    k4a::image out;
    if (img.num_of_channels_ == 3 && img.bytes_per_channel_ == 1)
    {
        out = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, img.width_, img.height_, img.width_ * (int)sizeof(uint32_t));
        uint8_t *dst_data = (uint8_t *)(void *)k4a_image_get_buffer(out.handle());
        memset(dst_data, 0, (size_t)img.width_ * (size_t)img.height_ * sizeof(uint32_t));
        for (int i = 0; i < img.height_; i++)
        {
            for (int j = 0; j < img.width_; j++)
            {
                *dst_data++ = *img.PointerAt<uint8_t>(j, i, 2); // b
                *dst_data++ = *img.PointerAt<uint8_t>(j, i, 1); // g
                *dst_data++ = *img.PointerAt<uint8_t>(j, i, 0); // r
                dst_data++;                                     // skip alpha
            }
        }
    }
    else if (img.num_of_channels_ == 1 && img.bytes_per_channel_ == 2)
    {
        out = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, img.width_, img.height_, img.width_ * (int)sizeof(uint16_t));
        uint16_t *dst_data = (uint16_t *)(void *)k4a_image_get_buffer(out.handle());
        memset(dst_data, 0, (size_t)img.width_ * (size_t)img.height_ * sizeof(uint16_t));
        for (int i = 0; i < img.height_; i++)
        {
            for (int j = 0; j < img.width_; j++)
            {
                *dst_data++ = *img.PointerAt<uint16_t>(j, i);
            }
        }
    }
    else if (img.num_of_channels_ == 1 && img.bytes_per_channel_ == 4)
    {
        out = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, img.width_, img.height_, img.width_ * (int)sizeof(uint16_t));
        uint16_t *dst_data = (uint16_t *)(void *)k4a_image_get_buffer(out.handle());
        memset(dst_data, 0, (size_t)img.width_ * (size_t)img.height_ * sizeof(uint16_t));
        for (int i = 0; i < img.height_; i++)
        {
            for (int j = 0; j < img.width_; j++)
            {
                *dst_data++ = (uint16_t)(*img.PointerAt<float>(j, i) * 1000);
            }
        }
    }
    else
    {
        cout << "Warning: Open3DToKinectImage for this image format is not supported" << endl;
    }
    return out;
}

std::shared_ptr<open3d::geometry::PointCloud> KinectPCDToOpen3D(const k4a::image &k4apcd, const k4a::image *color)
{
    Mat cvPcd = Mat(Size(k4apcd.get_width_pixels(), k4apcd.get_height_pixels()), CV_16UC3, (void *)k4apcd.get_buffer(), Mat::AUTO_STEP);
    auto out = make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < cvPcd.rows; i++)
    {
        for (int j = 0; j < cvPcd.cols; j++)
        {
            if (cvPcd.at<cv::Vec3s>(i, j)(2) != 0)
            {
                out->points_.push_back(Eigen::Vector3d(cvPcd.at<cv::Vec3s>(i, j)(0),
                                                       cvPcd.at<cv::Vec3s>(i, j)(1),
                                                       cvPcd.at<cv::Vec3s>(i, j)(2)) /
                                       1000.0);
            }
        }
    }

    if (color != nullptr && color->get_width_pixels() == k4apcd.get_width_pixels() && color->get_height_pixels() == k4apcd.get_height_pixels())
    {
        Mat cvColor4 = Mat(Size(color->get_width_pixels(), color->get_height_pixels()), CV_8UC4, (void *)color->get_buffer(), Mat::AUTO_STEP);
        Mat cvColor = Mat(Size(color->get_width_pixels(), color->get_height_pixels()), CV_8UC3);
        cvtColor(cvColor4, cvColor, cv::ColorConversionCodes::COLOR_BGRA2RGB); // remove alpha channel and change ordering
        for (int i = 0; i < cvColor.rows; i++)
        {
            for (int j = 0; j < cvColor.cols; j++)
            {
                if (cvPcd.at<cv::Vec3s>(i, j)(2) != 0)
                {

                    out->colors_.push_back(Eigen::Vector3d(cvColor.at<cv::Vec3b>(i, j)(0) / 255.0,
                                                           cvColor.at<cv::Vec3b>(i, j)(1) / 255.0,
                                                           cvColor.at<cv::Vec3b>(i, j)(2) / 255.0));
                }
            }
        }
    }
    return out;
}

// Convert rs2::frame to cv::Mat, note this only wraps the image data and does not copy
cv::Mat FrametoMat(const rs2::frame &f)
{
    using namespace cv;
    using namespace rs2;
    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();
    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        auto r_bgr = Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
        Mat r_rgb;
        cvtColor(r_bgr, r_rgb, COLOR_BGR2RGB);
        return r_rgb;
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        return Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat FrametoMatMeters(const rs2::depth_frame &f)
{
    cv::Mat dm = FrametoMat(f);
    dm.convertTo(dm, CV_64F);
    dm = dm * f.get_units();
    return dm;
}

// Converts depth frame to a matrix of doubles with distances in millimeters
cv::Mat FrametoMatMillimeters(const rs2::depth_frame &f)
{
    cv::Mat dm = FrametoMat(f);
    float factor = f.get_units() * 1000;
    dm.convertTo(dm, CV_64F);
    dm = dm * factor;
    dm.convertTo(dm, CV_16S);
    return dm;
}

double toDegrees(const double &radians)
{
    return radians * 180.0 / M_PI;
}
double toRadians(const double &degrees)
{
    return degrees * M_PI / 180.0;
}

// very specific for camera readout conversions
Eigen::Matrix4d arraysToEigenRow(float *rot, float *trans)
{
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) << rot[0], rot[1], rot[2],
        rot[3], rot[4], rot[5],
        rot[6], rot[7], rot[8];
    out.block<3, 1>(0, 3) << trans[0], trans[1], trans[2];
    return out;
}

// very specific for camera readout conversions
Eigen::Matrix4d arraysToEigenColumn(float *rot, float *trans)
{
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) << rot[0], rot[3], rot[6],
        rot[1], rot[4], rot[7],
        rot[2], rot[5], rot[8];
    out.block<3, 1>(0, 3) << trans[0], trans[1], trans[2];
    return out;
}

shared_ptr<open3d::geometry::Image> EigenToO3DDepthImage(const Eigen::MatrixXd &depth)
{
    auto out = make_shared<open3d::geometry::Image>();
    out->Prepare(depth.cols(), depth.rows(), 1, 4);
    for (int i = 0; i < depth.rows(); i++)
    {
        for (int j = 0; j < depth.cols(); j++)
        {
            *out->PointerAt<float>(j, i) = depth(i, j);
        }
    }
    return out;
}

Eigen::MatrixXd O3DDepthtoEigen(shared_ptr<open3d::geometry::Image> depth)
{
    Eigen::MatrixXd out = Eigen::MatrixXd::Zero(depth->height_, depth->width_);
    for (int i = 0; i < depth->height_; i++)
    {
        for (int j = 0; j < depth->width_; j++)
        {
            out(i, j) = *depth->PointerAt<float>(j, i);
        }
    }
    return out;
}



cv::Matx33d eigenToOpenCv33(const Eigen::Matrix3d &m)
{
    cv::Matx33d out;
    out(0, 0) = m(0, 0);
    out(0, 1) = m(0, 1);
    out(0, 2) = m(0, 2);
    out(1, 0) = m(1, 0);
    out(1, 1) = m(1, 1);
    out(1, 2) = m(1, 2);
    out(2, 0) = m(2, 0);
    out(2, 1) = m(2, 1);
    out(2, 2) = m(2, 2);
    return out;
}
cv::Matx33f eigenToOpenCv33f(const Eigen::Matrix3d &m)
{
    cv::Matx33f out;
    out(0, 0) = m(0, 0);
    out(0, 1) = m(0, 1);
    out(0, 2) = m(0, 2);
    out(1, 0) = m(1, 0);
    out(1, 1) = m(1, 1);
    out(1, 2) = m(1, 2);
    out(2, 0) = m(2, 0);
    out(2, 1) = m(2, 1);
    out(2, 2) = m(2, 2);
    return out;
}


Eigen::Matrix3d opencv33ToEigen(const cv::Matx33d m)
{
    Eigen::Matrix3d out;
    out(0, 0) = m(0, 0);
    out(0, 1) = m(0, 1);
    out(0, 2) = m(0, 2);
    out(1, 0) = m(1, 0);
    out(1, 1) = m(1, 1);
    out(1, 2) = m(1, 2);
    out(2, 0) = m(2, 0);
    out(2, 1) = m(2, 1);
    out(2, 2) = m(2, 2);
    return out;
}
