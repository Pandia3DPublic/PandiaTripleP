#include "vishelper.h"
#include "conversions.h"
using namespace std;
std::shared_ptr<open3d::geometry::LineSet> getOrigin()
{
    auto ls = std::make_shared<open3d::geometry::LineSet>();
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    Eigen::Vector3d x(1.0, 0.0, 0.0);
    Eigen::Vector3d y(0.0, 1.0, 0.0);
    Eigen::Vector3d z(0.0, 0.0, 1.0);

    Eigen::Vector3d xcolor(1.0, 0.0, 0.0);
    Eigen::Vector3d ycolor(0.0, 1.0, 0.0);
    Eigen::Vector3d zcolor(0.0, 0.0, 1.0);

    ls->points_.push_back(zero);
    ls->points_.push_back(x);
    ls->points_.push_back(y);
    ls->points_.push_back(z);
    ls->lines_.push_back(Eigen::Vector2i(0, 1));
    ls->lines_.push_back(Eigen::Vector2i(0, 2));
    ls->lines_.push_back(Eigen::Vector2i(0, 3));
    ls->colors_.push_back(xcolor);
    ls->colors_.push_back(ycolor);
    ls->colors_.push_back(zcolor);

    return ls;
}

std::shared_ptr<open3d::geometry::LineSet> getLineSet(const Eigen::Matrix3d &axis)
{
    auto ls = std::make_shared<open3d::geometry::LineSet>();
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    Eigen::Vector3d x = axis.block<3, 1>(0, 0);
    Eigen::Vector3d y = axis.block<3, 1>(0, 1);
    Eigen::Vector3d z = axis.block<3, 1>(0, 2);

    Eigen::Vector3d xcolor(0.0, 0.0, 0.0); // black
    Eigen::Vector3d ycolor(0.0, 1.0, 1.0); // green-blue
    Eigen::Vector3d zcolor(1.0, 0.0, 1.0); // pink

    ls->points_.push_back(zero);
    ls->points_.push_back(x);
    ls->points_.push_back(y);
    ls->points_.push_back(z);
    ls->lines_.push_back(Eigen::Vector2i(0, 1));
    ls->lines_.push_back(Eigen::Vector2i(0, 2));
    ls->lines_.push_back(Eigen::Vector2i(0, 3));
    ls->colors_.push_back(xcolor);
    ls->colors_.push_back(ycolor);
    ls->colors_.push_back(zcolor);

    return ls;
}

std::shared_ptr<open3d::geometry::LineSet> getLineSet(const Eigen::Vector3d &vec)
{
    auto ls = std::make_shared<open3d::geometry::LineSet>();
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    Eigen::Vector3d color(0.0, 0.0, 0.0); // black
    ls->points_.push_back(zero);
    ls->points_.push_back(vec);
    ls->lines_.push_back(Eigen::Vector2i(0, 1));
    ls->colors_.push_back(color);
    return ls;
}

shared_ptr<open3d::geometry::TriangleMesh> getCameraArrow(const Eigen::Matrix4d &trans)
{
    auto arrow = open3d::geometry::TriangleMesh::CreateArrow(0.05, 0.075, 0.2, 0.1);
    arrow->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
    arrow->Transform(trans);
    return arrow;
}

void showEdges(cv::Mat img)
{
    using namespace cv;
    // GaussianBlur(img, img, Size(3, 3), 0, 0, BORDER_DEFAULT);
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(img_gray, img_gray);
    Mat grad_y, abs_grad_y;
    cv::Sobel(img_gray, grad_y, CV_16S, 0, 1);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    cv::addWeighted(abs_grad_y, 0.5, abs_grad_y, 0, 0, img_gray);
    cv::imshow("Test", img_gray);
    cv::waitKey(0);
}
void showEdges(open3d::geometry::RGBDImage &rgbd)
{
    for (int i = 0; i < rgbd.color_.height_; i++)
    {
        for (int j = 0; j < rgbd.color_.width_; j++)
        {
            if (*rgbd.depth_.PointerAt<float>(j, i) == 0)
            {
                *rgbd.color_.PointerAt<uchar>(j, i, 0) = 0;
                *rgbd.color_.PointerAt<uchar>(j, i, 1) = 0;
                *rgbd.color_.PointerAt<uchar>(j, i, 2) = 0;
            }
        }
    }
    cv::Mat img = Open3DToOpenCV(rgbd.color_);
    showEdges(img);
}

std::shared_ptr<open3d::geometry::LineSet> visualizeHalfsphereVectors(const vector<Eigen::Vector3d> &vecs, const Eigen::Vector3d &p)
{

    auto ls = std::make_shared<open3d::geometry::LineSet>();
    ls->points_.push_back(p);
    for (int i = 0; i < vecs.size(); i++)
    {
        ls->points_.push_back(vecs[i]);
        ls->lines_.push_back(Eigen::Vector2i(0, i + 1));
    }
    return ls;
}
