#include "CameraWrapper.h"
#include "utils/conversions.h"
#include "utils/fileIO.h"
#include "utils/3DFunctions.h"
#include "utils/vishelper.h"
#include "utils/basicMath.h"
#include "utils/miscFunctions.h"

using namespace std;

CameraWrapper::CameraWrapper(/* args */)
{
}

CameraWrapper::~CameraWrapper()
{
    disconnect();
}

//########### virtual functions ###############

bool CameraWrapper::connectBySerialNumber(const std::string &serialNumber)
{
    cout << "Warning: NEVER USE CONNECT IN CAMERAWRAPPER BASE CLASS \n";
    return false;
}

bool CameraWrapper::record(int n, int n_wait)
{
    cout << "Warning: NEVER USE RECORD IN CAMERAWRAPPER BASE CLASS \n";
    return false;
}

bool CameraWrapper::disconnect()
{
    return false;
}

//########## base class functions ##############called specific enseneso destructor

void CameraWrapper::clearBuffers()
{
    rgbdImages.clear();
    Pcds.clear();
    CroppedPcds.clear();
    CornersListofLists.clear();
    FRCalibrationImages.clear();
    T_camToRef.clear();
    T_AddedToRef.clear();
    FrameTimestamps.clear();
}

std::shared_ptr<open3d::geometry::RGBDImage> CameraWrapper::getCroppedRGBDImageWithGround(std::shared_ptr<open3d::geometry::RGBDImage> in)
{
    auto out = make_shared<open3d::geometry::RGBDImage>(*in);

    float halfWidth = std::floor(out->depth_.width_ / 2.0);
    float halfHeight = std::floor(out->depth_.height_ / 2.0);

    // in pixels
    float left_thres = crop_left / 100 * halfWidth;
    float right_thres = (1 - crop_right / 100) * halfWidth + halfWidth;
    float top_thres = crop_top / 100 * halfHeight;
    float bottom_thres = (1 - crop_bottom / 100) * halfHeight + halfHeight;
    float d_thres = dmax - crop_depth / 100.0 * dmax;
    float d_thres_inv = dmax - (1 - crop_depth_inv / 100.0) * dmax;
    int nullCount = 0;
    // #pragma omp parallel for
    for (int i = 0; i < out->depth_.height_; i++)
    {
        for (int j = 0; j < out->depth_.width_; j++)
        {
            // note the coordinate change
            float d = abs(*out->depth_.PointerAt<float>(j, i));
            Eigen::Vector3d p = PixeltoPoint(i, j, d);
            if (!(d < d_thres && d > d_thres_inv && j < right_thres && j > left_thres &&
                  i > top_thres && i < bottom_thres))
            {
                *out->depth_.PointerAt<float>(j, i) = 0;
            }
            if (*out->depth_.PointerAt<float>(j, i) == 0)
            {
                nullCount++;
            }
        }
    }
    if (nullCount == out->depth_.width_ * out->depth_.height_)
    {
        cout << "Warning: Depth image of cam " << SerialNumber << " is all zeros after cropping!" << endl;
    }
    return out;
}

std::shared_ptr<open3d::geometry::RGBDImage> CameraWrapper::getCroppedRGBDImage(std::shared_ptr<open3d::geometry::RGBDImage> in)
{
    auto out = make_shared<open3d::geometry::RGBDImage>(*in);

    float halfWidth = std::floor(out->depth_.width_ / 2.0);
    float halfHeight = std::floor(out->depth_.height_ / 2.0);

    // in pixels
    float left_thres = crop_left / 100 * halfWidth;
    float right_thres = (1 - crop_right / 100) * halfWidth + halfWidth;
    float top_thres = crop_top / 100 * halfHeight;
    float bottom_thres = (1 - crop_bottom / 100) * halfHeight + halfHeight;
    float d_thres = dmax - crop_depth / 100.0 * dmax;
    float d_thres_inv = dmax - (1 - crop_depth_inv / 100.0) * dmax;

    Eigen::Vector3d n_gravity = GravityVector;
    Eigen::Vector3d p_plane = (crop_ground_height + crop_ground_height_fine / 100) * n_gravity; // point on the plane
    int nullCount = 0;
    // #pragma omp parallel for
    for (int i = 0; i < out->depth_.height_; i++)
    {
        for (int j = 0; j < out->depth_.width_; j++)
        {
            // note the coordinate change
            float d = abs(*out->depth_.PointerAt<float>(j, i));
            Eigen::Vector3d p = PixeltoPoint(i, j, d);
            if (!(d < d_thres && d > d_thres_inv && j < right_thres && j > left_thres &&
                  i > top_thres && i < bottom_thres && (p - p_plane).dot(n_gravity) < 0))
            {
                *out->depth_.PointerAt<float>(j, i) = 0;
            }
            if (*out->depth_.PointerAt<float>(j, i) == 0)
            {
                nullCount++;
            }
        }
    }
    if (nullCount == out->depth_.width_ * out->depth_.height_)
    {
        cout << "Warning: Depth image of cam " << SerialNumber << " is all zeros after cropping!" << endl;
    }
    return out;
}

// todo redundant croppping code!!
shared_ptr<open3d::geometry::PointCloud> CameraWrapper::getCroppedPcd(shared_ptr<open3d::geometry::PointCloud> pcd_in)
{
    auto out = make_shared<open3d::geometry::PointCloud>();
    if (pcd_in == nullptr)
    {
        cout << "Warning: Pcd is nullptr in getCroppedPcd!" << endl;
        return out;
    }

    float halfWidth = std::floor(DepthLDTIntrinsic.width_ / 2.0);
    float halfHeight = std::floor(DepthLDTIntrinsic.height_ / 2.0);

    // in pixels
    float left_thres = crop_left / 100 * halfWidth;
    float right_thres = (1 - crop_right / 100) * halfWidth + halfWidth;
    float top_thres = crop_top / 100 * halfHeight;
    float bottom_thres = (1 - crop_bottom / 100) * halfHeight + halfHeight;
    float d_thres = dmax - crop_depth / 100.0 * dmax;
    float d_thres_inv = dmax - (1 - crop_depth_inv / 100.0) * dmax;

    Eigen::Vector3d n_gravity = GravityVector;
    Eigen::Vector3d p_plane = (crop_ground_height + crop_ground_height_fine / 100) * n_gravity; // point on the plane
    // vis gravity vector
    // cout << "gravity in getCroppedPcd:\n " << n_gravity << endl;
    // open3d::visualization::DrawGeometries({getLineSet(n_gravity), pcd_in, getOrigin()});
    for (int k = 0; k < pcd_in->points_.size(); k++)
    {
        auto &p = pcd_in->points_[k];
        Eigen::Vector2i ind = PointtoPixel(p);
        int &i = ind(0);
        int &j = ind(1);

        if (abs(p(2)) < d_thres && abs(p(2)) > d_thres_inv && j < right_thres && j > left_thres &&
            i > top_thres && i < bottom_thres && (p - p_plane).dot(n_gravity) < 0)
        {
            out->points_.push_back(p);
            if (pcd_in->HasColors())
                out->colors_.push_back(pcd_in->colors_[k]);
            if (pcd_in->HasNormals())
                out->normals_.push_back(pcd_in->normals_[k]);
        }
    }
    return out;
}

void CameraWrapper::saveImagesToDisk(std::string fullPath, bool saveFRColor)
{
    if (fullPath.empty())
        fullPath = open3d::utility::filesystem::GetWorkingDirectory();
    std::string colorDir = fullPath + "/color/";
    std::string depthDir = fullPath + "/depth/";
    open3d::utility::filesystem::MakeDirectoryHierarchy(colorDir);
    open3d::utility::filesystem::MakeDirectoryHierarchy(depthDir);
    for (int i = 0; i < rgbdImages.size(); i++)
    {
        open3d::io::WriteImageToPNG(colorDir + getStringforNumberedFile(i) + ".png", rgbdImages[i]->color_);
        open3d::geometry::Image &depthFloat = rgbdImages[i]->depth_; // 4 byte float image scaled with 1/1000
        open3d::geometry::Image depth;
        depth.Prepare(depthFloat.width_, depthFloat.height_, 1, 2);
        for (int i = 0; i < depth.height_; i++)
        {
            for (int j = 0; j < depth.width_; j++)
            {
                *depth.PointerAt<uint16_t>(j, i) = *depthFloat.PointerAt<float>(j, i) * 1000.0;
            }
        }
        open3d::io::WriteImageToPNG(depthDir + getStringforNumberedFile(i) + ".png", depth); // note this cannot write float images!
    }
    if (saveFRColor)
    {
        std::string colorFRDir = fullPath + "/colorFR/";
        open3d::utility::filesystem::MakeDirectoryHierarchy(colorFRDir);
        for (int i = 0; i < FRCalibrationImages.size(); i++)
        {
            open3d::io::WriteImageToPNG(colorFRDir + getStringforNumberedFile(i) + ".png", *OpenCVToOpen3D(FRCalibrationImages.at(i)));
        }
    }
}

void CameraWrapper::saveCameraInfoToDisk(std::string fullPath)
{
    if (fullPath.empty())
        fullPath = open3d::utility::filesystem::GetWorkingDirectory();

    Eigen::Matrix4d intrCol = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d intrDepth = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d intrCalib = Eigen::Matrix4d::Identity();
    intrCol.block<3, 3>(0, 0) = ColorLDTIntrinsic.intrinsic_matrix_;
    intrDepth.block<3, 3>(0, 0) = DepthLDTIntrinsic.intrinsic_matrix_;
    intrCalib.block<3, 3>(0, 0) = CalibrationIntrinsic.intrinsic_matrix_;
    saveMatrixToDisc(fullPath, "/ColorLDTIntrinsic.txt", intrCol);
    saveMatrixToDisc(fullPath, "/DepthLDTIntrinsic.txt", intrDepth);
    saveMatrixToDisc(fullPath, "/CalibrationIntrinsic.txt", intrCalib);
}

bool CameraWrapper::ReadFOVDataFromDisk(const string &path)
{
    cout << "Reading field of view data from " << path + SerialNumber + "FOV.txt" << endl;

    ifstream fovdata(path + SerialNumber + "FOV.txt");
    string line;
    vector<float> data;
    if (fovdata.is_open())
    {
        while (getline(fovdata, line))
        {
            data.push_back(stod(line));
        }
        if (data.size() != 8)
        {
            cout << "Warning: fov data is bad.\n";
            return false;
        }
        crop_left = data[0];
        crop_right = data[1];
        crop_top = data[2];
        crop_bottom = data[3];
        crop_depth = data[4];
        crop_depth_inv = data[5];
        crop_ground_height = data[6];
        crop_ground_height_fine = data[7];
    }
    else
    {
        cout << "Warning: Reading fov data from file went wrong\n";
        return false;
    }

    cout << "Reading field of view threshold data from " << path + SerialNumber + "FOVThresholds.txt" << endl;

    ifstream fovdataThreshold(path + SerialNumber + "FOVThresholds.txt");
    vector<float> dataThres;
    if (fovdataThreshold.is_open())
    {
        while (getline(fovdataThreshold, line))
        {
            dataThres.push_back(stod(line));
        }
        if (dataThres.size() != 5)
        {
            cout << "Warning: fov threshold data is bad.\n";
            return false;
        }

        dmax = dataThres[0];
    }
    else
    {
        cout << "Warning: Reading fov threshold data from file went wrong\n";
        return false;
    }
    return true;
}

bool CameraWrapper::ReadGravityVectorFromDisk(const std::string &path)
{
    cout << "Reading gravity vector from " << path + SerialNumber + "Gravity.txt" << endl;

    ifstream file(path + SerialNumber + "Gravity.txt");
    string line;
    vector<float> data;
    if (file.is_open())
    {
        while (getline(file, line))
        {
            data.push_back(stod(line));
        }
        if (data.size() != 3)
        {
            cout << "Warning: Gravity data is bad.\n";
            return false;
        }
        GravityVector(0) = data[0];
        GravityVector(1) = data[1];
        GravityVector(2) = data[2];
    }
    else
    {
        cout << "Warning: Reading gravity vector from file went wrong\n";
        return false;
    }
    return true;
}

bool CameraWrapper::ReadCameraPositionFromDisk(const std::string &path)
{
    cout << "Reading camera position from " << path + SerialNumber + "Position.txt" << endl;
    ifstream file(path + SerialNumber + "Position.txt");
    string line;
    std::string delimiter = " ";
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    if (file.is_open())
    {
        int row = 0;
        while (getline(file, line))
        {
            std::string token;
            size_t pos;
            int col = 0;
            while ((pos = line.find(delimiter)) != std::string::npos)
            {
                token = line.substr(0, pos);
                if (token.compare(""))
                { // this stupid shit returns wrong if strings are equal
                    pose(row, col) = stod(token);
                    line.erase(0, pos + delimiter.length());
                    col++;
                }
                else
                {
                    line.erase(0, 1);
                }
            }
            pose(row, col) = stod(line); // last entry
            row++;
        }
        Extrinsic = pose;
    }
    else
    {
        cout << "Warning: Could not read camera position file \n";
        return false;
    }
    return true;
}

void CameraWrapper::SaveGravityVectorToDisk(const std::string &path)
{
    ofstream file(path + SerialNumber + "Gravity.txt");
    if (file.is_open())
    {
        file << GravityVector(0) << endl;
        file << GravityVector(1) << endl;
        file << GravityVector(2) << endl;
    }
    else
    {
        cout << "Warning: Writing gravity vector to file failed! \n";
        return;
    }
    file.close();
}

void CameraWrapper::SaveCameraPositionToDisk(const std::string &path)
{
    cout << "Saving camera position to " << path + SerialNumber + "Position.txt" << endl;
    ofstream file(path + SerialNumber + "Position.txt");
    if (file.is_open())
    {
        file << Extrinsic << endl;
    }
    else
    {
        cout << "Warning: Writing camera position to file failed! \n";
    }
    file.close();
}

void CameraWrapper::SetMaxDepth(std::shared_ptr<open3d::geometry::PointCloud> pcd)
{
    dmax = 0;
    // o3d coordinate system: x right, y down, z forward
    for (auto &p : pcd->points_)
    {
        if (abs(p(2)) > dmax)
        {
            dmax = abs(p(2));
        }
    }
}

// void CameraWrapper::SaveFOVValuesToDisk(const std::string &path)
// {
//     ofstream file(path + SerialNumber + "FOV.txt");
//     if (file.is_open())
//     {
//         file << crop_left << endl;
//         file << crop_right << endl;
//         file << crop_top << endl;
//         file << crop_bottom << endl;
//         file << crop_depth << endl;
//         file << crop_depth_inv << endl;
//         file << crop_ground_height << endl;
//         file << crop_ground_height_fine << endl;
//     }
//     else
//     {
//         cout << "Writing Crop data to file failed! \n";
//         return;
//     }
//     file.close();
// }

// visualization
void CameraWrapper::showOpen3dImages()
{
    for (auto img : rgbdImages)
    {
        open3d::visualization::DrawGeometries({img});
    }
}

void CameraWrapper::showOpen3DPcds()
{
    for (auto pcd : Pcds)
    {
        open3d::visualization::DrawGeometries({pcd, getOrigin()});
    }
}

void CameraWrapper::printInfo()
{
    cout << "#################################### \n";
    cout << "Printing Camera Information \n";
    cout << "The Camera connection is " << Connected << endl;
    cout << "The camera SerialNumber is " << SerialNumber << endl;
    cout << "The list size of rgbdImages is " << rgbdImages.size() << endl;
    cout << "#################################### \n";
}

// i is row, j is colum. open3d pointer at takes the arguments in reverse
Eigen::Vector3d CameraWrapper::PixeltoPoint(const int &i, const int &j, const float &depth)
{
    return Eigen::Vector3d((j - DepthLDTIntrinsic.intrinsic_matrix_(0, 2)) * depth / DepthLDTIntrinsic.intrinsic_matrix_(0, 0),
                           (i - DepthLDTIntrinsic.intrinsic_matrix_(1, 2)) * depth / DepthLDTIntrinsic.intrinsic_matrix_(1, 1),
                           depth);
}

// i is row, j is colum. open3d pointer_at takes the arguments in reverse
Eigen::Vector2i CameraWrapper::PointtoPixel(const Eigen::Vector3d &p)
{
    return Eigen::Vector2i(std::round(p(1) / p(2) * DepthLDTIntrinsic.intrinsic_matrix_(1, 1) + DepthLDTIntrinsic.intrinsic_matrix_(1, 2)),
                           std::round(p(0) / p(2) * DepthLDTIntrinsic.intrinsic_matrix_(0, 0) + DepthLDTIntrinsic.intrinsic_matrix_(0, 2)));
}

// i is row, j is colum. open3d pointer_at takes the arguments in reverse
Eigen::Vector2d CameraWrapper::PointtoPixelExact(const Eigen::Vector3d &p)
{
    return Eigen::Vector2d(p(1) / p(2) * DepthLDTIntrinsic.intrinsic_matrix_(1, 1) + DepthLDTIntrinsic.intrinsic_matrix_(1, 2),
                           p(0) / p(2) * DepthLDTIntrinsic.intrinsic_matrix_(0, 0) + DepthLDTIntrinsic.intrinsic_matrix_(0, 2));
}

bool CameraWrapper::InsideDepthImage(const Eigen::Vector2d &p)
{
    if (p(0) >= 0 && p(0) < DepthLDTIntrinsic.height_ && p(1) >= 0 && p(1) < DepthLDTIntrinsic.width_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool CameraWrapper::InsideDepthImage(const Eigen::Vector2i &p)
{
    if (p(0) >= 0 && p(0) < DepthLDTIntrinsic.height_ && p(1) >= 0 && p(1) < DepthLDTIntrinsic.width_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

shared_ptr<open3d::geometry::PointCloud> CameraWrapper::pcdFromDepth(const open3d::geometry::Image &img)
{

    auto out = make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < img.height_; i++)
    {
        for (int j = 0; j < img.width_; j++)
        {
            if (!isnan(*img.PointerAt<float>(j, i)) && *img.PointerAt<float>(j, i) != 0)
            {
                out->points_.push_back(PixeltoPoint(i, j, *img.PointerAt<float>(j, i)));
            }
        }
    }
    return out;
}

shared_ptr<open3d::geometry::PointCloud> CameraWrapper::pcdFromDepth(const open3d::geometry::RGBDImage &img)
{

    auto out = make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < img.depth_.height_; i++)
    {
        for (int j = 0; j < img.depth_.width_; j++)
        {
            if (!isnan(*img.depth_.PointerAt<float>(j, i)) && *img.depth_.PointerAt<float>(j, i) != 0)
            {
                out->points_.push_back(PixeltoPoint(i, j, *img.depth_.PointerAt<float>(j, i)));
                Eigen::Vector3d c(*img.color_.PointerAt<uchar>(j, i, 0) / 255.0,
                                  *img.color_.PointerAt<uchar>(j, i, 1) / 255.0,
                                  *img.color_.PointerAt<uchar>(j, i, 2) / 255.0);
                out->colors_.push_back(c);
            }
        }
    }
    return out;
}

// pcd must be correctly transformed
Eigen::MatrixXd CameraWrapper::getDepthBufferFromPCD(open3d::geometry::PointCloud &pcd)
{
    Eigen::MatrixXd depth_buffer = Eigen::MatrixXd::Constant(DepthLDTIntrinsic.height_, DepthLDTIntrinsic.width_, 0);
    for (int i = 0; i < pcd.points_.size(); i++)
    {
        Eigen::Vector2i ind = PointtoPixel(pcd.points_[i]);
        if (InsideDepthImage(ind))
        {
            if (depth_buffer(ind(0), ind(1)) > pcd.points_[i](2) || depth_buffer(ind(0), ind(1)) == 0)
            {
                depth_buffer(ind(0), ind(1)) = pcd.points_[i](2);
            }
        }
    }
    return depth_buffer;
}

// pcd must be correctly transformed
shared_ptr<open3d::geometry::Image> CameraWrapper::getDepthImageFromPCD(open3d::geometry::PointCloud &pcd)
{
    auto out = make_shared<open3d::geometry::Image>();
    out->Prepare(DepthLDTIntrinsic.width_, DepthLDTIntrinsic.height_, 1, 4);
    for (int i = 0; i < pcd.points_.size(); i++)
    {
        Eigen::Vector2i ind = PointtoPixel(pcd.points_[i]);
        if (InsideDepthImage(ind))
        {
            if (*out->PointerAt<float>(ind(1), ind(0)) > pcd.points_[i](2) || *out->PointerAt<float>(ind(1), ind(0)) == 0)
            {
                *out->PointerAt<float>(ind(1), ind(0)) = pcd.points_[i](2);
            }
        }
    }
    return out;
}

Eigen::MatrixXd CameraWrapper::getFOVDepthBuffer(open3d::geometry::TriangleMesh &mesh)
{
    PandiaTimer t;
    Eigen::MatrixXd depth_buffer = Eigen::MatrixXd::Constant(DepthLDTIntrinsic.height_, DepthLDTIntrinsic.width_, 0);

    for (int i = 0; i < mesh.triangles_.size(); i++)
    {

        Eigen::Vector3d v03D = mesh.vertices_[mesh.triangles_[i](0)];
        Eigen::Vector3d v13D = mesh.vertices_[mesh.triangles_[i](1)];
        Eigen::Vector3d v23D = mesh.vertices_[mesh.triangles_[i](2)];

        Eigen::Vector2d v0 = PointtoPixelExact(v03D);
        Eigen::Vector2d v1 = PointtoPixelExact(v13D);
        Eigen::Vector2d v2 = PointtoPixelExact(v23D);

        // now we have the pixel coordinates of the projected vertices
        int xmin = std::min(std::min(v0(0), v1(0)), v2(0));
        int ymin = std::min(std::min(v0(1), v1(1)), v2(1));
        int xmax = std::ceil(std::max(std::max(v0(0), v1(0)), v2(0)));
        int ymax = std::ceil(std::max(std::max(v0(1), v1(1)), v2(1)));
        double area = edgeFunction(v0, v1, v2);

        // abort if boundaries are absurdly large
        if (xmin < -5000 || ymin < -5000 || xmax > 5000 || ymax > 5000)
        {
            cout << "Warning invalid depth buffer detected \n";
            return depth_buffer;
        }
#pragma omp parallel for collapse(2)
        for (int i = xmin; i < xmax; i++)
        {
            for (int j = ymin; j < ymax; j++)
            {
                Eigen::Vector2d p(i + 0.5, j + 0.5);
                if (InsideDepthImage(p))
                {
                    double w0 = edgeFunction(v1, v2, p) / area;
                    double w1 = edgeFunction(v2, v0, p) / area;
                    double w2 = edgeFunction(v0, v1, p) / area;
                    // check if in triangle
                    if (w0 >= 0 && w1 >= 0 && w2 >= 0 && w0 <= 1 && w1 <= 1 && w2 <= 1)
                    {
                        // check if in image plane
                        double dinv = w0 * 1 / v03D(2) + w1 * 1 / v13D(2) + w2 * 1 / v23D(2);
                        double d = 1 / dinv;
                        if (d < depth_buffer(i, j) || depth_buffer(i, j) == 0)
                        {
                            depth_buffer(i, j) = d;
                        }
                    }
                }
            }
        }
    }
    // cout << "Fov buffer taking " << t.seconds() << endl;

    return depth_buffer;
}
// pcd must be correctly transformed
shared_ptr<open3d::geometry::RGBDImage> CameraWrapper::getRGBDImageFromPCD(open3d::geometry::PointCloud &pcd)
{
    auto out = make_shared<open3d::geometry::RGBDImage>();
    if (pcd.colors_.size() == 0)
    {
        cout << "Warning, calling getRGBDImageFromPCD() with pcd without color \n";
        return out;
    }
    out->depth_.Prepare(DepthLDTIntrinsic.width_, DepthLDTIntrinsic.height_, 1, 4);
    out->color_.Prepare(DepthLDTIntrinsic.width_, DepthLDTIntrinsic.height_, 3, 1);
    for (int i = 0; i < pcd.points_.size(); i++)
    {
        Eigen::Vector2i ind = PointtoPixel(pcd.points_[i]);
        if (InsideDepthImage(ind))
        {
            if (*out->depth_.PointerAt<float>(ind(1), ind(0)) > pcd.points_[i](2) || *out->depth_.PointerAt<float>(ind(1), ind(0)) == 0)
            {
                *out->depth_.PointerAt<float>(ind(1), ind(0)) = pcd.points_[i](2);
            }
            *out->color_.PointerAt<uint8_t>(ind(1), ind(0), 0) = (uint8_t)(255 * pcd.colors_[i](0));
            *out->color_.PointerAt<uint8_t>(ind(1), ind(0), 1) = (uint8_t)(255 * pcd.colors_[i](1));
            *out->color_.PointerAt<uint8_t>(ind(1), ind(0), 2) = (uint8_t)(255 * pcd.colors_[i](2));
        }
    }
    return out;

    //                     Eigen::Vector3d c(*img.color_.PointerAt<uchar>(j, i,0)/255.0,
    //                                 *img.color_.PointerAt<uchar>(j, i,1)/255.0,
    //                                 *img.color_.PointerAt<uchar>(j, i,2)/255.0);
    //                 out->colors_.push_back(c);
}

shared_ptr<open3d::geometry::PointCloud> CameraWrapper::EigenDepthToPCD(const Eigen::MatrixXd &depth_buffer)
{
    auto out = make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < depth_buffer.rows(); i++)
    {
        for (int j = 0; j < depth_buffer.cols(); j++)
        {
            if (!isnan(depth_buffer(i, j)) && depth_buffer(i, j) != 0)
            {
                out->points_.push_back(PixeltoPoint(i, j, depth_buffer(i, j)));
            }
        }
    }
    return out;
}
