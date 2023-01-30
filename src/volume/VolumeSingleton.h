#pragma once
#include "../cameras/CameraWrapper.h"
#include "../utils/Plane.h"

class VolumeSingleton
{
private:
    /* data */
public:
    VolumeSingleton(/* args */);
    ~VolumeSingleton();

    // bool ReadAlignmentDataFromDisk(const std::string &path);
    // void SaveAlignmentData(const std::string &path);
    double getCurrentVolume();
    bool ProcessCenterButton();
    bool ProcessICPButton();
    void setSiloModel(std::shared_ptr<open3d::geometry::TriangleMesh> mesh);
    void setSiloModel(std::shared_ptr<open3d::geometry::PointCloud> pcd);
    bool setSiloToFirstCamTransform(); //from slider and offset values
    bool SiloModelSet();
    void clearSiloModel();

    double x_offset = 0;
    double y_offset = 0;
    double z_offset = 0;
    double alpha = 0;
    double beta = 0;
    double gamma = 0;
    // The camera noise we assume for volume calculation so that walls dont get measured
    double VolumeNoise = -0.007;
    double DensityFactor = 1;
    double MaxProductHeight = 0.1;
    double CurrentVolumeMaxPointHeight = 0; //set in getCurrentVolume
    double CurrentVolumeAvgPointHeight = 0; //set in getCurrentVolume
    // volume measurement specific parameters
    Eigen::Matrix4d SiloToFirstCamTransform = Eigen::Matrix4d::Identity();
    // gets added to the slider values, in order to prevent slider range problems
    Eigen::Vector3d TranslationOffset = Eigen::Vector3d::Zero(); // from silo to recorded
    Eigen::Vector3d SiloCenter = Eigen::Vector3d::Zero();

    std::shared_ptr<open3d::geometry::PointCloud> VolumePcd;  // not used yet
    std::shared_ptr<open3d::geometry::PointCloud> CroppedPcd; // actually used for volume calculation
    std::atomic<bool> VolumePcdChanged{false};
    cv::Mat VolumeImage; // image for display on the website, with optional overlay
    bool CreateOverlayinImage = true;
    std::vector<std::shared_ptr<CameraWrapper>> Cameras;

    //#### Conveyor Belt #####
    double BeltSpeed = 0;
    std::shared_ptr<open3d::geometry::PointCloud> FirstDirectionPcd;
    std::shared_ptr<open3d::geometry::PointCloud> SecondDirectionPcd;
    Eigen::Vector3d DirectionVector = Eigen::Vector3d::Zero();
    void CalculateDirectionVector();
    double getVolumeIncrement(double dt);
    Eigen::Vector3d BasePlanePoint = Eigen::Vector3d::Zero(); // for database saving
    PandiaPlane BasePlane = PandiaPlane(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d::Zero());
    void setBasePlane();
    cv::Mat BeltAreaImage;
    void clearBeltAreaImage();

    //########################

private:
    std::shared_ptr<open3d::geometry::TriangleMesh> SiloModel;  // mesh of the cad model of the silo, always centered
    std::shared_ptr<open3d::geometry::PointCloud> SiloModelPCD; // pcd of the cad model of the silo, always centered
    Eigen::MatrixXd FOVDepthBuffer; //virtual depth image of reference model from camera view
    std::string ResourceFolder = "resources/volume/";
};
