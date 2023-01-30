#pragma once

enum class CameraTypes
{
    Ensenso,
    Intel,
    Kinect,
    Phoxi,
    Zivid,
    Sick
};

class CameraWrapper
{
private:
    /* data */
protected:
    virtual bool disconnect();

public:
    CameraWrapper(/* args */);
    virtual ~CameraWrapper();

    //########## wannabe pure virtual functions ##############
    virtual bool connectBySerialNumber(const std::string &serialNumber);
    virtual bool record(int n, int n_wait = 0);
    // static virtual int getAvailableCameras(); //cannot have static virtual functions. But all derived classes should support this
    // static virtual std::shared_ptr<CameraWrapper> CreateCameraPointer;
    //########## base class functions ##############
    void clearBuffers();
    std::shared_ptr<open3d::geometry::RGBDImage> getCroppedRGBDImage(std::shared_ptr<open3d::geometry::RGBDImage> in);
    std::shared_ptr<open3d::geometry::RGBDImage> getCroppedRGBDImageWithGround(std::shared_ptr<open3d::geometry::RGBDImage> in);
    std::shared_ptr<open3d::geometry::PointCloud> getCroppedPcd(std::shared_ptr<open3d::geometry::PointCloud> pcd_in);
    // disk IO
    void saveImagesToDisk(std::string fullPath = "", bool saveFRColor = false);
    void saveCameraInfoToDisk(std::string fullPath = "");
    bool ReadFOVDataFromDisk(const std::string &path);
    bool ReadGravityVectorFromDisk(const std::string &path);
    bool ReadCameraPositionFromDisk(const std::string &path);
    void SaveGravityVectorToDisk(const std::string &path);
    void SaveCameraPositionToDisk(const std::string &path);
    void SetMaxDepth(std::shared_ptr<open3d::geometry::PointCloud> pcd);
    void SaveFOVValuesToDisk(const std::string &path);
    // visualization
    void showOpen3dImages();
    void showOpen3DPcds();
    void printInfo();

    bool InsideDepthImage(const Eigen::Vector2d &p);
    bool InsideDepthImage(const Eigen::Vector2i &p);

    //############ general member variables ############
    CameraTypes CameraType;
    // image data
    // the -ith entry refers to the i-1-th ocmi scan
    std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> rgbdImages;   // always distorted
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Pcds;        // for sdk generated pcds
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> CroppedPcds; //
    std::vector<double> FrameTimestamps; //used for combining multiple frames to one

    // intrinsic data
    std::vector<cv::Mat> FRCalibrationImages; // used in Calibration Singleton
    open3d::camera::PinholeCameraIntrinsic DepthLDTIntrinsic;
    open3d::camera::PinholeCameraIntrinsic ColorLDTIntrinsic;
    open3d::camera::PinholeCameraIntrinsic CalibrationIntrinsic;
    // misc
    bool Connected = false;
    std::string SerialNumber = "undefined";
    Eigen::Matrix4d Extrinsic = Eigen::Matrix4d::Identity();
    // the i-th entry refers to the i-th pcd of the i-1-th omci scan
    std::vector<Eigen::Matrix4d> T_camToRef;
    std::vector<Eigen::Matrix4d> T_AddedToRef;
    Eigen::Vector3d GravityVector = Eigen::Vector3d(0, 0, -1);
    double CameraHeight = 0;
    std::vector<std::vector<cv::Point2f>> CornersListofLists; // for chessboard corners

    // cropping parameters
    float crop_top = 0;
    float crop_bottom = 0;
    float crop_left = 0;
    float crop_right = 0;
    float crop_depth = 0;
    float crop_depth_inv = 0;
    float crop_ground_height = 1e6; // todo taking large height to avoid cropping
    float crop_ground_height_fine = 0.0;
    // thresholds for cropping
    float dmax = 5;
    //accuracy thresholds
    double ColorThresholdMin = 0.005;
    double ColorThresholdMax = 0.005;

    //a variable that gives a rough estimate on how large a voxel one should expect one point in is (edge length in m)
    //calcualted in the connect method of each inherited class for 1m distance.
    double ApproxVoxelSize = 0;

    //############# depth to 3d and reverse

    Eigen::Vector3d PixeltoPoint(const int &i, const int &j, const float &depth);
    Eigen::Vector2i PointtoPixel(const Eigen::Vector3d &p);
    Eigen::Vector2d PointtoPixelExact(const Eigen::Vector3d &p);
    Eigen::MatrixXd getDepthBufferFromPCD(open3d::geometry::PointCloud &pcd);
    Eigen::MatrixXd getFOVDepthBuffer(open3d::geometry::TriangleMesh &mesh);
    std::shared_ptr<open3d::geometry::Image> getDepthImageFromPCD(open3d::geometry::PointCloud &pcd);
    std::shared_ptr<open3d::geometry::PointCloud> pcdFromDepth(const open3d::geometry::Image &img);
    std::shared_ptr<open3d::geometry::PointCloud> pcdFromDepth(const open3d::geometry::RGBDImage &img);
    std::shared_ptr<open3d::geometry::RGBDImage> getRGBDImageFromPCD(open3d::geometry::PointCloud &pcd);
    std::shared_ptr<open3d::geometry::PointCloud> EigenDepthToPCD(const Eigen::MatrixXd &depth_buffer);

    std::vector<float> BrownianCalibrationCamDistortionCoeffs; // todo not ready see github issue

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
