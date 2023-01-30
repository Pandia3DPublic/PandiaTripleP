#pragma once
#include "../cameras/CameraWrapper.h"
#include "CalibrationSingleton.h"

class ScanningSingleton
{
public:
    ScanningSingleton();
    ~ScanningSingleton();

    // former button functions
    bool ComputeCameraPositions();
    bool StartScan();
    bool AddScan();
    bool RevertLastScan();
    bool StartReferenceScan();
    bool AddReferenceScan();
    bool RevertLastReferenceScan();
    bool RefineReferenceScan();
    bool SetReferenceModel(std::shared_ptr<open3d::geometry::TriangleMesh> mesh);
    bool SetReferenceModel(std::shared_ptr<open3d::geometry::PointCloud> pcd); // wip
    std::vector<std::vector<Eigen::Vector3d>> getLatestDistanceColorsGradient();
    // std::vector<Eigen::Vector3d> getDistanceColorsGradient();                  // for currentpcd
    std::shared_ptr<open3d::geometry::PointCloud> getMissingPoints();
    std::shared_ptr<open3d::geometry::PointCloud> getVisibilityPCD();
    std::shared_ptr<open3d::geometry::PointCloud> getCalibrationPCD();
    void clearReferenceModel();
    bool ReferenceModelSet();
    Eigen::Matrix4d getIterativePCABasedAlignment(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target);
Eigen::Matrix4d getIterativePCABasedAlignment(std::shared_ptr<open3d::geometry::PointCloud> source_large, std::shared_ptr<open3d::geometry::PointCloud> target_large, double &alignmentFitness);

    // config vars

    bool RefineAlignWithCAD = true;
    bool UseVoxelGrid = false;

    //################# Current Scan Variables ####################
    std::shared_ptr<open3d::geometry::PointCloud> CurrentPCD; // resulting comined pcd from startScan
    std::shared_ptr<open3d::geometry::PointCloud> CurrentDownsampledPCD; //NOT always up to date
    std::shared_ptr<open3d::pipelines::integration::ScalableTSDFVolume> CurrentVoxelGrid;

    std::shared_ptr<open3d::geometry::PointCloud> OldPCD; 
    std::shared_ptr<open3d::geometry::PointCloud> OldDownsampledPCD; 

    std::vector<Eigen::Matrix4d> T_OCMIs;
    int OCMIScanIteration = 0; //for debug coloring
    double OCMILastAngle = 0; //in degrees
    bool OCMIScanRevertable = false;
    double FinalFitness = 0; //fitness of best icp align of CurrentPCD to reference model
    double OldFinalFitness = 0;
    double ReferenceVoxelResolution = 0.0033333; //todo json
    bool ReferenceScanRevertable = false;

    std::shared_ptr<open3d::geometry::PointCloud> ReferenceScanPCD;
    std::shared_ptr<open3d::geometry::PointCloud> OldReferenceScanPCD; //for last scan revert
    std::vector<std::shared_ptr<CameraWrapper>> Cameras;
    std::shared_ptr<CalibrationSingleton> CalibrationManager;

private:
    // various functions
    void clearCurrentStructures();
    bool ApplyFilterOnCroppedImages();
    void RecolorCurrentPCD(const std::vector<Eigen::Vector3d> &newColors);
    bool RecordAndFilterFrames();
    void PerformRefineAlignWithCAD();

    //########### Reference Model Variables ########################
    std::shared_ptr<open3d::geometry::TriangleMesh> ReferenceMesh;
    std::shared_ptr<open3d::geometry::PointCloud> ReferencePCD;
    std::shared_ptr<open3d::geometry::PointCloud> SmallReferencePCD;
    Eigen::Vector3d SmallReferencePCDCenter = Eigen::Vector3d::Zero();

    std::vector<double> DistancesCurrentToReference; // Distances of points in scanned pcd to transformed reference mesh/pcd
    Eigen::Matrix3d ReferencePCAVectors = Eigen::Matrix3d::Identity(); // column wise

    std::string ResourceFolder = "resources/quality/";

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
