#include "qualityUtil.h"
#include "open3d/pipelines/registration/GlobalOptimizationConvergenceCriteria.h"
#include "open3d/pipelines/registration/GlobalOptimization.h"
#include "open3d/pipelines/registration/GlobalOptimizationMethod.h"
#include "utils/basicMath.h"
#include "utils/conversions.h"
#include "utils/miscFunctions.h"
#include "utils/3DFunctions.h"
#include "utils/vishelper.h"
#include "utils/fileIO.h"
#include "utils/jsonUtil.h"
#include <Eigen/Eigenvalues>

using namespace std;

cv::Matx33f CalibrationIntrToOpenCVMat(CameraWrapper &cam)
{
    cv::Matx33f camera_matrix = cv::Matx33f::eye();
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            camera_matrix(i, j) = cam.CalibrationIntrinsic.intrinsic_matrix_(i, j);
        }
    }
    return camera_matrix;
}

Eigen::Matrix4d OpencvTransformationToEigen(cv::Matx33d R, cv::Vec3d t)
{
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out(0, 0) = R(0, 0);
    out(0, 1) = R(0, 1);
    out(0, 2) = R(0, 2);
    out(1, 0) = R(1, 0);
    out(1, 1) = R(1, 1);
    out(1, 2) = R(1, 2);
    out(2, 0) = R(2, 0);
    out(2, 1) = R(2, 1);
    out(2, 2) = R(2, 2);
    out(0, 3) = t(0);
    out(1, 3) = t(1);
    out(2, 3) = t(2);
    return out;
}

// for this variant to work the pcds must already be centered
Eigen::Matrix4d refineRegistrationICPCenteredPCDs(std::shared_ptr<open3d::geometry::PointCloud> source,
                                                  std::shared_ptr<open3d::geometry::PointCloud> target, double &fitness,
                                                  const Eigen::Matrix4d &init, int maxIterations, double maxCorsDistance)
{
    if (!source->HasNormals())
        source->EstimateNormals();
    if (!target->HasNormals())
        target->EstimateNormals();
    // if (!source->HasColors())
    //     cout << "warning source pcd has no colors!" << endl;
    // if (!target->HasColors())
    //     cout << "warning target pcd has no colors!" << endl;

    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    crit.max_iteration_ = maxIterations;
    // colored does not work at all for some reason
    auto result = open3d::pipelines::registration::RegistrationICP(*source, *target, maxCorsDistance, init, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
    fitness = result.fitness_;
    // cout << "rsme " <<  result.inlier_rmse_ << endl;
    if (isnan(fitness))
    {
        cout << "Warning: Goodness in icp is nan \n";
    }
    return result.transformation_;
}

// todo eigen vector stuff
std::vector<Eigen::Vector2i> getEdgePixels(std::shared_ptr<open3d::geometry::RGBDImage> img)
{

    auto &depth = img->depth_;
    vector<Eigen::Vector2i> out;
    float thres = 1e-3;

    for (int i = 1; i < depth.height_ - 1; i++)
    {
        for (int j = 1; j < depth.width_ - 1; j++)
        {
            int count = 0;
            if (*depth.PointerAt<float>(j, i) != 0)
            {
                if (*depth.PointerAt<float>(j + 1, i) < thres)
                    count++;
                if (*depth.PointerAt<float>(j, i + 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j + 1, i + 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j - 1, i) < thres)
                    count++;
                if (*depth.PointerAt<float>(j, i - 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j - 1, i - 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j + 1, i - 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j - 1, i + 1) < thres)
                    count++;
            }
            if (count >= 1)
                out.push_back(Eigen::Vector2i(j, i));
        }
    }
    return out;
}

std::vector<Eigen::Vector2i> getBackgroundEdgePixels(std::shared_ptr<open3d::geometry::RGBDImage> img, std::vector<Eigen::Vector2i> &edges)
{
    auto &depth = img->depth_;
    auto cvDepth = Open3DToOpenCV(img->depth_).clone();
    vector<int> innerToOuterEdge;
    // 1. Make binary image where only the edge is non zero
    cv::Mat binary(cvDepth.size(), CV_8U);
    binary = 255; // all filled except the edge
    for (auto &edge : edges)
    {
        binary.at<u_int8_t>(edge(1), edge(0)) = 0;
    }
    cv::Mat dist;

    cv::distanceTransform(binary, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::Mat ring;
    cv::inRange(dist, 9.5, 10.5, ring);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(ring, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    vector<Eigen::Vector2i> out;
    for (int i = 0; i < contours[0].size(); i++)
    {
        out.push_back(Eigen::Vector2i(contours[0][i].x, contours[0][i].y));
    }

    return out;
}

void applyEdgeColorPersistenceFilter(std::shared_ptr<open3d::geometry::RGBDImage> rgbd)
{

    float edgeSpacing = 10;
    float colorThreshold = 150;
    int n_iterations = 6;
    auto edges = getEdgePixels(rgbd);
    auto bgEdges = getBackgroundEdgePixels(rgbd, edges);
    auto edgeMap = getInnerToOuterEdgeMapping(edges, bgEdges);

    int counter = 1e6;
    int firstIterationCounter = 0;
    int iteration = 0;
    while (counter > firstIterationCounter / 10 && iteration < n_iterations)
    {
        counter = 0;
        for (int i = 0; i < edges.size(); i++)
        {
            float r = *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 0);
            float g = *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 1);
            float b = *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 2);

            float r2 = *rgbd->color_.PointerAt<u_int8_t>(bgEdges[edgeMap[i]](0), bgEdges[edgeMap[i]](1), 0);
            float g2 = *rgbd->color_.PointerAt<u_int8_t>(bgEdges[edgeMap[i]](0), bgEdges[edgeMap[i]](1), 1);
            float b2 = *rgbd->color_.PointerAt<u_int8_t>(bgEdges[edgeMap[i]](0), bgEdges[edgeMap[i]](1), 2);

            double d = std::sqrt((r - r2) * (r - r2) + (g - g2) * (g - g2) + (b - b2) * (b - b2));
            if (d < colorThreshold)
            {
                // *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 2) = 255;
                // *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 0) = 0;
                // *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 1) = 0;
                *rgbd->depth_.PointerAt<float>(edges[i](0), edges[i](1)) = 0;
                counter++;
            }
        }
        // cout << "Removed " << counter << " pixel with persistence filter \n";
        edges = getEdgePixels(rgbd);
        edgeMap = getInnerToOuterEdgeMapping(edges, bgEdges);
        if (iteration == 0)
            firstIterationCounter = counter;
        iteration++;
    }
}

// the columns of the matrix are the pca vectors
Eigen::Matrix3d getPCAVectors(std::shared_ptr<open3d::geometry::PointCloud> pcd)
{
    Eigen::MatrixXd pointMatrix(pcd->points_.size(), 3);
    for (int i = 0; i < pcd->points_.size(); i++)
    {
        pointMatrix.block<1, 3>(i, 0) = pcd->points_[i];
    }
    //################# center the pcds #######################
    pointMatrix = pointMatrix.rowwise() - pointMatrix.colwise().mean();
    //#################### perform actual pca ###################
    Eigen::MatrixXd cov = pointMatrix.adjoint() * pointMatrix;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);

    Eigen::Matrix3d pcaVectors = solver.eigenvectors();
    // check if pca vector are right handed, otherwise for them too
    Eigen::Vector3d zCross = pcaVectors.block<3, 1>(0, 0).cross(pcaVectors.block<3, 1>(0, 1));
    if (zCross.dot(pcaVectors.block<3, 1>(0, 2)) < 0)
    {
        // cout << "switching source axis handedness \n";
        pcaVectors.block<3, 1>(0, 2) = -pcaVectors.block<3, 1>(0, 2);
    }
    return pcaVectors;
}

// the rotation returned by this is valid for pcds that are transformed to their center zero point
Eigen::Matrix4d getPCABasedRotation(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target)
{
    Eigen::Matrix3d sourcePCAVectors = getPCAVectors(source);
    Eigen::Matrix3d targetPCAVectors = getPCAVectors(target);

    Eigen::Matrix3d R0 = sourcePCAVectors.inverse();
    Eigen::Matrix3d R = targetPCAVectors * R0;
    Eigen::Matrix4d R4 = Eigen::Matrix4d::Identity();
    R4.block<3, 3>(0, 0) = R;
    return R4;
}


// all angles in degrees
Eigen::Matrix4d getIterativeAlignmentforOCMI(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target, const Eigen::Vector3d &rotationAxis, double &lastAngle)
{
    // find center
    Eigen::Vector3d centerSource = source->GetCenter();
    Eigen::Vector3d centerTarget = target->GetCenter();
    // translate both pcds to center
    auto source_c = std::make_shared<open3d::geometry::PointCloud>(*source);
    auto target_c = std::make_shared<open3d::geometry::PointCloud>(*target);
    source_c->Translate(-centerSource);
    target_c->Translate(-centerTarget);

    bool bidirectional = true; // if true checks both clockwise and counterclockwise rotation, if false assumes clockwise movement of scanned object
    double angle_max = 120;
    double angle_step = 10;
    double thres_fitness = 0.85;
    double thres_icp_inlier = 0.035;
    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    crit.max_iteration_ = 60;

    auto json = readJsonFromDisk("functionConfigs/IterativeAlignment.json");
    setValueFromJson(json, "bidirectional", bidirectional);
    setValueFromJson(json, "angle_max", angle_max);
    setValueFromJson(json, "angle_step", angle_step);
    setValueFromJson(json, "thres_fitness", thres_fitness);
    setValueFromJson(json, "thres_icp_inlier", thres_icp_inlier);
    setValueFromJson(json, "icp_max_iterations", crit.max_iteration_);

    // open3d::pipelines::registration::TransformationEstimationPointToPlane()

    double fitness = 0;
    double bestFitness = 0;
    double bestAngle = 0;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d bestT = Eigen::Matrix4d::Identity();

    cout << "starting at angle " << lastAngle << endl;
    for (double angle = 0; angle < angle_max; angle += angle_step)
    {
        int angleIter = 0;
        int angleSign = -1;
        if (angle == 0 || !bidirectional)
        {
            angleIter = 1;
        }
        while (angleIter < 2)
        {
            angleIter++;
            angleSign = -angleSign;
            double newAngle = angleSign * angle + lastAngle;
            Eigen::Matrix4d R = getRotationMatrixFromVectorAndAngle(rotationAxis, toRadians(-newAngle)); // for alignment must rotate against actual pysical rotation

            open3d::pipelines::registration::RegistrationResult result;
            result = open3d::pipelines::registration::RegistrationICP(*source_c, *target_c, thres_icp_inlier, R, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
            T = result.transformation_;
            fitness = result.fitness_;

            cout << "iterative ocmi fitness: " << fitness << ", angle: " << newAngle << endl;
            // auto sourceTrans = make_shared<open3d::geometry::PointCloud>(*source_c);
            // sourceTrans->Transform(T);
            // sourceTrans->PaintUniformColor(Eigen::Vector3d(1,0,0));
            // open3d::visualization::DrawGeometries({getOrigin(),sourceTrans,target_c});

            if (fitness > thres_fitness)
            {
                lastAngle = newAngle;
                return getTrans(centerTarget) * T * getTrans(-centerSource);
            }
            if (fitness > bestFitness)
            {
                bestAngle = newAngle;
                bestT = T;
                bestFitness = fitness;
            }
        }
    }

    lastAngle = bestAngle;
    // return best matrix here in case non was really good
    return getTrans(centerTarget) * bestT * getTrans(-centerSource);
}
