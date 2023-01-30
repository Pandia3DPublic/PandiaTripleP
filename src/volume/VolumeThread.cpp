// #include "VolumeThread.h"
// #include "GlobalDefines.h"
// #include "util.h"
// #include "GUI/guiUtil.h"
// #include "frustum.h"
// #include "configVariables.h"

// void VolumeThreadFunc(VolumeGUI *vg)
// {
//     using namespace std::chrono_literals;
//     using namespace std;
//     int count = 0;
//     while (!g_StopVolumeThread)
//     {
//         //record pcd
//         auto &cam = vg->Cameras->front();
//         Eigen::Matrix3d &intr = cam.DepthLDTIntrinsic.intrinsic_matrix_;
//         cam.clearBuffers();
//         if (!cam.record(1, 0))
//         {
//             cout << "Error: Recording images failed! Aborting" << endl;
//             return;
//         }
//         cam.AlignUndistortandConvertO3D();
//         cam.CropRGBDImages();

//         //for both pcds build z-buffers. Keep a pixel only if the recorded value is some minimum value smaller
//         //than the reference value.
//         auto combinedPcd = make_shared<open3d::geometry::PointCloud>();
//         cam.T_camToRef = vg->VolumeManager->T_FirstCamToCAD; //used in getFOVDepthBuffer
//         Eigen::MatrixXd referenceDepth = vg->ScanningManager->getFOVDepthBuffer(cam);
//         auto recordedDepth = cam.rgbdImages.front()->depth_;
//         auto rcolor = cam.rgbdImages.front()->color_;

//         // auto filteredRecordedDepth = make_shared<open3d::geometry::Image>();
//         // filteredRecordedDepth->Prepare(recordedDepth.width_, recordedDepth.height_, 1, 4);
//         // auto filteredReferenceDepth = make_shared<open3d::geometry::Image>();
//         // filteredReferenceDepth->Prepare(recordedDepth.width_, recordedDepth.height_, 1, 4);

//         //Possbile improvements:
//         //1. adjust for wallnoise volume loses, since data gets cut
//         //2. try to filter small extra clusters
//         //3. If there are large parts with no data, we neglect those parts in volume calculation, try 2d convex hull (optional)
//         double volume = 0;

//         for (int i = 0; i < referenceDepth.rows(); i++)
//         {
//             for (int j = 0; j < referenceDepth.cols(); j++)
//             {

//                 double d = *recordedDepth.PointerAt<float>(j, i) - referenceDepth(i, j);
//                 if (d<VolumeNoise && * recordedDepth.PointerAt<float>(j, i)> 1e-3 && referenceDepth(i, j) > 1e-3)
//                 {
//                     //just for vis
//                     combinedPcd->points_.push_back(PixeltoPoint(i, j, referenceDepth(i, j), cam.DepthLDTIntrinsic.intrinsic_matrix_));
//                     combinedPcd->points_.push_back(PixeltoPoint(i, j, *recordedDepth.PointerAt<float>(j, i), cam.DepthLDTIntrinsic.intrinsic_matrix_));
//                     combinedPcd->colors_.push_back(Eigen::Vector3d(0.5, 0.5, 0.5));
//                     Eigen::Vector3d c((double)*rcolor.PointerAt<uchar>(j, i, 0), (double)*rcolor.PointerAt<uchar>(j, i, 1), (double)*rcolor.PointerAt<uchar>(j, i, 2));
//                     c = c / 255.0;
//                     combinedPcd->colors_.push_back(c);

//                     //the actual images
//                     // *filteredRecordedDepth->PointerAt<float>(j, i) = *recordedDepth.PointerAt<float>(j, i);
//                     // *filteredReferenceDepth->PointerAt<float>(j, i) = referenceDepth(i, j);
                    
//                     //construct frustum per image, near plane is recorded, far plane is reference
//                     //i is for height, j is for width
//                     Frustum f(std::abs(d), *recordedDepth.PointerAt<float>(j, i) / intr(1, 1),
//                               *recordedDepth.PointerAt<float>(j, i) / intr(0, 0),
//                               referenceDepth(i, j) / intr(1, 1),
//                               referenceDepth(i, j) / intr(0, 0));
//                     volume += f.getVolume();
//                 }
//             }
//         }

//         // auto refpcd = open3d::geometry::PointCloud::CreateFromDepthImage(*filteredReferenceDepth, cam.DepthLDTIntrinsic);
//         // auto recpcd = open3d::geometry::PointCloud::CreateFromDepthImage(*filteredRecordedDepth, cam.DepthLDTIntrinsic);
//         // open3d::visualization::DrawGeometries({refpcd,recpcd,getOrigin()});


//         g_VolumeLock.lock();
//         *vg->VolumeManager->VolumePcd = *combinedPcd;
//         g_VolumeLock.unlock();
//         vg->VolumeManager->VolumePcdChanged = true;
//         // open3d::visualization::DrawGeometries({getOrigin(), vg->VolumeManager->VolumePcd});
        
//         string label = "volume : " + to_string(1000*volume) + " l";
//         dynamic_cast<OgreBites::Label *>(vg->TrayManagers[ACTIVE]->getWidget("lb_volume"))->setCaption(label);
//         std::this_thread::sleep_for(1s);
//         count++;
//     }
// }