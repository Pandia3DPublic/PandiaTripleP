//USE THIS BY COPYING IT IN TEST.CPP


#include "cameras/ZividCamera.h"
#include "cameras/PhoxiCamera.h"
#include "cameras/IntelCamera.h"
#include "cameras/KinectCamera.h"
#include "cameras/EnsensoCamera.h"
#include "cameras/SickCamera.h"
#include "utils/vishelper.h"
#include "utils/imageFunctions.h"
#include "utils/conversions.h"

#include <iostream>
#include <thread>

using namespace std;

// i is row, j is colum. open3d pointer at takes the arguments in reverse
Eigen::Vector3d PixeltoPoint(int &i, int &j, const float &depth, const Eigen::Matrix3d &m)
{
	return Eigen::Vector3d((j - m(0, 2)) * depth / m(0, 0),
						   (i - m(1, 2)) * depth / m(1, 1),
						   depth);
}

// i is row, j is colum. open3d pointer_at takes the arguments in reverse
Eigen::Vector2d PointtoPixelExact(const Eigen::Vector3d &p, const Eigen::Matrix3d &m)
{
	return Eigen::Vector2d(p(1) / p(2) * m(1, 1) + m(1, 2),
						   p(0) / p(2) * m(0, 0) + m(0, 2));
}



shared_ptr<open3d::geometry::PointCloud> getOutlierFilteredPcd(shared_ptr<open3d::geometry::PointCloud> largePcd, shared_ptr<open3d::geometry::PointCloud> smallPcd)
{


	auto out = make_shared<open3d::geometry::PointCloud>();
    open3d::geometry::KDTreeFlann largePcdKdtree;
	largePcdKdtree.SetGeometry(*largePcd);


    std::vector<int> indices(1);
    std::vector<double> dists(1);
	double thres = 2e-3; //2mm
	for (auto& p:smallPcd->points_){
    	largePcdKdtree.SearchKNN(p, 1, indices, dists);
		if (dists.front() < thres){
			out->points_.push_back(p);
		}
	}

	return out;
}

bool greaterx(double a, double b){
	return a < b;
}

double getBadMedianPcdDistance(shared_ptr<open3d::geometry::PointCloud> pcd1, shared_ptr<open3d::geometry::PointCloud> pcd2){

    open3d::geometry::KDTreeFlann Pcd1Kdtree;
	Pcd1Kdtree.SetGeometry(*pcd1);

	vector<double> distances;
	double thres = 2e-3; //2mm
	for (auto& p:pcd2->points_){
		std::vector<int> indices(1);
		std::vector<double> dists(1);
    	Pcd1Kdtree.SearchKNN(p, 1, indices, dists);
		distances.push_back(dists.front());
	}

	std::sort(distances.begin(),distances.end());
	// for(auto d: distances){
	// 	cout << d<< endl;
	// }

	return distances[(int)(0.99 * distances.size())];


}

int main_()
{
	// auto kinectSerials = KinectCamera::getAvailableCameras();
	// auto zividSerials = ZividCamera::getAvailableCameras();
	// auto intelSerials = IntelCamera::getAvailableCameras();
	// auto ensensoSerials = EnsensoCamera::getAvailableCameras();
	auto sickSerials = SickCamera::getAvailableCameras();
	{
		auto cam = SickCamera::CreateCameraPointer();
		if (!sickSerials.empty())
		{
			cam->connectBySerialNumber(sickSerials[0]);
			cam->record(1);
		}
	}
	{
		sickSerials = SickCamera::getAvailableCameras();
		auto cam1 = SickCamera();
		cam1.connectBySerialNumber(sickSerials[0]);
		cam1.record(1);
	}


	sickSerials = SickCamera::getAvailableCameras();
	cout << "### sick serial: " << sickSerials[0] << endl;
	auto cam2 = SickCamera::CreateCameraPointer();
	cam2->connectBySerialNumber(sickSerials[0]);
	cam2->record(1);

	// auto cam = KinectCamera::CreateCameraPointer();
	// if (!kinectSerials.empty())
	// {
	// 	cam->connectBySerialNumber(kinectSerials[0]);
	// 	cam->record(1, 30);
	// }
	// auto cam = IntelCamera::CreateCameraPointer();
	// if (!intelSerials.empty())
	// {
	// 	cam->connectBySerialNumber(intelSerials[0]);
	// 	cam->record(1);
	// }

	// auto cam = EnsensoCamera::CreateCameraPointer();
	// if (!ensensoSerials.empty())
	// {
	// 	cam->connectBySerialNumber(ensensoSerials[0]);
	// 	cam->record(1);
	// }




//   auto reproPcd = cam->pcdFromDepth(cam->rgbdImages.front()->depth_);
//   reproPcd->PaintUniformColor(Eigen::Vector3d(0,0,1));
//   auto sdkpcd = cam->Pcds.front();

//   vector<double> dists = reproPcd->ComputePointCloudDistance(*sdkpcd);
//   double sum = 0;
//   int n = dists.size();
//   for (int i=0; i<dists.size();i++){
//     sum+= dists[i];
//   }
//   double mean  = sum/n;
//   double rsme =0;
//   for (int i=0; i<dists.size();i++){
//     rsme+= std::pow(dists[i] -mean,2);
//   }
//   rsme/=n;
//   rsme = std::sqrt(rsme);
//   cout << "The mean error in mm is " <<1e3 * mean << endl;
//   cout << "The root square mean error in mm is " <<1e3* rsme << endl;

//   open3d::visualization::DrawGeometries({cam->rgbdImages.front()});
//   open3d::visualization::DrawGeometries({getOrigin(), cam->Pcds.front()});
//   open3d::visualization::DrawGeometries({getOrigin(), cam->Pcds.front(), reproPcd});




	// // show original sdk genereated pcd and pinhole generated pcd
	// auto img = cam->rgbdImages.front();
	// auto pinholePcd = make_shared<open3d::geometry::PointCloud>();
	// for (int i = 0; i < img->depth_.height_; i++)
	// {
	// 	for (int j = 0; j < img->depth_.width_; j++)
	// 	{
	// 		// pinholePcd->points_.push_back(cam->PixeltoPoint(i, j, *img->depth_.PointerAt<float>(j, i)));
	// 		if (!isnan(*img->depth_.PointerAt<float>(j, i)) && *img->depth_.PointerAt<float>(j, i) != 0)
	// 		{
	// 			pinholePcd->points_.push_back(PixeltoPoint(i, j, *img->depth_.PointerAt<float>(j, i), cam->DepthLDTIntrinsic.intrinsic_matrix_));
	// 		}
	// 	}
	// }

	// pinholePcd->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
	// // open3d::visualization::DrawGeometries({pinholePcd,cam->Pcds.front(), getOrigin()});

	// // show original sdk genereated pcd and pcd generated by loop over pixel to point

	// auto nonlinearPcd = make_shared<open3d::geometry::PointCloud>();
	// for (int i = 0; i < img->depth_.height_; i++)
	// {
	// 	for (int j = 0; j < img->depth_.width_; j++)
	// 	{
	// 		if (!isnan(*img->depth_.PointerAt<float>(j, i)) && *img->depth_.PointerAt<float>(j, i) != 0)
	// 		{
	// 			nonlinearPcd->points_.push_back(cam->PixeltoPoint(i, j, *img->depth_.PointerAt<float>(j, i)));
	// 		}
	// 	}
	// }


	// nonlinearPcd->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
	// double metric = getBadMedianPcdDistance(nonlinearPcd,cam->Pcds.front());
	// // double metric = getBadMedianPcdDistance(pinholePcd,cam->Pcds.front());
	// Eigen::Vector3d d = cam->Pcds.front()->GetCenter() - nonlinearPcd->GetCenter();
	// // cout << 1000 * d << endl;
	// // cout << 1000 * d.norm() << endl;
	// cout << "median metric " << metric << endl;
	// cout << "size original " << cam->Pcds.front()->points_.size() << endl;
	// cout << "size nonlinear filtered" << nonlinearPcd->points_.size() << endl;
	// // open3d::visualization::DrawGeometries({cam->Pcds.front(), nonlinearPcd, getOrigin()});
	// open3d::visualization::DrawGeometries({nonlinearPcd, pinholePcd, getOrigin(), cam->Pcds.front()});

	// // open3d::visualization::DrawGeometries({ nonlinearPcd});

	// // do pixel-to-point and point-to-pixel and check if nothing has changed

	// auto projectedDepth = make_shared<open3d::geometry::RGBDImage>();
	// projectedDepth->color_.Prepare(cam->DepthLDTIntrinsic.width_,
	// 							   cam->DepthLDTIntrinsic.height_, 3, 1);
	// projectedDepth->depth_.Prepare(cam->DepthLDTIntrinsic.width_,
	// 							   cam->DepthLDTIntrinsic.height_, 1, 4);

	// for (int i = 0; i < projectedDepth->depth_.height_; i++)
	// {
	// 	for (int j = 0; j < projectedDepth->depth_.width_; j++)
	// 	{
	// 		*projectedDepth->depth_.PointerAt<float>(j, i) = 0;
	// 	}
	// }

	// int c =0;
	// for (int i = 0; i < nonlinearPcd->points_.size(); i++)
	// {
	// 	Eigen::Vector3d &p = nonlinearPcd->points_[i];
	// 	Eigen::Vector2i ind = cam->PointtoPixel(p);
	// 	// Eigen::Vector2d ind = PointtoPixel(p,cam->DepthLDTIntrinsic.intrinsic_matrix_);
	// 	if (cam->InsideDepthImage(ind))
	// 	{
	// 		*projectedDepth->depth_.PointerAt<float>(std::round(ind(1)), std::round(ind(0))) = p(2);
	// 	}else{
	// 		c++;
	// 		// cout << "reprojected p not in depth image -> weird. \n";
	// 	}
	// }


	// // open3d::visualization::DrawGeometries({nonlinearPcd});
	// open3d::visualization::DrawGeometries({projectedDepth});
	// open3d::visualization::DrawGeometries({cam->rgbdImages.front()});

	// // image test

	// auto distcvimg = Open3DToOpenCV(cam->rgbdImages.front()->depth_).clone();
	// auto undistcvimg = Open3DToOpenCV(cam->rgbdImages.front()->depth_).clone();
	// auto redistcvimg = Open3DToOpenCV(cam->rgbdImages.front()->depth_).clone();
	// cv::remap(distcvimg, undistcvimg, cam->DistortionMap, cv::noArray(), cv::INTER_NEAREST);
	// cv::remap(undistcvimg, redistcvimg, cam->UndistortionMap, cv::noArray(), cv::INTER_NEAREST);
	// cv::imshow("dist", distcvimg);
	// cv::imshow("undistort", undistcvimg);
	// cv::imshow("redistort", redistcvimg);

	// double errcv = cv::norm(distcvimg, redistcvimg, cv::NORM_L2);
	// cout << "err dist redist " << errcv << endl;
	// double errdist = cv::norm(distcvimg, undistcvimg, cv::NORM_L2);
	// cout << "err dist undist " << errdist << endl;

	// double errselfprojected = cv::norm(Open3DToOpenCV(projectedDepth->depth_), Open3DToOpenCV(cam->rgbdImages.front()->depth_), cv::NORM_L2);
	// cout << "errselfprojected " << errselfprojected << endl;
	// cv::waitKey(0);

	// // cam->showOpen3dImages();

	// // todos
	// //  change the variable structure
	// //  check the record of all cameras

	return 0;
}

int main()
{
	int a = 0;
	int nInvalid = 0;
	for (int i = 0; i < 1 + nInvalid; i++)
	{
		cout << i << endl;
		if (i < 10)
		{
			nInvalid++;
			cout << "continiue" << endl;
			continue;
		}
		a++;
	}
	cout << "a " << a << endl;
	return 0;
}