#include "../cameras/CameraWrapper.h"
#include "../cameras/IntelCamera.h"
#include "../cameras/KinectCamera.h"
#include "../cameras/EnsensoCamera.h"
#include "../cameras/ZividCamera.h"
#include "../cameras/SickCamera.h"
#include "../cameras/PhoxiCamera.h"
#include "../volume/VolumeSingleton.h"
#include "../quality/ScanningSingleton.h"
#include "../quality/CalibrationSingleton.h"
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace std;

PYBIND11_MODULE(cppBindings, m)
{
    py::class_<CameraWrapper, std::shared_ptr<CameraWrapper>>(m, "CameraWrapper")
        .def(py::init<>())
        // ###### virtual functions ######
        .def("connectBySerialNumber", &CameraWrapper::connectBySerialNumber, py::arg("serialNumber"))
        .def("record", &CameraWrapper::record, "n"_a, "n_wait"_a = 0)
        // .def("disconnect", &CameraWrapper::disconnect)
        // ###### base class functions ######
        .def("clearBuffers", &CameraWrapper::clearBuffers)
        .def("getCroppedRGBDImage", &CameraWrapper::getCroppedRGBDImage, py::arg("in"))
        .def("getCroppedPcd", &CameraWrapper::getCroppedPcd, py::arg("pcd_in"))
        .def("SetMaxDepth", &CameraWrapper::SetMaxDepth, py::arg("pcd"))
        // visualization
        .def("showImages", &CameraWrapper::showOpen3dImages)
        .def("printInfo", &CameraWrapper::printInfo)
        // variables
        .def_readwrite("Connected", &CameraWrapper::Connected)
        .def_readwrite("FRCalibrationImages", &CameraWrapper::FRCalibrationImages)
        .def_readwrite("SerialNumber", &CameraWrapper::SerialNumber)
        .def_readwrite("Pcds", &CameraWrapper::Pcds)
        .def_readwrite("CroppedPcds", &CameraWrapper::CroppedPcds)
        // cropping vars
        .def_readwrite("crop_top", &CameraWrapper::crop_top)
        .def_readwrite("crop_bottom", &CameraWrapper::crop_bottom)
        .def_readwrite("crop_left", &CameraWrapper::crop_left)
        .def_readwrite("crop_right", &CameraWrapper::crop_right)
        .def_readwrite("crop_depth", &CameraWrapper::crop_depth)
        .def_readwrite("crop_depth_inv", &CameraWrapper::crop_depth_inv)
        .def_readwrite("crop_ground_height", &CameraWrapper::crop_ground_height)
        .def_readwrite("crop_ground_height_fine", &CameraWrapper::crop_ground_height_fine)
        // thresholds for cropping
        .def_readwrite("dmax", &CameraWrapper::dmax)
        // pose
        .def_readwrite("GravityVector", &CameraWrapper::GravityVector)
        .def_readwrite("Extrinsic", &CameraWrapper::Extrinsic)
        .def_readwrite("T_camToRef", &CameraWrapper::T_camToRef)
        //varoius 
        .def_readwrite("ColorThresholdMin", &CameraWrapper::ColorThresholdMin)
        .def_readwrite("ColorThresholdMax", &CameraWrapper::ColorThresholdMax)
        .def_readwrite("ApproxVoxelSize", &CameraWrapper::ApproxVoxelSize)
        ;

    py::class_<IntelCamera, std::shared_ptr<IntelCamera>, CameraWrapper>(m, "IntelCamera")
        .def(py::init<>())
        .def("getAvailableCameras", &IntelCamera::getAvailableCameras, "get the serial numbers of available devices")
        .def("recordWithoutChecks", &IntelCamera::recordWithoutChecks, "n"_a, "n_wait"_a = 0)
        .def("startStreaming", &IntelCamera::startStreaming, "n_warmup"_a)
        .def("stopStreaming", &IntelCamera::stopStreaming)
        .def("resetConnection", &IntelCamera::resetConnection, "streamAfterReset"_a = true)
        .def("connectBySerialNumber", &IntelCamera::connectBySerialNumber, py::arg("serialNumber"))
        .def("CreateCameraPointer", &IntelCamera::CreateCameraPointer);

    py::class_<KinectCamera, std::shared_ptr<KinectCamera>, CameraWrapper>(m, "KinectCamera")
        .def(py::init<>())
        .def("getAvailableCameras", &KinectCamera::getAvailableCameras, "get the serial numbers of available devices")
        .def("connectBySerialNumber", &KinectCamera::connectBySerialNumber, py::arg("serialNumber"))
        .def("CreateCameraPointer", &KinectCamera::CreateCameraPointer);

    py::class_<EnsensoCamera, std::shared_ptr<EnsensoCamera>, CameraWrapper>(m, "EnsensoCamera")
        .def(py::init<>())
        .def("getAvailableCameras", &EnsensoCamera::getAvailableCameras, "get the serial numbers of available devices")
        .def("connectBySerialNumber", &EnsensoCamera::connectBySerialNumber, py::arg("serialNumber"))
        .def("combineRecordedFrames", &EnsensoCamera::combineRecordedFrames, py::arg("beltDirection"), py::arg("beltSpeed"))
        .def("CreateCameraPointer", &EnsensoCamera::CreateCameraPointer);

    py::class_<ZividCamera, std::shared_ptr<ZividCamera>, CameraWrapper>(m, "ZividCamera")
    .def(py::init<>())
    .def("getAvailableCameras", &ZividCamera::getAvailableCameras, "get the serial numbers of available devices")
    .def("connectBySerialNumber", &ZividCamera::connectBySerialNumber, py::arg("serialNumber"))
    .def("CreateCameraPointer", &ZividCamera::CreateCameraPointer);

    py::class_<SickCamera, std::shared_ptr<SickCamera>, CameraWrapper>(m, "SickCamera")
    .def(py::init<>())
    .def("getAvailableCameras", &SickCamera::getAvailableCameras, "get the serial numbers of available devices")
    .def("connectBySerialNumber", &SickCamera::connectBySerialNumber, py::arg("serialNumber"))
    .def("CreateCameraPointer", &SickCamera::CreateCameraPointer);

    py::class_<PhoxiCamera, std::shared_ptr<PhoxiCamera>, CameraWrapper>(m, "PhoxiCamera")
    .def(py::init<>())
    .def("getAvailableCameras", &PhoxiCamera::getAvailableCameras, "get the serial numbers of available devices")
    .def("connectBySerialNumber", &PhoxiCamera::connectBySerialNumber, py::arg("serialNumber"))
    .def("CreateCameraPointer", &PhoxiCamera::CreateCameraPointer);

    py::class_<cv::Mat>(m, "cvMatrix", py::buffer_protocol())
        .def_buffer([](cv::Mat &m) -> py::buffer_info
                    {
                        string formatdescriptor;
                        if (m.elemSize1() == 2){
                            formatdescriptor = py::format_descriptor<short>::format();
                        } else if(m.elemSize1() == 1){
                            formatdescriptor = py::format_descriptor<uchar>::format();
                        }
                        return py::buffer_info(
                            m.data,                         /* Pointer to buffer */
                            m.elemSize1(),                  /* Size of one scalar */
                            formatdescriptor,               /* Python struct-style format descriptor */
                            3,                              /* Number of dimensions */
                            {m.rows, m.cols, m.channels()}, /* Buffer dimensions */
                            {m.elemSize1() * m.channels() * m.cols,
                             m.elemSize1() * m.channels(), /* Strides (in bytes) for each index */
                             m.elemSize1()}); });
    py::class_<VolumeSingleton>(m, "VolumeSingleton", py::dynamic_attr())
        .def(py::init<>())
        .def("getCurrentVolume", &VolumeSingleton::getCurrentVolume)
        .def("setSiloModel", py::overload_cast<std::shared_ptr<open3d::geometry::TriangleMesh>>(&VolumeSingleton::setSiloModel))
        .def("setSiloModel", py::overload_cast<std::shared_ptr<open3d::geometry::PointCloud>>(&VolumeSingleton::setSiloModel))
        .def("SiloModelSet", &VolumeSingleton::SiloModelSet)
        .def("clearSiloModel", &VolumeSingleton::clearSiloModel)
        .def("ProcessCenterButton", &VolumeSingleton::ProcessCenterButton)
        .def("setSiloToFirstCamTransform", &VolumeSingleton::setSiloToFirstCamTransform)
        .def("ProcessICPButton", &VolumeSingleton::ProcessICPButton)
        .def("CalculateDirectionVector", &VolumeSingleton::CalculateDirectionVector)
        .def("setBasePlane", &VolumeSingleton::setBasePlane)
        .def("getVolumeIncrement", &VolumeSingleton::getVolumeIncrement,"dt"_a)
        .def("clearBeltAreaImage", &VolumeSingleton::clearBeltAreaImage)
        .def_readwrite("CreateOverlayinImage", &VolumeSingleton::CreateOverlayinImage)
        .def_readwrite("TranslationOffset", &VolumeSingleton::TranslationOffset)
        .def_readwrite("SiloToFirstCamTransform", &VolumeSingleton::SiloToFirstCamTransform)
        .def_readwrite("Cameras", &VolumeSingleton::Cameras)
        .def_readwrite("x_offset", &VolumeSingleton::x_offset)
        .def_readwrite("y_offset", &VolumeSingleton::y_offset)
        .def_readwrite("z_offset", &VolumeSingleton::z_offset)
        .def_readwrite("alpha", &VolumeSingleton::alpha)
        .def_readwrite("beta", &VolumeSingleton::beta)
        .def_readwrite("gamma", &VolumeSingleton::gamma)
        .def_readwrite("VolumeNoise", &VolumeSingleton::VolumeNoise)
        .def_readwrite("DensityFactor", &VolumeSingleton::DensityFactor)
        .def_readwrite("BeltSpeed", &VolumeSingleton::BeltSpeed)
        .def_readwrite("FirstDirectionPcd", &VolumeSingleton::FirstDirectionPcd)
        .def_readwrite("SecondDirectionPcd", &VolumeSingleton::SecondDirectionPcd)
        .def_readwrite("DirectionVector", &VolumeSingleton::DirectionVector)
        .def_readwrite("BasePlanePoint", &VolumeSingleton::BasePlanePoint)
        .def_readwrite("MaxProductHeight", &VolumeSingleton::MaxProductHeight)
        .def_readwrite("CurrentVolumeMaxPointHeight", &VolumeSingleton::CurrentVolumeMaxPointHeight)
        .def_readwrite("CurrentVolumeAvgPointHeight", &VolumeSingleton::CurrentVolumeAvgPointHeight)
        .def_readwrite("VolumeImage", &VolumeSingleton::VolumeImage)
        ;

    py::class_<ScanningSingleton>(m, "ScanningSingleton", py::dynamic_attr())
        .def(py::init<>())
        .def("ComputeCameraPositions", &ScanningSingleton::ComputeCameraPositions)
        .def("StartScan", &ScanningSingleton::StartScan)
        .def("AddScan", &ScanningSingleton::AddScan)
        .def("RevertLastScan", &ScanningSingleton::RevertLastScan)
        .def("StartReferenceScan", &ScanningSingleton::StartReferenceScan)
        .def("AddReferenceScan", &ScanningSingleton::AddReferenceScan)
        .def("RevertLastReferenceScan", &ScanningSingleton::RevertLastReferenceScan)
        .def("RefineReferenceScan", &ScanningSingleton::RefineReferenceScan)
        .def("SetReferenceModel", py::overload_cast<std::shared_ptr<open3d::geometry::TriangleMesh>>(&ScanningSingleton::SetReferenceModel))
        .def("SetReferenceModel", py::overload_cast<std::shared_ptr<open3d::geometry::PointCloud>>(&ScanningSingleton::SetReferenceModel))
        .def("getLatestDistanceColorsGradient", &ScanningSingleton::getLatestDistanceColorsGradient)
        // .def("getDistanceColorsGradient", &ScanningSingleton::getDistanceColorsGradient)
        .def("getMissingPoints", &ScanningSingleton::getMissingPoints)
        .def("getVisibilityPCD", &ScanningSingleton::getVisibilityPCD)
        .def("getCalibrationPCD", &ScanningSingleton::getCalibrationPCD)
        .def("clearReferenceModel", &ScanningSingleton::clearReferenceModel)
        .def("ReferenceModelSet", &ScanningSingleton::ReferenceModelSet)
        .def_property_readonly("CalibrationManager", [](const ScanningSingleton& s){return s.CalibrationManager.get();}, py::return_value_policy::reference_internal)
        .def_readwrite("Cameras", &ScanningSingleton::Cameras)
        .def_readwrite("CurrentPCD", &ScanningSingleton::CurrentPCD)
        .def_readwrite("CurrentDownsampledPCD", &ScanningSingleton::CurrentDownsampledPCD)
        .def_readwrite("ReferenceScanPCD", &ScanningSingleton::ReferenceScanPCD)
        .def_readwrite("RefineAlignWithCAD", &ScanningSingleton::RefineAlignWithCAD)
        .def_readwrite("UseVoxelGrid", &ScanningSingleton::UseVoxelGrid)
        .def_readonly("FinalFitness", &ScanningSingleton::FinalFitness)
        ;

    py::class_<CalibrationSingleton>(m, "CalibrationSingleton") // used by ScanningSingleton
        .def(py::init<>())
        .def("getImagewithCorners", &CalibrationSingleton::getImagewithCorners)
        .def_readwrite("SquareLength", &CalibrationSingleton::SquareLength)
        .def_readwrite("Pattern_h", &CalibrationSingleton::Pattern_h)
        .def_readwrite("Pattern_w", &CalibrationSingleton::Pattern_w)
        ;


}