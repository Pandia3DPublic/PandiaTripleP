#include "PhoxiUtil.h"

void printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList)
{
    for (std::size_t i = 0; i < DeviceList.size(); ++i)
    {
        std::cout << "Device: " << i << std::endl;
        printDeviceInfo(DeviceList[i]);
    }
}

void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo)
{
    std::cout << "  Name:                    " << DeviceInfo.Name << std::endl;
    std::cout << "  Hardware Identification: " << DeviceInfo.HWIdentification << std::endl;
    std::cout << "  Type:                    " << std::string(DeviceInfo.Type) << std::endl;
    std::cout << "  Firmware version:        " << DeviceInfo.FirmwareVersion << std::endl;
    std::cout << "  Variant:                 " << DeviceInfo.Variant << std::endl;
    std::cout << "  IsFileCamera:            " << (DeviceInfo.IsFileCamera ? "Yes" : "No") << std::endl;
    std::cout << "  Status:                  "
        << (DeviceInfo.Status.Attached ? "Attached to PhoXi Control. " : "Not Attached to PhoXi Control. ")
        << (DeviceInfo.Status.Ready ? "Ready to connect" : "Occupied")
        << std::endl << std::endl;
}

void printFrameInfo(const pho::api::PFrame &Frame)
{
    const pho::api::FrameInfo &FrameInfo = Frame->Info;
    std::cout << "  Frame params: " << std::endl;
    std::cout << "    Frame Index: "                << FrameInfo.FrameIndex << std::endl;
    std::cout << "    Frame Timestamp: "            << FrameInfo.FrameTimestamp << " s" << std::endl;
    std::cout << "    Frame Acquisition duration: " << FrameInfo.FrameDuration << " ms" << std::endl;
    std::cout << "    Frame Computation duration: " << FrameInfo.FrameComputationDuration << " ms" << std::endl;
    std::cout << "    Frame Transfer duration: "    << FrameInfo.FrameTransferDuration << " ms" << std::endl;
    std::cout << "    Sensor Position: ["
        << FrameInfo.SensorPosition.x << "; "
        << FrameInfo.SensorPosition.y << "; "
        << FrameInfo.SensorPosition.z << "]"
        << std::endl;
    std::cout << "    Total scan count: "           << FrameInfo.TotalScanCount << std::endl;
}

void printFrameData(const pho::api::PFrame &Frame)
{
    if (Frame->Empty())
    {
        std::cout << "Frame is empty.";
        return;
    }
    std::cout << "  Frame data: " << std::endl;
    if (!Frame->PointCloud.Empty())
    {
        std::cout << "    PointCloud:    ("
            << Frame->PointCloud.Size.Width << " x "
            << Frame->PointCloud.Size.Height << ") Type: "
            << Frame->PointCloud.GetElementName()
            << std::endl;
    }
    if (!Frame->NormalMap.Empty())
    {
        std::cout << "    NormalMap:     ("
            << Frame->NormalMap.Size.Width << " x "
            << Frame->NormalMap.Size.Height << ") Type: "
            << Frame->NormalMap.GetElementName()
            << std::endl;
    }
    if (!Frame->DepthMap.Empty())
    {
        std::cout << "    DepthMap:      ("
            << Frame->DepthMap.Size.Width << " x "
            << Frame->DepthMap.Size.Height << ") Type: "
            << Frame->DepthMap.GetElementName()
            << std::endl;
    }
    if (!Frame->ConfidenceMap.Empty())
    {
        std::cout << "    ConfidenceMap: ("
            << Frame->ConfidenceMap.Size.Width << " x "
            << Frame->ConfidenceMap.Size.Height << ") Type: "
            << Frame->ConfidenceMap.GetElementName()
            << std::endl;
    }
    if (!Frame->Texture.Empty())
    {
        std::cout << "    Texture:       ("
            << Frame->Texture.Size.Width << " x "
            << Frame->Texture.Size.Height << ") Type: "
            << Frame->Texture.GetElementName()
            << std::endl;
    }
    if (!Frame->TextureRGB.Empty())
    {
        std::cout << "    TextureRGB:       ("
            << Frame->TextureRGB.Size.Width << " x "
            << Frame->TextureRGB.Size.Height << ") Type: "
            << Frame->TextureRGB.GetElementName()
            << std::endl;
    }
}

void PrintCapturingSettings(const pho::api::PhoXiCapturingSettings &CapturingSettings)
{
    std::cout << "  CapturingSettings: "         << std::endl;
    std::cout << "    ShutterMultiplier: "       << CapturingSettings.ShutterMultiplier << std::endl;
    std::cout << "    ScanMultiplier: "          << CapturingSettings.ScanMultiplier << std::endl;
    std::cout << "    CameraOnlyMode: "          << CapturingSettings.CameraOnlyMode << std::endl;
    std::cout << "    AmbientLightSuppression: " << CapturingSettings.AmbientLightSuppression << std::endl;
    std::cout << "    MaximumFPS: "              << CapturingSettings.MaximumFPS << std::endl;
    std::cout << "    SinglePatternExposure: "   << CapturingSettings.SinglePatternExposure << std::endl;
    std::cout << "    CodingStrategy: "          << std::string(CapturingSettings.CodingStrategy) << std::endl;
    std::cout << "    CodingQuality: "           << std::string(CapturingSettings.CodingQuality) << std::endl;
    std::cout << "    TextureSource: "           << std::string(CapturingSettings.TextureSource) << std::endl;
}

void PrintProcessingSettings(const pho::api::PhoXiProcessingSettings &ProcessingSettings)
{
    std::cout << "  ProcessingSettings: " << std::endl;
    std::cout << "    Confidence (MaxInaccuracy): " << ProcessingSettings.Confidence << std::endl;
    std::cout << "    CalibrationVolumeOnly: " << ProcessingSettings.CalibrationVolumeOnly;
    PrintVector("MinCameraSpace(in DataCutting)", ProcessingSettings.ROI3D.CameraSpace.min);
    PrintVector("MaxCameraSpace(in DataCutting)", ProcessingSettings.ROI3D.CameraSpace.max);
    PrintVector("MinPointCloudSpace (in DataCutting)", ProcessingSettings.ROI3D.PointCloudSpace.min);
    PrintVector("MaxPointCloudSpace (in DataCutting)", ProcessingSettings.ROI3D.PointCloudSpace.max);
    std::cout << "    MaxCameraAngle: "          << ProcessingSettings.NormalAngle.MaxCameraAngle << std::endl;
    std::cout << "    MaxProjectionAngle: "      << ProcessingSettings.NormalAngle.MaxProjectorAngle << std::endl;
    std::cout << "    MinHalfwayAngle: "         << ProcessingSettings.NormalAngle.MinHalfwayAngle << std::endl;
    std::cout << "    MaxHalfwayAngle: "         << ProcessingSettings.NormalAngle.MaxHalfwayAngle << std::endl;
    std::cout << "    SurfaceSmoothness: "       << std::string(ProcessingSettings.SurfaceSmoothness) << std::endl;
    std::cout << "    NormalsEstimationRadius: " << ProcessingSettings.NormalsEstimationRadius << std::endl;
    std::cout << "    InterreflectionsFiltering: " << ProcessingSettings.InterreflectionsFiltering;
}

void PrintCoordinatesSettings(const pho::api::PhoXiCoordinatesSettings &CoordinatesSettings)
{
    std::cout << "  CoordinatesSettings: " << std::endl;
    PrintMatrix("CustomRotationMatrix", CoordinatesSettings.CustomTransformation.Rotation);
    PrintVector("CustomTranslationVector", CoordinatesSettings.CustomTransformation.Translation);
    PrintMatrix("RobotRotationMatrix", CoordinatesSettings.CustomTransformation.Rotation);
    PrintVector("RobotTranslationVector", CoordinatesSettings.RobotTransformation.Translation);
    std::cout << "    CoordinateSpace: " << std::string(CoordinatesSettings.CoordinateSpace) << std::endl;
    std::cout << "    RecognizeMarkers: " << CoordinatesSettings.RecognizeMarkers << std::endl;
    std::cout << "    MarkerScale: "
        << CoordinatesSettings.MarkersSettings.MarkerScale.Width << " x "
        << CoordinatesSettings.MarkersSettings.MarkerScale.Height
        << std::endl;
}

void PrintCalibrationSettings(const pho::api::PhoXiCalibrationSettings &CalibrationSettings)
{
    std::cout << "  CalibrationSettings: " << std::endl;
    std::cout << "    FocusLength: " << CalibrationSettings.FocusLength << std::endl;
    std::cout << "    PixelSize: "
        << CalibrationSettings.PixelSize.Width << " x "
        << CalibrationSettings.PixelSize.Height
        << std::endl;
    PrintMatrix("CameraMatrix", CalibrationSettings.CameraMatrix);
    std::cout << "    DistortionCoefficients: " << std::endl;
    std::cout << "      Format is the following: " << std::endl;
    std::cout << "      (k1, k2, p1, p2[, k3[, k4, k5, k6[, s1, s2, s3, s4[, tx, ty]]]])" << std::endl;

    std::vector<double> distCoeffs = CalibrationSettings.DistortionCoefficients;
    std::stringstream currentDistCoeffsSS;
    int brackets = 0;
    currentDistCoeffsSS << "(";
    currentDistCoeffsSS << distCoeffs[0];
    for (int i = 1; i < distCoeffs.size(); ++i)
    {
        if (i == 4 || i == 5 || i == 8 || i == 12 || i == 14)
        {
            currentDistCoeffsSS << "[";
            ++brackets;
        }
        currentDistCoeffsSS << ", " << distCoeffs[i];
    }
    for (int j = 0; j < brackets; ++j)
    {
        currentDistCoeffsSS << "]";
    }
    currentDistCoeffsSS << ")";
    std::cout << "      " << currentDistCoeffsSS.str() << std::endl;
}

void PrintVector(const std::string &name, const pho::api::Point3_64f &vector)
{
    std::cout << "    " << name << ": ["
        << vector.x << "; "
        << vector.y << "; "
        << vector.z << "]"
        << std::endl;
}

void PrintMatrix(const std::string &name, const pho::api::CameraMatrix64f &matrix)
{
    std::cout << "    " << name << ": "
        << std::endl << "      ["
        << matrix[0][0] << ", "
        << matrix[0][1] << ", "
        << matrix[0][2] << "]"

        << std::endl << "      ["
        << matrix[1][0] << ", "
        << matrix[1][1] << ", "
        << matrix[1][2] << "]"

        << std::endl << "      ["
        << matrix[2][0] << ", "
        << matrix[2][1] << ", "
        << matrix[2][2] << "]"
        << std::endl;
}

void printProfilesList(const std::vector<pho::api::PhoXiProfileDescriptor> &ProfilesList)
{
    std::cout << std::boolalpha << true;
    for (const pho::api::PhoXiProfileDescriptor &profile : ProfilesList)
    {
        std::cout << "Profile: " << std::endl;
        std::cout << "  Name: " << profile.Name << std::endl;
        std::cout << "  Is factory profile: " << profile.IsFactory << std::endl;
    }
}