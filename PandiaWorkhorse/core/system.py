from flask import Blueprint, json, request, Response
from flask.json import jsonify
from core import CameraList, ScanningSingleton, VolumeSingleton, logger
from .threadfunctions import stopVolumeThreadFunction
from . import volume as vol
import cppBindings as cp
import traceback
import numpy as np

system = Blueprint('system', __name__)  # this actually gets registered

@system.route('/isAlive')
def isAlive():
    return jsonify(Message='OK')

#todo logic not quite right
#returns True if wh is active and cameralist in post request is identical
#returns json["Message"] = "busy" if wh is active but cameralist not identical
@system.route('/isActive', methods=['POST'])
def isActive():
    global CameraList
    if len(CameraList) == 0:
        return jsonify(Message=False)
    # for realCam in CameraList:
    #     if realCam.Connected:
    #         return jsonify(Message=True)
    serialNumbers = request.json['serialnumbers']
    allFound = True
    for serial in serialNumbers:
        found = False
        for realCam in CameraList:
            if realCam.SerialNumber == serial and realCam.Connected:
                found = True
        allFound = found
        if not allFound:
            break
    if allFound and len(serialNumbers) == len(CameraList):
        return jsonify(Message=True)
    elif len(CameraList) > 0:
        return jsonify(Message = "busy")
    else:
        return jsonify(Message=False)

@system.route('/clearCameraList')
def clearCameraList():
    try:
        stopVolumeThreadFunction()
        VolumeSingleton.Cameras= []
        ScanningSingleton.Cameras= []
        global CameraList
        CameraList.clear()
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route('/getAvailableCameras')
def getAvailableCameras():
    serialnumbers = getAvailableCameraSerialNumbersWithNames()
    return jsonify(content =serialnumbers)

#todo unnecessary, just serial numbers would be enough
def getAvailableCameraSerialNumbersWithClasses():
    out = []
    try:
        out.append({"serialnumbers": cp.IntelCamera.getAvailableCameras(), "class": cp.IntelCamera})
        out.append({"serialnumbers": cp.KinectCamera.getAvailableCameras(), "class": cp.KinectCamera})
        out.append({"serialnumbers": cp.ZividCamera.getAvailableCameras(), "class": cp.ZividCamera})
        out.append({"serialnumbers": cp.EnsensoCamera.getAvailableCameras(), "class": cp.EnsensoCamera})
        out.append({"serialnumbers": cp.SickCamera.getAvailableCameras(), "class": cp.SickCamera})
        out.append({"serialnumbers": cp.PhoxiCamera.getAvailableCameras(), "class": cp.PhoxiCamera})
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)
    return out

    #todo turn into dict
def getAvailableCameraSerialNumbersWithNames():
    out = []
    try:
        out.append((cp.IntelCamera.getAvailableCameras(),"IntelCamera"))
        out.append((cp.KinectCamera.getAvailableCameras(),"KinectCamera"))
        out.append((cp.ZividCamera.getAvailableCameras(),"ZividCamera"))
        out.append((cp.EnsensoCamera.getAvailableCameras(),"EnsensoCamera"))
        out.append((cp.SickCamera.getAvailableCameras(),"SickCamera"))
        out.append((cp.PhoxiCamera.getAvailableCameras(),"PhoxiCamera"))
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)
    return out

@system.route("/activateSystemWorkhorse", methods=['POST'])
def activateSystemWorkhorse():
    #check if all system cameras are available
    systemJson = request.json
    availableCameras = getAvailableCameraSerialNumbersWithClasses()
    systemCameras = systemJson['cameras']
    allThere = True
    found = False
    for cam in systemCameras:
        found = False
        camSerial = cam["serialnumber"]
        for avcams in availableCameras:
            for avcamserial in avcams["serialnumbers"]:
                if camSerial == avcamserial:
                    found = True
                    break
        if not found:
            allThere = False
            break

    if not allThere:
        return jsonify(Message="Not all system cameras found")

    VolumeSettings = systemJson['VolumeSettings']
    QualitySettings = systemJson['QualitySettings']
    SiloToFirstCamTransform = np.fromstring(VolumeSettings['SiloToFirstCamTransform'], sep=',').reshape([4, 4])
    SiloToFirstCamTransform[0:3, 3] = SiloToFirstCamTransform[0:3, 3] / 1000
        
    #Create the CameraWrapper objects in the camera list
    clearCameraList()
    for cam in systemCameras:
        camSettings = cam["settings"]
        camClass = ""
        if cam["type"] == "IntelCamera":
            camClass = cp.IntelCamera
        if cam["type"] == "KinectCamera":
            camClass = cp.KinectCamera
        if cam["type"] == "ZividCamera":
            camClass = cp.ZividCamera
        if cam["type"] == "EnsensoCamera":
            camClass = cp.EnsensoCamera
        if cam["type"] == "SickCamera":
            camClass = cp.SickCamera
        if cam["type"] == "PhoxiCamera":
            camClass = cp.PhoxiCamera
        camPointer = camClass.CreateCameraPointer()
        if camPointer.connectBySerialNumber(cam['serialnumber']): 
            CameraList.append(camPointer)
            CameraList[-1].crop_left = camSettings['crop_left']
            CameraList[-1].crop_right = camSettings['crop_right']
            CameraList[-1].crop_top = camSettings['crop_top']
            CameraList[-1].crop_bottom = camSettings['crop_bottom']
            CameraList[-1].crop_depth = camSettings['crop_depth']
            CameraList[-1].crop_depth_inv = camSettings['crop_depth_inv']
            CameraList[-1].crop_ground_height = camSettings['crop_ground_height']
            CameraList[-1].crop_ground_height_fine = camSettings['crop_ground_height_fine']
            CameraList[-1].dmax = camSettings['dmax']
            CameraList[-1].ColorThresholdMin = camSettings['ColorThresholdMin'] * 0.001
            CameraList[-1].ColorThresholdMax = camSettings['ColorThresholdMax'] * 0.001
            CameraList[-1].Extrinsic = np.fromstring(camSettings['extrinsic'], sep=',').reshape([4, 4])
            CameraList[-1].GravityVector = np.fromstring(camSettings['gravityvector'], sep=',')
        else:
            return jsonify(Message="Connecting to a camera failed")
        
        # volume
        VolumeSingleton.Cameras = CameraList
        VolumeSingleton.x_offset = VolumeSettings['x_offset']/1000
        VolumeSingleton.y_offset = VolumeSettings['y_offset']/1000
        VolumeSingleton.z_offset = VolumeSettings['z_offset']/1000
        VolumeSingleton.alpha = VolumeSettings['alpha']* 2*3.141592/360
        VolumeSingleton.beta = VolumeSettings['beta']* 2*3.141592/360
        VolumeSingleton.gamma = VolumeSettings['gamma']* 2*3.141592/360
        VolumeSingleton.TranslationOffset= np.fromstring(VolumeSettings['TranslationOffset'], sep=',')
        VolumeSingleton.SiloToFirstCamTransform = SiloToFirstCamTransform
        VolumeSingleton.VolumeNoise = VolumeSettings['VolumeNoise']/1000
        VolumeSingleton.DensityFactor = VolumeSettings['DensityFactor']
        VolumeSingleton.BeltSpeed = VolumeSettings['BeltSpeed']
        VolumeSingleton.DirectionVector = np.fromstring(VolumeSettings['DirectionVector'], sep=',')
        VolumeSingleton.BasePlanePoint = np.fromstring(VolumeSettings['BasePlanePoint'], sep=',')
        VolumeSingleton.setBasePlane()
        VolumeSingleton.MaxProductHeight = VolumeSettings['MaxProductHeight'] / 1000
        vol.ProductPresenceThreshold = VolumeSettings['ProductPresenceThreshold']
        vol.UseBeltSpeedEndpoint = VolumeSettings['UseBeltSpeedEndpoint']
        vol.BeltSpeedEndpointAddress = VolumeSettings['BeltSpeedEndpointAddress']
        vol.BeltSpeedEndpointPayload = VolumeSettings['BeltSpeedEndpointPayload']
        vol.BeltSpeedEndpointDistance = VolumeSettings['BeltSpeedEndpointDistance']
        vol.BeltSpeedEndpointUsername = VolumeSettings['BeltSpeedEndpointUsername']
        vol.BeltSpeedEndpointPassword = VolumeSettings['BeltSpeedEndpointPassword']

        # quality
        ScanningSingleton.Cameras = CameraList
        ScanningSingleton.CalibrationManager.SquareLength = QualitySettings['SquareLength'] * 0.001
        ScanningSingleton.CalibrationManager.Pattern_w = QualitySettings['Pattern_w']
        ScanningSingleton.CalibrationManager.Pattern_h = QualitySettings['Pattern_h']
        ScanningSingleton.RefineAlignWithCAD = QualitySettings['RefineAlignWithCAD']
        ScanningSingleton.UseVoxelGrid = QualitySettings['UseVoxelGrid']
    
        
    print("Successfully activated system!")
    return jsonify(Message="OK")


