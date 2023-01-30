from flask import Blueprint, json, request, Response
from flask.json import jsonify
import numpy as np
import open3d as o3d
import cppBindings as cp
import cv2
from core import CameraList, VolumeSingleton, ScanningSingleton, logger
from .conversions import *
from .utils import getCameraIndex
import traceback

base = Blueprint('base', __name__)  # this actually gets registered

@base.route('/clearModels')
def clearModels():
    try:
        VolumeSingleton.clearSiloModel()
        ScanningSingleton.clearReferenceModel()
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@base.route('/getFOVValues', methods=['POST'])
def getFOVValues():
    try:
        data = request.json
        i = getCameraIndex(CameraList, data['serialnumber'])
        crop_top = CameraList[i].crop_top
        crop_bottom = CameraList[i].crop_bottom
        crop_left = CameraList[i].crop_left
        crop_right = CameraList[i].crop_right
        crop_depth = CameraList[i].crop_depth
        crop_depth_inv = CameraList[i].crop_depth_inv
        crop_ground_height = round(CameraList[i].crop_ground_height, 5)
        crop_ground_height_fine = round(CameraList[i].crop_ground_height_fine, 5)

        JsonDict = {'crop_top': crop_top, 'crop_bottom': crop_bottom, 'crop_left': crop_left,
                    'crop_right': crop_right, 'crop_depth': crop_depth, 'crop_depth_inv': crop_depth_inv,
                    'crop_ground_height' : crop_ground_height, 'crop_ground_height_fine' : crop_ground_height_fine}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


#note: first a options request is sent, then the post request. so this is called twice
@base.route('/setFOVValues', methods=['POST'])
def setFOVValues():
    try:
        json = request.json
        i = getCameraIndex(CameraList, json['serialnumber'])
        CameraList[i].crop_top = json['crop_top']
        CameraList[i].crop_bottom = json['crop_bottom']
        CameraList[i].crop_left = json['crop_left']
        CameraList[i].crop_right = json['crop_right']
        CameraList[i].crop_depth = json['crop_depth']
        CameraList[i].crop_depth_inv = json['crop_depth_inv']
        if 'crop_ground_height' in json:
            CameraList[i].crop_ground_height = json['crop_ground_height']
        if 'crop_ground_height_fine' in json:
            CameraList[i].crop_ground_height_fine = json['crop_ground_height_fine']
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@base.route('/getFOVImage', methods=['POST'])
def getFOVImage():
    try:
        json = request.json
        i = getCameraIndex(CameraList, json['serialnumber'])
        crop_top = CameraList[i].crop_top
        crop_bottom = CameraList[i].crop_bottom
        crop_left = CameraList[i].crop_left
        crop_right = CameraList[i].crop_right

        img = CameraList[i].FRCalibrationImages[0]
        img = np.array(img).copy()
        if (img.size != 0):
            height, width, ch = img.shape
            halfWidth = width / 2.0
            halfHeight = height / 2.0
            left_thres = crop_left / 100.0 * halfWidth
            right_thres = (1 - crop_right / 100.0) * halfWidth + halfWidth
            top_thres = crop_top / 100.0 * halfHeight
            bottom_thres = (1 - crop_bottom / 100.0) * halfHeight + halfHeight

            img = cv2.rectangle(img, (int(left_thres), int(top_thres)), (int(right_thres), int(bottom_thres)), (0, 255, 239), 2)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = numpyToUTF8JPG(img, 50, 500)
        else:
            img = ''
        return jsonify({'image': img})
        
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@base.route('/receive3DModel/<app>', methods=['POST'])
def receive3DModel(app):
    try:
        json = request.get_json()
        if 'vertices' in json:
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(np.asarray(json['vertices']))
            mesh.triangles = o3d.utility.Vector3iVector(np.asarray(json['triangles']).astype(np.int32))
            mesh.scale(0.001,center = mesh.get_center()) #cpp side uses m as standard unit
            if not mesh.has_triangle_normals():
                mesh.compute_triangle_normals()
            mesh.normalize_normals()
            if app == 'Volume':
                VolumeSingleton.setSiloModel(mesh)
            elif app == 'Quality':
                ScanningSingleton.SetReferenceModel(mesh)
            else:
                print("Unkown arg in receive3DModel")
        elif 'points' in json:
            pcd = o3d.geometry.PointCloud()
            points = np.asarray(json['points'])
            points /= 1000
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(np.asarray(json['colors']))
            pcd.translate(-pcd.get_center())
            if app == 'Volume':
                VolumeSingleton.setSiloModel(pcd)
            elif app == 'Quality':
                ScanningSingleton.SetReferenceModel(pcd)
            else:
                print("Unkown arg in receive3DModel")
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@base.route('/getPreviewImg', methods=['POST'])
def getPreviewImg():
    try:
        jsonData= request.json
        cameraName = jsonData['cameraName']
        type = cameraName.split(' : ')[0]
        serialnumber = cameraName.split(' : ')[1]

        #check if camera is already connected and busy
        global CameraList
        for camera in CameraList:
            if camera.SerialNumber == serialnumber and camera.Connected:
                print('Warning in getPreviewImg: Camera is already connected and busy.')
                return jsonify({'image': ''})

        cam = False
        if type == 'IntelCamera':
            cam = cp.IntelCamera()
        if type == 'KinectCamera':
            cam = cp.KinectCamera()
        if type == 'EnsensoCamera':
            cam = cp.EnsensoCamera()
        if type == 'ZividCamera':
            cam = cp.ZividCamera()
        if type == 'SickCamera':
            cam = cp.SickCamera()
        if type == 'PhoxiCamera':
            cam = cp.PhoxiCamera()
        if not cam:
            print('Error in getPreviewImg: supplied cam type is unknown!')
            return jsonify({'image': ''})

        if not cam.connectBySerialNumber(serialnumber):
            print('Error in getPreviewImg: connect failed!')
            return jsonify({'image': ''})

        if not cam.record(1, 0):
            print('Error in getPreviewImg: record failed!')
            return jsonify({'image': ''})

        img = cam.FRCalibrationImages[0]
        img = np.array(img)
        if (img.size != 0):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = numpyToUTF8JPG(img, 75, 800)
        else:
            img = ''
        return jsonify({'image': img})
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)
        

@base.route('/startReferenceScan')
def startReferenceScan():
    try:
        success = ScanningSingleton.StartReferenceScan()
        if not success:
            return jsonify(Message='Failed')
        
        pcd = ScanningSingleton.ReferenceScanPCD
        points = np.asarray(pcd.points).copy()
        points = points - pcd.get_center()
        points = points.flatten()
        points *= 1000
        colors = np.asarray(pcd.colors).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        JsonDict = {'points': points, 'colors': colors}
        return jsonify(JsonDict)
        
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@base.route('/AddReferenceScan')
def AddReferenceScan():
    try:
        success = ScanningSingleton.AddReferenceScan()
        if not success:
            return jsonify(Message='Failed')
        
        pcd = ScanningSingleton.ReferenceScanPCD
        points = np.asarray(pcd.points).copy()
        points = points - pcd.get_center()
        points = points.flatten()
        points *= 1000
        colors = np.asarray(pcd.colors).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        JsonDict = {'points': points, 'colors': colors}
        return jsonify(JsonDict)
        
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@base.route('/RevertLastReferenceScan')
def RevertLastReferenceScan():
    try:
        success = ScanningSingleton.RevertLastReferenceScan()
        if not success:
            return jsonify(Message='Failed')
        
        pcd = ScanningSingleton.ReferenceScanPCD
        points = np.asarray(pcd.points).copy()
        points = points - pcd.get_center()
        points = points.flatten()
        points *= 1000
        colors = np.asarray(pcd.colors).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        JsonDict = {'points': points, 'colors': colors}
        return jsonify(JsonDict)
        
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@base.route('/RefineReferenceScan')
def RefineReferenceScan():
    try:
        success = ScanningSingleton.RefineReferenceScan()
        if not success:
            return jsonify(Message='Failed')
        
        pcd = ScanningSingleton.ReferenceScanPCD
        points = np.asarray(pcd.points).copy()
        points = points - pcd.get_center()
        points = points.flatten()
        points *= 1000
        colors = np.asarray(pcd.colors).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        JsonDict = {'points': points, 'colors': colors}
        return jsonify(JsonDict)
        
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@base.route('/getReferenceScan')
def getReferenceScan():
    try:
        pcd = ScanningSingleton.ReferenceScanPCD
        if pcd.is_empty():
            return jsonify(Message='Failed')

        points = np.asarray(pcd.points).flatten()
        points *= 1000
        colors = np.asarray(pcd.colors).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        JsonDict = {'points': points, 'colors': colors}
        return jsonify(JsonDict)

    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

#### old camera functions #####

@base.route('/recordAndGetCroppedPcd', methods=['POST'])
def recordAndGetCroppedPcd():
    try:
        data = request.json
        i = getCameraIndex(CameraList, data['serialnumber'])

        CameraList[i].clearBuffers()
        success = CameraList[i].record(3)
        # CameraList[i].combineRecordedFrames(VolumeSingleton.DirectionVector, VolumeSingleton.BeltSpeed) #debug
        if not success:
            print("recording failed")
            return jsonify(Message='Failed')
        pcd = CameraList[i].Pcds[-1]
        pcd = pcd.voxel_down_sample(0.01)
        pcd = CameraList[i].getCroppedPcd(pcd)

        points = np.asarray(pcd.points).flatten()
        points *= 1000
        colors = (np.asarray(pcd.colors) * np.array([0,0,1])).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        JsonDict = {'points': points, 'colors': colors}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@base.route('/getCroppedPcd', methods=['POST'])
def getCroppedPcd():
    try:
        data = request.json
        i = getCameraIndex(CameraList, data['serialnumber'])

        if len(CameraList[i].Pcds) == 0:
            success = CameraList[i].record(1)
            if not success:
                print("recording failed")
                return jsonify(Message='Failed')
        pcd = CameraList[i].Pcds[-1]
        pcd = pcd.voxel_down_sample(0.01)
        pcd = CameraList[i].getCroppedPcd(pcd)

        points = np.asarray(pcd.points).flatten()
        points *= 1000
        colors = (np.asarray(pcd.colors) * np.array([0,0,1])).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        JsonDict = {'points': points, 'colors': colors}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


####### Camera base functions ##########


@base.route('/recordAndReturnFirstBeltPcd', methods=['POST'])
def recordAndReturnFirstBeltPcd():
    try:
        data = request.json
        i = getCameraIndex(CameraList, data['serialnumber'])

        CameraList[i].clearBuffers()
        success = CameraList[i].record(1)
        if not success:
            print("recording failed")
            return jsonify(Message='Failed')
        VolumeSingleton.FirstDirectionPcd = CameraList[i].getCroppedPcd(CameraList[i].Pcds[-1])
        threejsPcd = VolumeSingleton.FirstDirectionPcd.voxel_down_sample(0.01)
        points = np.asarray(threejsPcd.points).flatten()
        points *= 1000
        colors = (np.asarray(threejsPcd.colors) * np.array([0,0,1])).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        JsonDict = {'points': points, 'colors': colors}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@base.route('/recordAndReturnSecondBeltPcd', methods=['POST'])
def recordAndReturnSecondBeltPcd():
    try:
        data = request.json
        i = getCameraIndex(CameraList, data['serialnumber'])

        CameraList[i].clearBuffers()
        success = CameraList[i].record(1)
        if not success:
            print("recording failed")
            return jsonify(Message='Failed')
        VolumeSingleton.SecondDirectionPcd = CameraList[i].getCroppedPcd(CameraList[i].Pcds[-1])
        threejsPcd = VolumeSingleton.SecondDirectionPcd.voxel_down_sample(0.01)
        points = np.asarray(threejsPcd.points).flatten()
        points *= 1000
        colors = (np.asarray(threejsPcd.colors) * np.array([0,1,0])).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        JsonDict = {'points': points, 'colors': colors}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@base.route('/calculateAndReturnVelocityVector')
def calculateAndReturnVelocityVector():
    try: 
        VolumeSingleton.CalculateDirectionVector()
        direction = VolumeSingleton.DirectionVector
        JsonDict = {'DirectionVector': direction.tolist(), "BasePlanePoint" : VolumeSingleton.BasePlanePoint.tolist()}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

    # return jsonify("ok")
    # print("calculateAndReturnVelocityVector")



@base.route('/setCameraSettings', methods=['POST'])
def setCameraSettings():
    try:
        json = request.json
        i = getCameraIndex(CameraList, json['serialnumber'])
        if 'ColorThresholdMin' in json:
             CameraList[i].ColorThresholdMin = json['ColorThresholdMin'] * 0.001
        if 'ColorThresholdMax' in json:
            CameraList[i].ColorThresholdMax = json['ColorThresholdMax'] * 0.001
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@base.route('/getCameraSettings', methods=['POST'])
def getCameraSettings():
    try:
        data = request.json
        i = getCameraIndex(CameraList, data['serialnumber'])
        crop_top = CameraList[i].crop_top
        crop_bottom = CameraList[i].crop_bottom
        crop_left = CameraList[i].crop_left
        crop_right = CameraList[i].crop_right
        crop_depth = CameraList[i].crop_depth
        crop_depth_inv = CameraList[i].crop_depth_inv
        crop_ground_height = round(CameraList[i].crop_ground_height, 5)
        crop_ground_height_fine = round(CameraList[i].crop_ground_height_fine, 5)
        ColorThresholdMin = round(CameraList[i].ColorThresholdMin * 1000,5)
        ColorThresholdMax = round(CameraList[i].ColorThresholdMax * 1000,5)

        JsonDict = {'crop_top': crop_top, 'crop_bottom': crop_bottom, 'crop_left': crop_left,
                    'crop_right': crop_right, 'crop_depth': crop_depth, 'crop_depth_inv': crop_depth_inv,
                    'crop_ground_height' : crop_ground_height, 'crop_ground_height_fine' : crop_ground_height_fine,
                    'ColorThresholdMin': ColorThresholdMin, 'ColorThresholdMax':ColorThresholdMax}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)



@base.route('/getNumberOfCameras')
def getNumberOfCameras():
    try:
        return jsonify(len(CameraList))
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)
