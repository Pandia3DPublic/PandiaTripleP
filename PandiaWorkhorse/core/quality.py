from flask import Blueprint, request, Response, make_response
from flask.json import jsonify
from core import ScanningSingleton, CameraList, logger
import numpy as np
from .conversions import *
import traceback
import open3d as o3d
import time
import sys
import copy

quality = Blueprint('quality', __name__)  # this actually gets registered


@quality.route('/computeCameraPositions')
def computeCameraPositions():
    try:
        success = ScanningSingleton.ComputeCameraPositions()
        if not success:
            return jsonify(Message='Failed')

        jsondict = {}
        for i, camera in enumerate(CameraList):
            cameradict = {
                "serialnumber" : camera.SerialNumber,
                "extrinsic": camera.Extrinsic.copy().tolist(),
                "gravityvector" : camera.GravityVector.copy().tolist()
            }
            jsondict.update({f'camera{i}':cameradict})
        return jsonify(jsondict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@quality.route('/getCornerImages')
def getCornerImages():
    try:
        jsondict = {}
        for i, camera in enumerate(CameraList):
            img = ScanningSingleton.CalibrationManager.getImagewithCorners(camera)
            img = np.array(img).copy()
            if img.size != 0:
                img = numpyToUTF8JPG(img, 75)
                dict = {
                    "serialnumber" : camera.SerialNumber,
                    "image": img
                }
                jsondict.update({f'camera{i}':dict})
        if not jsondict:
            return jsonify(Message='Failed')
        return jsonify(jsondict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@quality.route('/startScan')
def startScan():
    try:
        if not ScanningSingleton.ReferenceModelSet():
            return jsonify(Message='ReferenceModelNotSet')
        success = ScanningSingleton.StartScan()
        if not success:
            return jsonify(Message='Failed')
        else:
            return jsonify({"fitness": ScanningSingleton.FinalFitness})

    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@quality.route('/addScan')
def addScan():
    try:
        success = ScanningSingleton.AddScan()
        if not success:
            return jsonify(Message='Failed')
        else:
            return jsonify({"fitness": ScanningSingleton.FinalFitness})

    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@quality.route('/revertLastScan')
def revertLastScan():
    try:
        success = ScanningSingleton.RevertLastScan()
        if not success:
            return jsonify(Message='Failed')
        else:
            return jsonify({"fitness": ScanningSingleton.FinalFitness})

    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@quality.route('/getCurrentPCD')
def getCurrentPCD():
    try:
        pcd = ScanningSingleton.CurrentPCD
        if pcd.is_empty():
            return jsonify(Message='Failed')

        points = np.asarray(pcd.points).flatten()
        points *= 1000
        colors = np.asarray(pcd.colors).flatten()
        points = np.around(points, 2).tolist()
        colors = np.around(colors, 2).tolist()
        jsondict = {'points': points, 'colors': colors}
        return jsonify(jsondict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)




@quality.route('/getLatestCameraPCDs')
def getLatestCameraPCDs():
    # start = time.time() 
    try:
        jsonlist =[]
        for cam in CameraList:
            if len(cam.CroppedPcds) == 0:
                return jsonify(Message='Failed')
            pcd = copy.deepcopy(cam.CroppedPcds[-1]).transform(cam.T_camToRef[-1])
            points = np.asarray(pcd.points).flatten()
            points *= 1000
            colors = np.asarray(pcd.colors).flatten()
            points = np.around(points, 2).tolist()
            colors = np.around(colors, 2).tolist()
            localdict = {'points': points, 'colors': colors, "ApproxVoxelSize" : cam.ApproxVoxelSize}
            jsonlist.append(localdict)
        tmp = jsonify(jsonlist)
        # print("Time passedo n python side " + str(time.time() - start))
        return tmp#jsonify(jsonlist)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

# @quality.route('/getGradientColors')
# def getGradientColors():
#     try:
#         colors = ScanningSingleton.getDistanceColorsGradient()
#         if len(colors) == 0:
#             return jsonify(Message='Failed')
#         colors = np.asarray(colors).flatten()
#         colors = np.around(colors, 2).tolist()
#         jsondict = {'colors': colors}
#         return jsonify(jsondict)
#     except Exception:
#         logger.error("Exception:" + traceback.format_exc())
#         return Response(status=420)


@quality.route('/getLatestGradientColors')
def getLatestGradientColors():
    try:
        jsonlist =[]
        pcdcolors = ScanningSingleton.getLatestDistanceColorsGradient()
        for singlePcdColor in pcdcolors:
            if len(singlePcdColor) == 0:
                return jsonify(Message='Failed')
            singlePcdColor = np.asarray(singlePcdColor).flatten()
            singlePcdColor = np.around(singlePcdColor, 2).tolist()
            localdict = {'colors': singlePcdColor}
            jsonlist.append(localdict)
        return jsonify(jsonlist)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@quality.route('/getMissingPoints')
def getMissingPoints():
    try:
        pcd = ScanningSingleton.getMissingPoints()
        points = np.asarray(pcd.points).flatten()
        points *= 1000
        points = np.around(points,2).tolist()
        jsondict = {'points': points}
        return jsonify(jsondict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@quality.route('/getVisibilityPCD')
def getVisibilityPCD():
    try:
        pcd = ScanningSingleton.getVisibilityPCD()
        if pcd.is_empty():
            return jsonify(Message='Failed')

        points = np.asarray(pcd.points).flatten()
        points *= 1000
        colors = np.asarray(pcd.colors).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        jsondict = {'points': points, 'colors': colors}
        return jsonify(jsondict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@quality.route('/getCalibrationPCD')
def getCalibrationPCD():
    try:
        pcd = ScanningSingleton.getCalibrationPCD()
        if pcd.is_empty():
            return jsonify(Message='Failed')

        points = np.asarray(pcd.points).flatten()
        points *= 1000
        colors = np.asarray(pcd.colors).flatten()
        points = np.around(points,2).tolist()
        colors = np.around(colors,2).tolist()

        jsondict = {'points': points, 'colors': colors}
        return jsonify(jsondict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@quality.route('/setQualitySettings', methods=['POST'])
def setQualitySettings():
    try:
        json = request.json
        if 'SquareLength' in json:
            ScanningSingleton.CalibrationManager.SquareLength = json['SquareLength'] * 0.001
        if 'Pattern_w' in json:
            ScanningSingleton.CalibrationManager.Pattern_w = json['Pattern_w']
        if 'Pattern_h' in json:
            ScanningSingleton.CalibrationManager.Pattern_h = json['Pattern_h']
        if 'RefineAlignWithCAD' in json:
            ScanningSingleton.RefineAlignWithCAD = json['RefineAlignWithCAD']
        if 'UseVoxelGrid' in json:
            ScanningSingleton.UseVoxelGrid = json['UseVoxelGrid']
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)



@quality.route('/getQualitySettings')
def getQualitySettings():
    try:
        jsondict = {
            'SquareLength': round(ScanningSingleton.CalibrationManager.SquareLength * 1000, 5),
            'Pattern_w': ScanningSingleton.CalibrationManager.Pattern_w,
            'Pattern_h': ScanningSingleton.CalibrationManager.Pattern_h,
            'RefineAlignWithCAD': ScanningSingleton.RefineAlignWithCAD,
            'UseVoxelGrid': ScanningSingleton.UseVoxelGrid,
        }
        return jsonify(jsondict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


