from flask import Blueprint, request, Response, make_response
from flask.json import jsonify
from core import VolumeSingleton, logger
import numpy as np
from datetime import datetime
import cv2
from .conversions import *
import traceback
import time

###### datetime notes #######
# https://docs.python.org/3/library/datetime.html
# Always use isoformat datetime/strings for best compatibility with javascript!
# YYYY-MM-DD HH:MM:SS.ffffff (we use ' ' as seperator)
# ----> to string: datetime.isoformat(' ')
# ----> from string: datetime.fromisoformat(string)
# use different formats only on interaction with user like labels, volume log download, etc.
# format codes examples: #24-hour "%Y-%m-%d %H:%M:%S.%f" , 12-hour "%Y-%m-%d %I:%M:%S.%f %p"

volume = Blueprint('volume', __name__)  # this actually gets registered

VolumeTimestaps = []
VolumeData = []
n_VolumeChart = 30
ConveyorBatchStartTime = 0
ConveyorBatchRunning = False
TotalVolumeFlow = 0
ProductPresenceThreshold = 1

UseBeltSpeedEndpoint = False
BeltSpeedEndpointAddress = False
BeltSpeedEndpointPayload = False
BeltSpeedEndpointDistance = False
BeltSpeedEndpointUsername = False
BeltSpeedEndpointPassword = False

@volume.route('/ConveyorBatchIsRunning')
def ConveyorBatchIsRunning():
    global ConveyorBatchRunning
    if ConveyorBatchRunning:
        return jsonify(Message='OK')
    else:
        return jsonify(Message='Conveyor batch not running')

@volume.route('/startConveyorBatch')
def startConveyorBatch():
    global ConveyorBatchRunning
    ConveyorBatchRunning = True
    global TotalVolumeFlow
    if TotalVolumeFlow == 0:
        global ConveyorBatchStartTime
        ConveyorBatchStartTime = datetime.now().isoformat(' ')
    return jsonify(Message='OK')


#todo batch: if paused the time keeps counting (see gui when starting again)
@volume.route('/stopConveyorBatch')
def stopConveyorBatch():
    global ConveyorBatchRunning
    ConveyorBatchRunning = False
    return jsonify(Message='OK')


@volume.route('/resetConveyorBatch')
def resetConveyorBatch():
    global ConveyorBatchRunning
    ConveyorBatchRunning = False
    global TotalVolumeFlow
    TotalVolumeFlow = 0
    global ConveyorBatchStartTime
    ConveyorBatchStartTime = 0
    VolumeSingleton.clearBeltAreaImage()
    return jsonify(Message='OK')


@volume.route('/getNewTotalVolumeFlowofCurrentConveyorBatch')
def getNewTotalVolumeFlowofCurrentConveyorBatch():
    # print("getTotalVolumeFlowofCurrentConveyorBatch")
    try:
        global TotalVolumeFlow
        JsonDict = {}
        if len(VolumeTimestaps) != 0:
            JsonDict = {'timestamp': VolumeTimestaps[-1], 'newTotalVolumeFlow': TotalVolumeFlow, 'BeltSpeed': VolumeSingleton.BeltSpeed}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)
    # calculate TotalVolumeFlow
    # return TotalVolumeFlow

@volume.route('/getConveyorBatchStartTime')
def getConveyorBatchStartTime():
    try:
        JsonDict = {'ConveyorBatchStartTime': ConveyorBatchStartTime}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)
    # calculate TotalVolumeFlow
    # return TotalVolumeFlow

@volume.route('/getNewVolumeImage')
def getNewVolumeImage():
    try:
        img = VolumeSingleton.VolumeImage  # always contains the newest image
        img = np.array(img)
        if (img.size != 0):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = numpyToUTF8JPG(img, 60, 800)
        else:
            img = ""
        JsonDict = {'image': img}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


lastTimestamp = -1
@volume.route('/getNewVolume')
def getNewVolume():
    try:
        global lastTimestamp
        global VolumeTimestaps
        global VolumeData
        JsonDict = {}
        if len(VolumeData) != 0:
            if VolumeTimestaps[-1] != lastTimestamp:
                lastTimestamp = VolumeTimestaps[-1]
                JsonDict = {'timestamp': VolumeTimestaps[-1], 'volume': VolumeData[-1]}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@volume.route('/getVolumeData')
def getVolumeData():
    try:
        volslice = 0
        timeslice = 0
        global n_VolumeChart
        if len(VolumeData) > n_VolumeChart:
            volslice = VolumeData[-n_VolumeChart:]
            timeslice = VolumeTimestaps[-n_VolumeChart:]
        else:
            volslice = VolumeData
            timeslice = VolumeTimestaps

        firstEntry = ''
        lastEntry = ''
        #note: we read each line and do not load the whole file because filesize can be huge
        with open('VolumeLog.txt', 'r') as logfile:
            firstEntry = logfile.readline()
        if firstEntry != '':
            with open('VolumeLog.txt', 'r') as logfile:
                for line in logfile:
                    lastEntry = line
        
        JsonDict = {'timestamps': timeslice, 'volumeData': volslice, 'firstEntry': firstEntry, 'lastEntry': lastEntry}
        return jsonify(JsonDict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


#see volume_thread_function for elements and order
VolumeLogLegend = ['Timestamp', 'Volume (l)', 'ProductPresence', 'ProductMoving',
                   'BeltSpeed (m/s)', 'AirPocketFactor', 'TotalVolumeFlow (l)',
                   'MaxPointHeight (mm)', 'AvgPointHeight (mm)']

@volume.route('/getVolumeLog', methods=['POST'])
def getVolumeLog():
    try:
        jsonData = request.json
        startTime = datetime.fromisoformat(jsonData["minDateTime"]).timestamp()
        stopTime = datetime.fromisoformat(jsonData["maxDateTime"]).timestamp()
        minidxset = False
        maxidxset = False
        responseList = []
        if startTime <= stopTime:
            with open('VolumeLog.txt', 'r') as logfile:
                for line in logfile:
                    splitData = line.split(" , ")
                    dateTime = datetime.fromisoformat(splitData[0])
                    time = dateTime.timestamp()
                    if time >= startTime and not minidxset:
                        minidxset = True
                    if time >= stopTime and not maxidxset:
                        maxidxset = True
                    splitData[0] = dateTime.strftime('%d-%m-%Y %H:%M:%S.%f') #change output format as wanted by user
                    if minidxset or maxidxset:
                        responseList.append(" , ".join(splitData))
                    if minidxset and maxidxset:
                        break
        responseList.insert(0, " , ".join(VolumeLogLegend) + '\n')
        return jsonify(responseList)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@volume.route('/deleteVolumeLog')
def deleteVolumeLog():
    try:
        f = open('VolumeLog.txt', 'r+')
        f.seek(0)
        f.truncate(0) # need '0' when using r+
        f.close()
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@volume.route('/toggleOverlay')
def toggleOverlay():
    VolumeSingleton.CreateOverlayinImage = not VolumeSingleton.CreateOverlayinImage
    return jsonify(Message='OK')


@volume.route('/centerButton')
def centerButton():
    try:
        success = VolumeSingleton.ProcessCenterButton()
        if not success:
            return jsonify(Message='Failed')
        SiloToFirstCamTransform = VolumeSingleton.SiloToFirstCamTransform.copy()
        SiloToFirstCamTransform[0:3, 3] = SiloToFirstCamTransform[0:3, 3] * 1000
        transdict = {'transformation': SiloToFirstCamTransform.tolist(), 'newOffsets': VolumeSingleton.TranslationOffset.tolist()}
        return jsonify(transdict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@volume.route('/icpButton')
def icpButton():
    try:
        success = VolumeSingleton.ProcessICPButton()
        if not success:
            return jsonify(Message='Failed')
        SiloToFirstCamTransform = VolumeSingleton.SiloToFirstCamTransform.copy()
        SiloToFirstCamTransform[0:3, 3] = SiloToFirstCamTransform[0:3, 3] * 1000
        transdict = {'transformation': SiloToFirstCamTransform.tolist()}
        return jsonify(transdict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@volume.route('/setPositionValues', methods=['POST'])
def setPositionValues():
    try:
        json = request.get_json()
        VolumeSingleton.x_offset = json['xposition']/1000
        VolumeSingleton.y_offset = json['yposition']/1000
        VolumeSingleton.z_offset = json['zposition']/1000
        VolumeSingleton.alpha = json['xrotation'] * 2*3.141592/360
        VolumeSingleton.beta = json['yrotation'] * 2*3.141592/360
        VolumeSingleton.gamma = json['zrotation'] * 2*3.141592/360
        VolumeSingleton.setSiloToFirstCamTransform()
        SiloToFirstCamTransform = VolumeSingleton.SiloToFirstCamTransform.copy()
        SiloToFirstCamTransform[0:3, 3] = SiloToFirstCamTransform[0:3, 3] * 1000
        transdict = {'transformation': SiloToFirstCamTransform.tolist()}
        return jsonify(transdict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@volume.route('/setVolumeSettings', methods=['POST'])
def setVolumeSettings():
    try:
        json = request.get_json()
        if 'VolumeNoise' in json:
            VolumeSingleton.VolumeNoise = float(json['VolumeNoise']) / 1000.0
        if 'DensityFactor' in json:
            VolumeSingleton.DensityFactor = float(json['DensityFactor'])
        if 'BeltSpeed' in json:
            VolumeSingleton.BeltSpeed = float(json['BeltSpeed'])
        if 'MaxProductHeight' in json:
            VolumeSingleton.MaxProductHeight = float(json['MaxProductHeight']) / 1000.0
        if 'ProductPresenceThreshold' in json:
            global ProductPresenceThreshold
            ProductPresenceThreshold = float(json['ProductPresenceThreshold'])
        if 'UseBeltSpeedEndpoint' in json:
            global UseBeltSpeedEndpoint
            UseBeltSpeedEndpoint = json['UseBeltSpeedEndpoint']
        if 'BeltSpeedEndpointAddress' in json:
            global BeltSpeedEndpointAddress
            BeltSpeedEndpointAddress = json['BeltSpeedEndpointAddress']
        if 'BeltSpeedEndpointPayload' in json:
            global BeltSpeedEndpointPayload
            BeltSpeedEndpointPayload = json['BeltSpeedEndpointPayload']
        if 'BeltSpeedEndpointDistance' in json:
            global BeltSpeedEndpointDistance
            BeltSpeedEndpointDistance = json['BeltSpeedEndpointDistance']
        if 'BeltSpeedEndpointUsername' in json:
            global BeltSpeedEndpointUsername
            BeltSpeedEndpointUsername = json['BeltSpeedEndpointUsername']
        if 'BeltSpeedEndpointPassword' in json:
            global BeltSpeedEndpointPassword
            BeltSpeedEndpointPassword = json['BeltSpeedEndpointPassword']
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@volume.route('/getVolumeSettings', methods=['GET'])
def getVolumeSettings():
    global ProductPresenceThreshold
    global UseBeltSpeedEndpoint
    global BeltSpeedEndpointAddress
    global BeltSpeedEndpointPayload
    global BeltSpeedEndpointDistance
    global BeltSpeedEndpointUsername
    global BeltSpeedEndpointPassword
    try:
        data = {
            'VolumeNoise': VolumeSingleton.VolumeNoise * 1000, 
            'DensityFactor': VolumeSingleton.DensityFactor, 
            'BeltSpeed': VolumeSingleton.BeltSpeed,
            'MaxProductHeight': VolumeSingleton.MaxProductHeight * 1000,
            'ProductPresenceThreshold': ProductPresenceThreshold,
            'UseBeltSpeedEndpoint': UseBeltSpeedEndpoint,
            'BeltSpeedEndpointAddress': BeltSpeedEndpointAddress,
            'BeltSpeedEndpointPayload': BeltSpeedEndpointPayload,
            'BeltSpeedEndpointDistance': BeltSpeedEndpointDistance,
            'BeltSpeedEndpointUsername': BeltSpeedEndpointUsername,
            'BeltSpeedEndpointPassword': BeltSpeedEndpointPassword,
        }
        return jsonify(data)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@volume.route('/readCalibrationData')
def readCalibrationData():
    try:
        SiloToFirstCamTransform = VolumeSingleton.SiloToFirstCamTransform.copy()
        SiloToFirstCamTransform[0:3, 3] = SiloToFirstCamTransform[0:3, 3] * 1000
        transdict = {'transformation': SiloToFirstCamTransform.tolist(),
                     'x_offset': round(VolumeSingleton.x_offset*1000), 'y_offset': round(VolumeSingleton.y_offset*1000), 'z_offset': round(VolumeSingleton.z_offset*1000),
                     'alpha': round(VolumeSingleton.alpha * 180/3.141592), 'beta': round(VolumeSingleton.beta * 180/3.141592), 'gamma': round(VolumeSingleton.gamma * 180/3.141592)}
        return jsonify(transdict)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)
