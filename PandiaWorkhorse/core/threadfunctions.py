import threading, time
from datetime import datetime
from core import VolumeSingleton, CameraList, logger
from flask import Blueprint, jsonify, request, Response
from . import volume as vol
# from .volume import VolumeTimestaps, VolumeData, n_VolumeChart, ConveyorBatchRunning, TotalVolumeFlow
import os
import json
import traceback
import requests

threadfunctions = Blueprint('threadfunctions', __name__)  # this actually gets registered

stopVolumeThreadBool = False
volumeThread = threading.Thread()

# volume config vars
combineFrames = False
n_record = 1
fetchBeltspeedTimeoutSeconds = 1

def readVolumeConfig():
    jsonPath = 'VolumeConfig.json'
    global combineFrames
    global n_record
    global fetchBeltspeedTimeoutSeconds
    try:
        if not os.path.isfile(jsonPath):
            print(f"{jsonPath} is missing!")
            return False
        with open(jsonPath, 'r') as f:
            jsonDict = json.load(f)
            combineFrames = jsonDict['combineFrames']
            n_record = jsonDict['n_record']
            fetchBeltspeedTimeoutSeconds = jsonDict['fetchBeltspeedTimeoutSeconds']
        # print("combineFrames: " + str(combineFrames))
        # print("n_record: " + str(n_record))
        # print("fetchBeltspeedTimeoutSeconds: " + str(fetchBeltspeedTimeoutSeconds))
        return True
    except Exception:
        print(f"Reading {jsonPath} failed!")
        logger.error("Exception:" + traceback.format_exc())
        return False


@threadfunctions.route('/startVolumeThread')
def startVolumeThread():
    readVolumeConfig()
    if not VolumeSingleton.SiloModelSet():
        return jsonify(Message='Model not set')
    global volumeThread
    if not volumeThread.is_alive():
        print("Starting Volume Measurement")
        volumeThread = threading.Thread(target=volume_thread_function)
        volumeThread.start()
    else:
        print("Volume thread already started")
    return jsonify(Message='OK')


def stopVolumeThreadFunction():
    global volumeThread
    if volumeThread.is_alive():
        print("Stopping Volume Measurement")
        global stopVolumeThreadBool
        stopVolumeThreadBool = True
        volumeThread.join()
        stopVolumeThreadBool = False

@threadfunctions.route('/stopVolumeThread')
def stopVolumeThread():
    stopVolumeThreadFunction()
    return jsonify(Message='OK')

@threadfunctions.route('/ThreadisRunning')
def ThreadisRunning():
    global volumeThread
    if volumeThread.is_alive():
        return jsonify(Message='OK')
    else:
        return jsonify(Message='Thread is currently not running')


def fetchBeltSpeed():
    global fetchBeltspeedTimeoutSeconds
    response = False
    errorstring = ""
    try:
        response = requests.post("http://{}".format(vol.BeltSpeedEndpointAddress),
                    auth=(vol.BeltSpeedEndpointUsername, vol.BeltSpeedEndpointPassword),
                    data='["'+str(vol.BeltSpeedEndpointPayload)+'"]',
                    headers={'Content-Type' : 'application/json'},
                    timeout=fetchBeltspeedTimeoutSeconds)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        errorstring = "Failed to access belt speed endpoint"
        return {"Error" : errorstring}

    if response and response.ok:
        try:
            v = -1
            t = -1
            jsonResponse = response.json()
            if 'readResults' in jsonResponse:
                for resultEntry in jsonResponse['readResults']:
                    if resultEntry['id'] == vol.BeltSpeedEndpointPayload:
                        v = resultEntry['v']
                        t = resultEntry['t']
            if v == -1 or t == -1:
                errorstring = "Invalid beltspeed json data from response."
                return {"Error" : errorstring}
            return {"v": v, "t": t}
        except Exception:
            logger.error("Exception:" + traceback.format_exc())
            errorstring="Failed to read beltspeed json from response"
            return {"Error" : errorstring}

def fetchAndUpdateBeltSpeed():
    global v_BeltspeedResponses
    responseData = fetchBeltSpeed()
    try:
        if "Error" in responseData:
            print("Warning: " + responseData["Error"])
            return False
        v_BeltspeedResponses.append(responseData)
        if len(v_BeltspeedResponses) > 1:
            dict_new = v_BeltspeedResponses[-1]
            dict_old = v_BeltspeedResponses[-2]
            if dict_new['v'] == dict_old['v'] + 1:
                VolumeSingleton.BeltSpeed = round(vol.BeltSpeedEndpointDistance / ((dict_new['t'] - dict_old['t']) / 1000), 5)
                return True
            else:
                return False
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return False

#this is only for the test endpoint button in gui
@threadfunctions.route('/TestEndpointButtonFunc', methods=['POST'])
def TestEndpointButtonFunc():
    try:
        responseData = fetchBeltSpeed()
        if "Error" in responseData:
            return jsonify(responseData)

        maxTime = int(request.get_json()['MaxWaitTime'])
        startTime = time.time()
        success = False
        while(not success and time.time() - startTime < maxTime):
            time.sleep(0.25)
            success = fetchAndUpdateBeltSpeed()

        global v_BeltspeedResponses
        v_BeltspeedResponses.clear()
        if success:
            return jsonify(BeltSpeed=VolumeSingleton.BeltSpeed)
        else:
            return jsonify(Error="Couldn't fetch a valid second value!")
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)



v_TotalVolumeFlow = []
v_Beltspeed = []
v_BeltspeedResponses = []
v_Density = []
v_ProductPresence = []
v_ProductMoving = []
v_MaxPointHeight = []
v_AvgPointHeight = []
def volume_thread_function():
    global stopVolumeThreadBool
    global CameraList
    global v_TotalVolumeFlow
    global v_Beltspeed
    global v_BeltspeedResponses
    global v_Density
    global v_ProductPresence
    global v_ProductMoving
    global v_MaxPointHeight
    global v_AvgPointHeight
    global combineFrames
    global n_record
    BackupCounter = 0
    backupThreshold = 50 #should be higher than vol.n_VolumeChart
    lastTimeStamp = 0
    while not stopVolumeThreadBool:
        time.sleep(0.5)
        CameraList[-1].clearBuffers()
        success = CameraList[-1].record(n_record)
        if not success:
            print("recording failed, starting anew")
            continue
        if combineFrames:
            try:
                CameraList[-1].combineRecordedFrames(VolumeSingleton.DirectionVector, VolumeSingleton.BeltSpeed);
            except:
                print("Warning: Combine frames is not yet implemented for this camera type")
        currentTime = datetime.now()
        vol.VolumeTimestaps.append(currentTime.isoformat(' '))
        vol.VolumeData.append(round(1000 * VolumeSingleton.getCurrentVolume(), 5))
        if vol.ConveyorBatchRunning and lastTimeStamp != 0:
            dt = currentTime.timestamp() - lastTimeStamp
            vol.TotalVolumeFlow = vol.TotalVolumeFlow + 1000 * VolumeSingleton.getVolumeIncrement(dt)
            if vol.UseBeltSpeedEndpoint:
                fetchAndUpdateBeltSpeed()
        v_TotalVolumeFlow.append(round(vol.TotalVolumeFlow, 5))
        lastTimeStamp = currentTime.timestamp() #s

        v_Beltspeed.append(round(VolumeSingleton.BeltSpeed, 5))
        v_Density.append(round(VolumeSingleton.DensityFactor, 5))
        if vol.VolumeData[-1] >= vol.ProductPresenceThreshold:
            v_ProductPresence.append(1)
        else:
            v_ProductPresence.append(0)
        if v_Beltspeed[-1] != 0 and v_ProductPresence[-1] == 1:
            v_ProductMoving.append(1)
        else:
            v_ProductMoving.append(0)
        v_MaxPointHeight.append(round(1000 * VolumeSingleton.CurrentVolumeMaxPointHeight, 5)) # in mm
        v_AvgPointHeight.append(round(1000 * VolumeSingleton.CurrentVolumeAvgPointHeight, 5)) # in mm

        BackupCounter += 1
        if BackupCounter >= backupThreshold:
            BackupCounter=0
            with open('VolumeLog.txt', 'a') as logfile: #open the logfile in APPEND mode, if not exists then make a new one
                # create a list of data tuples, all zipped lists need to be the same length, order of lists is important!
                tupleList = list(zip(
                    vol.VolumeTimestaps[-backupThreshold:], 
                    vol.VolumeData[-backupThreshold:],
                    v_ProductPresence[-backupThreshold:],
                    v_ProductMoving[-backupThreshold:],
                    v_Beltspeed[-backupThreshold:],
                    v_Density[-backupThreshold:],
                    v_TotalVolumeFlow[-backupThreshold:],
                    v_MaxPointHeight[-backupThreshold:],
                    v_AvgPointHeight[-backupThreshold:]
                    ))
                #write lines to file
                for tuple in tupleList:
                    logfile.write(' , '.join([str(elem) for elem in tuple]) + '\n')
                #clear the array execpt for a last few entries for dynamic website vis
                vol.VolumeData[:] = vol.VolumeData[-vol.n_VolumeChart:]
                vol.VolumeTimestaps[:] = vol.VolumeTimestaps[-vol.n_VolumeChart:]
                v_TotalVolumeFlow.clear()
                v_Beltspeed.clear()
                v_BeltspeedResponses.clear()
                v_Density.clear()
                v_ProductPresence.clear()
                v_ProductMoving.clear()
                v_MaxPointHeight.clear()
                v_AvgPointHeight.clear()