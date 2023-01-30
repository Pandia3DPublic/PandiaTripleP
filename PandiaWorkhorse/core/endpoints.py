from flask import Blueprint, make_response
from core import VolumeSingleton
from . import volume as vol
from . import threadfunctions as tf

# here are the production endpoints for the pandia peak software

# Pandia 3D provided Endpoints 
#     Start Volume Measurement 
#     Stop Volume Measurement 
#     Get Current Volume. 
#     Start Measuring Total Volume Flow. 
#     Pause Measuring Total Volume Flow. 
#     Reset Total Volume Flow. 
#     Get Current Total Volume Flow 
#  Endpoints provided for Pandia 3D  
#     Get Current Belt Speed. 

endpoints = Blueprint('endpoints', __name__)  # this actually gets registered

# VOLUME MEASUREMENT

@endpoints.route('/v1/startVolumeMeasurement')
def startVolumeMeasurement():
    responseString = "OK"
    if not VolumeSingleton.SiloModelSet():
        responseString = "Failed due to incomplete system setup"
    else:
        r = tf.startVolumeThread()
        if r.status_code != 200:
            responseString = "Failed due to network error"
    response = make_response("{" + responseString + "}")
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@endpoints.route('/v1/stopVolumeMeasurement')
def stopVolumeMeasurement():
    responseString = "OK"
    if not VolumeSingleton.SiloModelSet():
        responseString = "Failed due to incomplete system setup"
    else:
        r = tf.stopVolumeThread()
        if r.status_code != 200:
            responseString = "Failed due to network error"
    response = make_response("{" + responseString + "}")
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@endpoints.route('/v1/getCurrentVolume')
def getCurrentVolume():
    responseString = ""
    if len(vol.VolumeData) != 0 and tf.volumeThread.is_alive():
        volume = vol.VolumeData[-1]
        responseString="{:.5f}".format(volume)
    response = make_response("{" + responseString + "}")
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

# VOLUME FLOW MEASUREMENT

@endpoints.route('/v1/startVolumeFlowMeasurement')
def startVolumeFlowMeasurement():
    responseString = "OK"
    if not VolumeSingleton.SiloModelSet():
        responseString = "Failed due to incomplete system setup"
    else:
        r = vol.startConveyorBatch()
        if r.status_code != 200:
            responseString = "Failed due to network error"
    response = make_response("{" + responseString + "}")
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@endpoints.route('/v1/pauseVolumeFlowMeasurement')
def pauseVolumeFlowMeasurement():
    responseString = "OK"
    if not VolumeSingleton.SiloModelSet():
        responseString = "Failed due to incomplete system setup"
    else:
        r = vol.stopConveyorBatch()
        if r.status_code != 200:
            responseString = "Failed due to network error"
    response = make_response("{" + responseString + "}")
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@endpoints.route('/v1/resetVolumeFlowMeasurement')
def resetVolumeFlowMeasurement():
    responseString = "OK"
    if not VolumeSingleton.SiloModelSet():
        responseString = "Failed due to incomplete system setup"
    else:
        r = vol.resetConveyorBatch()
        if r.status_code != 200:
            responseString = "Failed due to network error"
    response = make_response("{" + responseString + "}")
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@endpoints.route('/v1/getCurrentVolumeFlow')
def getCurrentVolumeFlow():
    responseString = ""
    if vol.ConveyorBatchRunning:
        responseString = "{:.5f}".format(vol.TotalVolumeFlow)
    response = make_response("{" + responseString + "}")
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response
