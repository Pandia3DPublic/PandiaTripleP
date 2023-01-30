from core import create_app
from core.networkutil import findControl
from core.utils import buildVolumeLogtxt, WorkhorseCount
from waitress import serve
from requests import Session
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
import socket
import json
import os.path

if __name__ == '__main__':
    app = create_app()
    buildVolumeLogtxt()

    #read config and set variables
    jsonPath = 'config.json'
    jsonData = {}
    try:
        if os.path.isfile(jsonPath):
            with open(jsonPath, 'r') as f:
                jsonData = json.load(f)
        else:
            print(f"{jsonPath} is missing!")
    except:
        print(f"Reading {jsonPath} failed!")
    forceLocalhost = False  # for debug
    if 'forceLocalhost' in jsonData:
        forceLocalhost = jsonData['forceLocalhost']
    
    ####################### workhorse ip address and port ######################
    if forceLocalhost:
        print("Info: Forcing localhost")
        WorkhorseAddress = "127.0.0.1"
    else:
        WorkhorseAddress = socket.gethostbyname(socket.gethostname() + ".local")
    WorkhorsePort = 5000 + WorkhorseCount()

    ####################### control ip address and port ######################
    ControlPort="5050"
    #localhost for debug
    if (forceLocalhost):
        ControlAddress = "127.0.0.1"
    else:
        ControlAddress = findControl(ControlPort, WorkhorseAddress)

    session = Session()
    retry = Retry(connect=3, backoff_factor=0.5)
    adapter = HTTPAdapter(max_retries=retry)
    session.mount('http://', adapter)
    session.mount('https://', adapter)
    session.post("http://{}:{}/addNewWorkhorseToDB".format(ControlAddress, ControlPort), data = '{}:{}'.format(WorkhorseAddress, WorkhorsePort), timeout=5.0)
    
    serve(app, host=WorkhorseAddress, port=WorkhorsePort, threads=6) #production server


