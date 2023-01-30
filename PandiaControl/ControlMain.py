from core import create_app
from waitress import serve
import socket
import json
import os.path

if __name__=='__main__':
    app = create_app()

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

    if forceLocalhost:
        print("Info: Forcing localhost")
        ControlAdress="127.0.0.1"
    else :
        ControlAdress=socket.gethostbyname(socket.gethostname() + ".local")
    ControlPort="5050"

    serve(app, host=ControlAdress, port=ControlPort, threads=8) #production server
