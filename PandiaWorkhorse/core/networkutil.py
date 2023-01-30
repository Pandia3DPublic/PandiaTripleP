import requests
import sys
from flask import json
import socket
import os.path

def requestControlNetworkConfig(address, port, timeout_=0.1, timeout_max=5):
    try:
        print(f"Searching Pandia Control at {address}")
        #blocking but okay!
        response = requests.get("http://{}:{}/getNetworkInfo".format(address, port), timeout=timeout_)
        if response.status_code == 200:
            print(f"Found Pandia Control at {address}")
            return response.json()
        else:
            print(f"Error in fetching NetworkInfo, status code {response.status_code}")
            return
    except requests.exceptions.ConnectionError:
        # In the event of a network problem (e.g. DNS failure, refused connection, etc), Requests will raise a ConnectionError exception.
        return
    except requests.exceptions.Timeout:
        # Maybe set up for a retry, or continue in a retry loop
        timeout_new = 2 * timeout_
        if (timeout_new < timeout_max):
            print("Connection timeout! Trying again...")
            return requestControlNetworkConfig(address, port, timeout_new, timeout_max)
        else:
            print("Connection timeout!")
            return
    except requests.exceptions.HTTPError as e:
        # rare invalid HTTP response
        raise SystemExit(e)
    except requests.exceptions.RequestException as e:
        # catastrophic error. bail.
        raise SystemExit(e)


def findControl(ControlPort, WorkhorseAddress):
    ControlAddress=""
    ControlHostname=""
    FoundControl=False
    #read json with control network config
    jsonPath = "ControlNetworkConfig.json"
    try:
        if os.path.isfile(jsonPath):
            with open(jsonPath, 'r') as f:
                data = json.load(f)
                ControlAddressJSON=data["ip"]
                ControlHostnameJSON=data["hostname"]
    except:
        print(f"Reading {jsonPath} failed!")

    # does control run on same system?
    if not FoundControl:
        if requestControlNetworkConfig(WorkhorseAddress, ControlPort, 2) :
            ControlAddress = WorkhorseAddress
            ControlHostname = socket.gethostname() + ".local"
            FoundControl = True

    # does hostname from json resolve?
    if not FoundControl:
        try:
            ControlAddress = socket.gethostbyname(ControlHostnameJSON)
            ControlHostname=ControlHostnameJSON
            # does control respond to ip address from hostname?
            if requestControlNetworkConfig(ControlAddress, ControlPort, 2) :
                FoundControl = True
        except:
            print(f"Info: Control Hostname '{ControlHostnameJSON}' does not resolve.")

    # does control respond to ip address from json?
    if not FoundControl:
        res=requestControlNetworkConfig(ControlAddressJSON, ControlPort, 2)
        if res :
            ControlAddress=res['ip']
            ControlHostname=res['hostname']
            FoundControl = True

    #lastly if still not found, do a network scan to find pandia control
    #todo make even more general, check subnet mask as the host range can vary in some networks
    if not FoundControl:
        print("######## Scanning Network for Pandia Control ########")
        network=WorkhorseAddress.rsplit(".", 1)[0]
        for host in range(1,255):
            adr=network+"."+str(host)
            res = requestControlNetworkConfig(adr, ControlPort)
            if res : #success
                ControlAddress=res['ip']
                if ControlAddress == WorkhorseAddress: #special case: control and workhorse are on the same pc
                    ControlHostname=res["hostname"]+".local"
                FoundControl=True
                break

    if not FoundControl:
        print("Error: Pandia Control not found. Check network settings, then try again.")
        sys.exit()

    # save new json to disk
    with open('ControlNetworkConfig.json', 'w') as f:
        json.dump({'hostname' : ControlHostname, 'ip' : ControlAddress}, f, indent=4)

    return ControlAddress