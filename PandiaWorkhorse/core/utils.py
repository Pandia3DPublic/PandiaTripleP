from pathlib import Path
import psutil


#build VolumeLog.txt if it doesnt exist yet
def buildVolumeLogtxt():
    fle = Path('VolumeLog.txt')
    fle.touch(exist_ok=True)

def WorkhorseCount():
    count =0
    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            if proc.name()=='startWorkhorse.':
                count += 1
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return count

def getCameraIndex(CameraList, SerialNumber):
    index = -1
    for i, camera in enumerate(CameraList):
        if camera.SerialNumber == SerialNumber:
            index = i
            break
    if index == -1:
        pass
        # print(f"Warning: Camera '{SerialNumber}' not found in CameraList. Using last in list.")
    return index