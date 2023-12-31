import subprocess, sys
from subprocess import Popen, PIPE
import win32com.client
from enum import Enum



class ControlCamera():


    def __init__(self):
        print("Control camera instantiate")

    def loadFirmwareCamera(self):

        pathLoader = "C:/Users/Patri/Downloads/ps5_camera/PS5_camera_files-main/OrbisEyeCameraFirmwareLoader.exe"

        result = str(subprocess.check_output(pathLoader))
        
        if( result.find("Firmware uploaded")):
            return True
        else:
            return False
        
    def getCameraStatus(self):
        try:
            statusCamera = StatusCamera.CAMERA_NOT_CONNECTED
            listDeviceId = []
            wmi = win32com.client.GetObject("winmgmts:")

            # get just a deviceId with VID: 05A9
            usbDevices = wmi.ExecQuery("SELECT * FROM Win32_PnPEntity WHERE PNPDeviceID LIKE '%VID_05A9%'")
            
            print("Scanning")

            if (len(usbDevices) == 0):
                print("Camera not found")
                return StatusCamera.CAMERA_NOT_CONNECTED
            
            for deviceId in usbDevices:
                print(deviceId.DeviceID)

                if( deviceId.DeviceID.find("VID_05A9&PID_0580") > 0):
                    print("Camera found -> pending load firmware")
                    return StatusCamera.CAMERA_CONNECTED_PENDING_FW
                elif( deviceId.DeviceID.find("VID_05A9&PID_058C") > 0):
                    print("Camera found-> with firmware")
                    return StatusCamera.COMERA_CONNECTED_OK

            return StatusCamera
        except Exception as error:
            print('error', error)



class StatusCamera(Enum):
    CAMERA_NOT_CONNECTED = 1
    CAMERA_CONNECTED_PENDING_FW = 2
    COMERA_CONNECTED_OK = 3

controlCamera = ControlCamera()
print("Modo de prueba")
# controlCamera.getCameraStatus()
# print(controlCamera.loadFirmwareCamera())


statusCamera = controlCamera.getCameraStatus()
if( statusCamera == StatusCamera.CAMERA_NOT_CONNECTED ):
    print("Camera not found")
elif( statusCamera == StatusCamera.CAMERA_CONNECTED_PENDING_FW ):
    print("Trying load firmware")
    if( controlCamera.loadFirmwareCamera() == True ):
        print("Camera connected!")
    else:
        print("Error trying connect")
else:
    print("Camera already connect")

