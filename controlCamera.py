import subprocess
from enum import Enum
import platform

VID_PS5 = "VID_05A9"
WINDOWS_PLATFORM_NAME = 'Windows'
LINUX_PLATFORM_NAME = 'Linux'


class ControlCamera():

    def __init__(self):
        print('Running at:' + platform.system())

    def loadFirmwareCamera(self):                       # TODO: deberia funcionar todo con pyusb, pero en windows no encuentra dispositivos
        
        if (platform.system() == WINDOWS_PLATFORM_NAME): 
            self.__loadFirmCameraInWindows()
        elif(platform.system() == LINUX_PLATFORM_NAME):
            self.__loadFirmCameraInLinux()
        else:
            print('Running at unknown OS')
            return False
        
    def getCameraStatus(self):

        if (platform.system() == WINDOWS_PLATFORM_NAME): 
            return self.__getStatusCameraInWindows()
        
        elif(platform.system() == LINUX_PLATFORM_NAME):
            return self.__getStatusCameraInLinux()


    def __getStatusCameraInWindows(self):
        try:
            import win32com.client
            wmi = win32com.client.GetObject("winmgmts:")
            usbDevices = wmi.ExecQuery(f"SELECT * FROM Win32_PnPEntity WHERE PNPDeviceID LIKE '%{VID_PS5}%'")

            if (len(usbDevices) == 0):
                return StatusCamera.CAMERA_NOT_CONNECTED 
            
            for deviceId in usbDevices:
                if( deviceId.DeviceID.find("VID_05A9&PID_0580") > 0):
                    return StatusCamera.CAMERA_CONNECTED_PENDING_FW
                elif( deviceId.DeviceID.find("VID_05A9&PID_058C") > 0):
                    return StatusCamera.CAMERA_CONNECTED_OK
        except Exception as error:
            print('error', error)
            return StatusCamera.CAMERA_NOT_CONNECTED
        
    def __getStatusCameraInLinux(self):

        import usb.core
        import usb.util

        try:     
            devCamWithoutFirm = usb.core.find(idVendor=0x05a9, idProduct=0x0580) 
            devCamWithFirm = usb.core.find(idVendor=0x05a9, idProduct=0x058c) 
            if devCamWithoutFirm is not None:
                return StatusCamera.CAMERA_CONNECTED_PENDING_FW
            elif devCamWithFirm is not None:
                return StatusCamera.CAMERA_CONNECTED_OK
            else:
                return StatusCamera.CAMERA_NOT_CONNECTED 

        except Exception as error:
            return StatusCamera.CAMERA_NOT_CONNECTED
        
    def __loadFirmCameraInWindows(self):
        pathLoader = "FirmwareLoader/PS5_camera_files-main/OrbisEyeCameraFirmwareLoader.exe"
        result = str(subprocess.check_output(pathLoader))
        
        if( result.find("Firmware uploaded") > 0):
            return True
        else:
            return False

    def __loadFirmCameraInLinux(self):
        import usb.core
        import usb.util
        dev = usb.core.find(idVendor=0x05a9, idProduct=0x0580)
        if dev is not None:
            return True

        # set the active configuration. With no arguments, the first
        # configuration will be the active one
        dev.set_configuration()

        # helper function for chunking a file
        def read_chunks(infile, chunk_size):
            while True:
                chunk = infile.read(chunk_size)
                if chunk:
                    yield chunk
                else:
                    return

        chunk_size=512
        index=0x14
        value=0

        firmware=open("firmware_V2.bin","rb")

        # transfer 512b chunks of the firmware
        for chunk in read_chunks(firmware, chunk_size):
            ret = dev.ctrl_transfer(0x40, 0x0, value, index, chunk)
            value+=chunk_size
            if value>=65536:
                value=0
                index+=1
            if len(chunk)!=ret:
                print("sent %d/%d bytes" % (ret,len(chunk)))

        # command reboots device with new firmware and product id
        try:
            ret = dev.ctrl_transfer(0x40, 0x0, 0x2200, 0x8018, [0x5b])
        except:
            return True
            print('PS4 camera firmware uploaded and device reset')

        

class StatusCamera(Enum):
    CAMERA_NOT_CONNECTED = 1
    CAMERA_CONNECTED_PENDING_FW = 2
    CAMERA_CONNECTED_OK = 3
