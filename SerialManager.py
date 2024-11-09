
import serial
import serial.tools.list_ports 
import threading
import re

def readNewData(ser,callback):
         
    pattern = re.compile(r"X:\s*(-?\d+\.\d+),Y:\s*(-?\d+\.\d+),Z:\s*(-?\d+\.\d+)")

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            match = pattern.search(line)
            
            if match:
                roll = float(match.group(1))
                pitch = float(match.group(2))
                yaw = float(match.group(3))
                callback((pitch, roll, yaw), False)
        except Exception:
            callback(None,None,None,True)
            break



class SerialManager:

    def connect(self,commPort,baudrate,callback):
        self.ser = serial.Serial()

        self.ser.port = commPort
        self.ser.baudrate = baudrate
        self.ser.open()

        if(self.ser.is_open):
            self.threading = threading.Thread(target=readNewData,args=(self.ser,callback))
            self.threading.daemon = True
            self.threading.start()
            return True
        else:
            return False

    
    def disconnect(self):
        self.serial.close()

    def getSerialComms(self):
        ports = serial.tools.list_ports.comports()
        listOfComms = []
        for port in ports:
            listOfComms.append(port.device)

        return listOfComms



