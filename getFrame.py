import numpy as np
import cv2
from matplotlib import pyplot as plt
from enum import Enum

class GetFrame:

    out_full = cv2.VideoWriter('full.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(3448,808))
    out_left = cv2.VideoWriter('left.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))
    out_right = cv2.VideoWriter('right.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))

    def getCamerasConnected():
        contCamera = 0
        while True:
            cap = cv2.VideoCapture(contCamera)
            if not cap.isOpened():
                break
            cap.release()
            contCamera += 1

        # Mostrar la cantidad de cámaras conectadas
        print(f"Se encontraron {contCamera} cámara(s) conectada(s).")
        return contCamera

    
    def connectToCamera(self,indexVideo,resolution = 1,fps = 30):

        cap = cv2.VideoCapture(indexVideo)

        if not cap.isOpened:
            print("Camera is not found")
            return None
        else:
            # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
            # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)
            # cap.set(cv2.CAP_PROP_FPS, 8)

            if (resolution == Resolutions.RES_2560x800.value):   
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
            elif (resolution == Resolutions.RES_3448x808.value):
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)

            cap.set(cv2.CAP_PROP_FPS, fps)
            return cap

    def decode(self,frame):

        height, width, _ = frame.shape
        mid = width // 2
        left_frame = frame[:, :mid, :]
        right_frame = frame[:, mid:, :]

        left_frame = cv2.resize(left_frame,(1264,800))
        right_frame = cv2.resize(right_frame,(1264,800))
        
        return (left_frame, right_frame)

    def getNewFrame(self,cap):

        ret, frame = cap.read()
        left,right = self.decode(frame)

        self.out_full.write(frame)
        self.out_left.write(left)
        self.out_right.write(right)

        return left,right
    
    def showPreviewRgb(self,frameLeft,frameRight):
        left = cv2.resize(frameLeft,(int(256*3),int(108*4.5)))
        right = cv2.resize(frameRight,(int(256*3),int(108*4.5)))
        cv2.imshow('RGB left', left)
        cv2.imshow('RGB right', right)

class Resolutions(Enum):          # TODO: implementar
    RES_3448x808 = 0
    RES_2560x800 = 1
