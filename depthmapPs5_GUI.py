# --------------------------------
# Importamos librearías a utilizar
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.Qt import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtCore import QTimer, Qt

import sys
import numpy as np
import logging
import time
import cv2
from reactivex import create
from reactivex import operators as ops


from Utils.utils import CameraStatusUi
from controlCamera import ControlCamera 
from controlCamera import StatusCamera
from getFrame import GetFrame
from getFrame import Resolutions
from depthMapProcessor import DepthMapProcessor

# --------------------------------
# Creamos la Clase de la Interfaz


class UIManager(QMainWindow):

    statusCameraUi = 0
    cameraResolution = Resolutions.RES_2560x800
    cameraFps = 8

    def __init__(self,_controlCamera,_getFrame,_depthMapProcessor):
        # --------------------------------
        # Esta línea es importante para que funcione el programa
        QMainWindow.__init__(self)
        super(QMainWindow, self).__init__(parent=None)

        self.controlCamera = controlCamera
        self.getFrame = _getFrame
        self.depthMapProcessor = _depthMapProcessor
        # --------------------------------
        # Propiedades de la ventana
        # self.setFixedHeight(650)
        # self.setFixedWidth(800)
        self.setWindowTitle("Ps5 Camera GUI")
        self.setGrid()
        self.setVideoControlPanel()
        self.setVideoCameraPanel()
        # self.applyStyles()

        self.updateCameraStatus()
        self.setObserverGetFrames()

    def setGrid(self):
        # --------------------------------
        # Pestaña Principal
        self.MainWidget = QWidget()
        self.setCentralWidget(self.MainWidget)
        self.Grid_layout = QGridLayout()
        self.Grid_layout.setSpacing(2)
        self.MainWidget.setLayout(self.Grid_layout)
        
    def setVideoControlPanel(self):

         # Cuadro de control de camara
        self.videoControlLayout = QVBoxLayout()
        self.videoControlLayout.setSpacing(10)
        self.videoControlLayout.setAlignment(Qt.AlignTop)

        self.gb_videoControl = QGroupBox("Control camera", parent=self)
        self.gb_videoControl.setLayout(self.videoControlLayout)

        self.Grid_layout.addWidget(self.gb_videoControl, 0, 1, 1, 1)

        # Label de status
        self.labelStatus = QLabel(parent=self.gb_videoControl)
        self.labelStatus.setText("Desconectado")
        # self.labelStatus.setFixedHeight(45)
        self.labelStatus.setAlignment(Qt.AlignCenter)
        
        lbl_statusStyle = """
            QLabel
            {
                background-color: white; 
                font-size: 18px;
                color: black;
                font-family: "Consolas";
                font-weight: bold; 
                border: 3px groove #212121;
                border-radius: 5px;
                padding: 15px;
            }
        """
        self.labelStatus.setStyleSheet(lbl_statusStyle)

        self.videoControlLayout.addWidget(self.labelStatus)

        # Boton para la carga de firmware
        self.btnConnectCamera = QPushButton(text="Conectar camara")
        self.btnConnectCamera.setEnabled(True)
        self.btnConnectCamera.clicked.connect(self.actionCameraBtn)
        self.videoControlLayout.addWidget(self.btnConnectCamera)

        # Combo box para seleccionar la resolucion de la camara
        self.comboResolution= QComboBox()
        resolutions = [
            "2560x800 8fps",
            "2560x800 30fps",
            "2560x800 60fps",
            "3448x808 8fps",
            "3448x808 30fps",
            "3448x808 60fps",
        ]

        self.comboResolution.addItems(resolutions)
        self.comboResolution.currentIndexChanged.connect(self.changeResolution)
        self.comboResolution.setEnabled(False)
        self.videoControlLayout.addWidget(self.comboResolution)

        # Boton para tomar una captura
        self.btnTakeSnapshot = QPushButton(text="Take snapshot")
        self.btnTakeSnapshot.setEnabled(False)
        # self.btnTakeSnapshot.clicked.connect(self.EMER_STOP)
        self.videoControlLayout.addWidget(self.btnTakeSnapshot)

    def setVideoCameraPanel(self):

        # logging.warning('This is a warning message')

        # Cuadro de frames video
        self.videoLayout = QGridLayout()
        self.videoLayout.setSpacing(2)

        self.gb_video = QGroupBox("View Camera", parent=self)
        self.gb_video.setLayout(self.videoLayout)

        self.Grid_layout.addWidget(self.gb_video, 0, 0, 1, 1)

        self.video_labelL = QLabel("No video")             # Etiqueta para mostrar el video
        self.video_labelR = QLabel("No video")

        self.video_labelR.setFixedWidth(320)
        self.video_labelR.setFixedHeight(200)
        self.video_labelL.setFixedWidth(320)
        self.video_labelL.setFixedHeight(200)

        self.video_labelDepthMap = QLabel("No video")

        self.video_labelDepthMap.setFixedWidth(660)
        self.video_labelDepthMap.setFixedHeight(410)

        lbl_videoStyle = """
            QLabel
            {
                background-color: gray; 
                font-size: 20px;
                color: white;
                font-family: "Consolas";
                font-weight: bold; 
                border: 0px groove #212121;
                border-radius: 5px;
            }
        """

        self.video_labelL.setStyleSheet(lbl_videoStyle)
        self.video_labelR.setStyleSheet(lbl_videoStyle)
        self.video_labelDepthMap.setStyleSheet(lbl_videoStyle)

        self.video_labelL.setAlignment(Qt.AlignCenter)
        self.video_labelR.setAlignment(Qt.AlignCenter)
        self.video_labelDepthMap.setAlignment(Qt.AlignCenter)

        self.videoLayout.addWidget(self.video_labelL, 0, 0, 1, 1)
        self.videoLayout.addWidget(self.video_labelR, 0, 1, 1, 1)
        self.videoLayout.addWidget(self.video_labelDepthMap, 1, 0, 1, 2)
        self.setLayout(self.videoLayout)

    def updateFrames(self,frames):
        frameR,frameL = frames

        frame = cv2.cvtColor(frameL, cv2.COLOR_BGR2RGB)                            # Convertir el fotograma a RGB
        self.video_labelL.setPixmap(self.convertFrameToQt(frame))                  # Mostrar el fotograma en el QLabel

        frame = cv2.cvtColor(frameR, cv2.COLOR_BGR2RGB)                            # Convertir el fotograma a RGB
        self.video_labelR.setPixmap(self.convertFrameToQt(frame))                  # Mostrar el fotograma en el QLabel

        depthMap,depthMapColor,points = self.depthMapProcessor.processFrame(frameL,frameR)

        frame = depthMapColor
        frame = cv2.cvtColor(depthMapColor, cv2.COLOR_BGR2RGB)                     # Convertir el fotograma a RGB
        self.video_labelDepthMap.setPixmap(self.convertFrameToQt(frame,self.video_labelDepthMap.width(),self.video_labelDepthMap.height()))           # Mostrar el fotograma en el QLabel
        
    def convertFrameToQt(self,frame,scaledWidth = 300, scaledHeight = 240):
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        convert_to_qt_format = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(convert_to_qt_format)
        pixmap = pixmap.scaled(scaledWidth,scaledHeight, Qt.KeepAspectRatio)

        return pixmap

    def closeEvent(self, event):
        self.video_capture.release()  # Liberar la captura de la cámara al cerrar la aplicación

    def applyStyles(self):
        # --------------------------------
        # Se configuran los estilos de los widgets con CSS

        # --------------------------------
        # Estilo para los GroupBoxes
        gb_estilo = """ 
            QGroupBox {
            font-size: 12px;  
            color: #4E342E;
            padding: 2px;
            border: 1px solid #D0D3D4; 
            font-family: "Consolas";  
            }
            """

        # --------------------------------
        # Estilo para los Botones
        btn_estilo = """
            QPushButton {  
            font-size: 12px; 
            font-family: "Consolas";
            font-weight: bold;  }
            QPushButton::disabled{
            color: #F4F6F6; 
            background-color: #F4F6F6; 
            border: 0px solid #424949;
            font-size: 12px;  
            }
            """
        # --------------------------------
        # Estilo para la lista de opciones
        list_options = """
            QComboBox {  
            font-size: 12px;
            font-family: "Consolas";  } 
            """

        # --------------------------------
        # Estilo para los Labels
        lbl_estilo1 = """
            QLabel
            {
            background-color: black; 
            font-size: 18px;
            color: #00FF99;
            font-family: "Consolas";
            font-weight: bold; 
            border: 3px groove #212121;
            border-radius: 5px;
            }"""
        # --------------------------------
        # Estilo para los Labels
        lbl_estilo2 = """
            QLabel
            {
            background-color: #F2F4F4; 
            font-size: 15px;
            color:  black;
            font-family: "Consolas";  
            font-weight: bold; 
            border: 0px;
            padding: 2px;
            border-radius: 5px;
            }"""
        # --------------------------------
        # Estilo para los inputs
        inpt_estilo = """
            QLineEdit
            {
            background-color: #F2F3F4; 
            font-size: 18px;
            color: #1A5276;
            font-family: "Consolas"; 
            font-weight: bold; 
            padding: 1px; 
            border-left: 3px groove #515A5A;
            }
            
            """

        # --------------------------------
        # Se aplican los estilos
        self.inpt_Px.setStyleSheet(inpt_estilo)
        self.inpt_Py.setStyleSheet(inpt_estilo)
        self.inpt_Pz.setStyleSheet(inpt_estilo)
        self.inpt_Pxi.setStyleSheet(inpt_estilo)
        self.inpt_Pyi.setStyleSheet(inpt_estilo)
        self.inpt_Pzi.setStyleSheet(inpt_estilo)
        self.inpt_Pxf.setStyleSheet(inpt_estilo)
        self.inpt_Pyf.setStyleSheet(inpt_estilo)
        self.inpt_Pzf.setStyleSheet(inpt_estilo)
        self.inpt_points_qty.setStyleSheet(inpt_estilo)

        self.inpt_Px.setFixedWidth(150)
        self.inpt_Py.setFixedWidth(150)
        self.inpt_Pz.setFixedWidth(150)
        self.inpt_Pxi.setFixedWidth(150)
        self.inpt_Pyi.setFixedWidth(150)
        self.inpt_Pzi.setFixedWidth(150)
        self.inpt_Pxf.setFixedWidth(150)
        self.inpt_Pyf.setFixedWidth(150)
        self.inpt_Pzf.setFixedWidth(150)
        self.inpt_points_qty.setFixedWidth(150)

        self.inptQ1pos.setStyleSheet(inpt_estilo)
        self.inptQ2pos.setStyleSheet(inpt_estilo)
        self.inptQ3pos.setStyleSheet(inpt_estilo)

        self.lbl_angQ1.setStyleSheet(lbl_estilo1)
        self.lbl_angQ2.setStyleSheet(lbl_estilo1)
        self.lbl_angQ3.setStyleSheet(lbl_estilo1)
        self.lbl_Q1.setStyleSheet(lbl_estilo2)
        self.lbl_Q2.setStyleSheet(lbl_estilo2)
        self.lbl_Q3.setStyleSheet(lbl_estilo2)
        self.lbl_Px.setStyleSheet(lbl_estilo2)
        self.lbl_Py.setStyleSheet(lbl_estilo2)
        self.lbl_Pz.setStyleSheet(lbl_estilo2)
        self.lbl_Pxi.setStyleSheet(lbl_estilo2)
        self.lbl_Pyi.setStyleSheet(lbl_estilo2)
        self.lbl_Pzi.setStyleSheet(lbl_estilo2)
        self.lbl_Pxf.setStyleSheet(lbl_estilo2)
        self.lbl_Pyf.setStyleSheet(lbl_estilo2)
        self.lbl_Pzf.setStyleSheet(lbl_estilo2)
        self.lbl_points_qty.setStyleSheet(lbl_estilo2)
        self.lbl_Pinicial.setStyleSheet(lbl_estilo2)
        self.lbl_Pfinal.setStyleSheet(lbl_estilo2)
        self.lbl_Q1value.setStyleSheet(lbl_estilo2)
        self.lbl_Q2value.setStyleSheet(lbl_estilo2)
        self.lbl_Q3value.setStyleSheet(lbl_estilo2)

        self.lbl_angQ1.setFixedHeight(40)
        self.lbl_angQ2.setFixedHeight(40)
        self.lbl_angQ3.setFixedHeight(40)
        self.lbl_Q1.setFixedHeight(20)
        self.lbl_Q2.setFixedHeight(20)
        self.lbl_Q3.setFixedHeight(20)
        self.lbl_Px.setFixedHeight(20)
        self.lbl_Py.setFixedHeight(20)
        self.lbl_Pz.setFixedHeight(20)
        self.lbl_Pxi.setFixedHeight(20)
        self.lbl_Pyi.setFixedHeight(20)
        self.lbl_Pzi.setFixedHeight(20)
        self.lbl_Pxf.setFixedHeight(20)
        self.lbl_Pyf.setFixedHeight(20)
        self.lbl_Pzf.setFixedHeight(20)
        self.lbl_points_qty.setFixedHeight(20)
        self.lbl_Pinicial.setFixedHeight(20)
        self.lbl_Pfinal.setFixedHeight(20)
        self.lbl_Q1value.setFixedHeight(20)
        self.lbl_Q2value.setFixedHeight(20)
        self.lbl_Q3value.setFixedHeight(20)

        self.combo_box_speed.setStyleSheet(list_options)
        self.combo_box_def_position.setStyleSheet(list_options)
        """
        self.gb_Commands.setStyleSheet(gb_estilo)
        self.gb_Inputs.setStyleSheet(gb_estilo)
        self.gb_JPositions.setStyleSheet(gb_estilo)
        self.gb_Movement.setStyleSheet(gb_estilo)
        self.gb_speed.setStyleSheet(gb_estilo)"""

        self.btnConnectCamera.setStyleSheet(btn_estilo)
        self.btn_lineal_move.setStyleSheet(btn_estilo)
        self.btnTakeSnapshot.setStyleSheet(btn_estilo)
        self.btn_Move.setStyleSheet(btn_estilo)
        self.btn_MovetoArt.setStyleSheet(btn_estilo)

        self.btnConnectCamera.setFixedHeight(20)
        self.btn_lineal_move.setFixedHeight(20)
        self.btnTakeSnapshot.setFixedHeight(20)
        self.btn_Move.setFixedHeight(20)
        self.btn_MovetoArt.setFixedHeight(20)
        self.btn_MovetoArt.setFixedWidth(100)

        self.setStyleSheet("background-color:#F4F6F6;")

    def startVideoStream(self):
        try:
            self.getFrame.startStream()
        except Exception as error :
            print('error', error)
            return False
        else:
            return True

    def stopVideoStream(self):
        print("stopStream")
        try:
            self.getFrame.stopStream()
        except Exception as error :
            print('error', error)
        
        # subscription.dispose()

    def setObserverGetFrames(self):
        getFrameSubject = self.getFrame.getSubjectGetFrame()

        self.subscription = getFrameSubject.pipe(
            ops.map(lambda frames: frames),
        ).subscribe(
            on_next=lambda frames: self.updateFrames(frames),
            on_error=lambda e: print(f"Error: {e}"),
            on_completed=lambda: print("Stream completado"),
        )
        print("setObserverGetFrames")

    def changeResolution(self,index):                   # optimizar
        print("index: ",index)
        if (index == 0):
            self.cameraResolution = Resolutions.RES_2560x800
            self.cameraFps = 8
        elif (index == 1):
            self.cameraResolution = Resolutions.RES_2560x800
            self.cameraFps = 30
        elif (index == 2):
            self.cameraResolution = Resolutions.RES_2560x800
            self.cameraFps = 60
        elif (index == 3):
            self.cameraResolution = Resolutions.RES_3448x808
            self.cameraFps = 8
        elif (index == 4):
            self.cameraResolution = Resolutions.RES_3448x808
            self.cameraFps = 30
        elif (index == 5):
            self.cameraResolution = Resolutions.RES_3448x808
            self.cameraFps = 60

        self.stopVideoStream()
        print("Set camera resolution: ",self.cameraResolution)
        getFrame.setCameraResolution( self.cameraResolution, self.cameraFps)       # TODO: deberia ser self
        self.startVideoStream()

    def setStatusCameraUi(self,status: CameraStatusUi):
        self.statusCameraUi = status
        self.labelStatus.setText(status.value)

        if (status == CameraStatusUi.WAITING_FW):
            self.btnConnectCamera.setText("Cargar FW")
        elif (status == CameraStatusUi.CONNECTED):
            self.btnConnectCamera.setText("Iniciar video")
        elif (status == CameraStatusUi.NOT_FOUND):
            self.btnConnectCamera.setText("Buscar camara")
        elif (status == CameraStatusUi.STREAM_RUNNING):
            self.btnConnectCamera.setText("Parar stream")

        self.comboResolution.setEnabled(status == CameraStatusUi.STREAM_RUNNING)
    
    def updateCameraStatus(self):
        statusCameraHardware = self.controlCamera.getCameraStatus()
        if( statusCameraHardware == StatusCamera.CAMERA_NOT_CONNECTED ):
            self.setStatusCameraUi(CameraStatusUi.NOT_FOUND)
        elif( statusCameraHardware == StatusCamera.CAMERA_CONNECTED_PENDING_FW ):
            self.setStatusCameraUi(CameraStatusUi.WAITING_FW)
        elif( statusCameraHardware == StatusCamera.CAMERA_CONNECTED_OK ):
            self.setStatusCameraUi( CameraStatusUi.CONNECTED)
    
    def actionCameraBtn(self):
        if (self.statusCameraUi == CameraStatusUi.WAITING_FW):
            self.loadFirmware()
        elif (self.statusCameraUi == CameraStatusUi.CONNECTED):
            self.setStatusCameraUi(CameraStatusUi.STREAM_INIT)
            if (self.startVideoStream()):
                self.setStatusCameraUi(CameraStatusUi.STREAM_RUNNING)
            else:
                self.setStatusCameraUi(CameraStatusUi.ERROR)
        elif (self.statusCameraUi == CameraStatusUi.NOT_FOUND):
            self.updateCameraStatus()
        elif (self.statusCameraUi == CameraStatusUi.STREAM_RUNNING):
            self.setStatusCameraUi(CameraStatusUi.CONNECTED)                    # vuelvo al estado de connected
            self.stopVideoStream()

    def loadFirmware(self):
        if( self.controlCamera.loadFirmwareCamera() == True ):
            self.setStatusCameraUi(CameraStatusUi.CONNECTED)
        else:
            self.setStatusCameraUi( CameraStatusUi.ERROR)

# --------------------------------
# Se ejecuta la interfaz

if __name__ == "__main__":
    App = QApplication(sys.argv)
    App.setStyle("Fusion")

    controlCamera = ControlCamera()
    getFrame = GetFrame('VID_05A9')

    npRet = np.load('DepthParams/param_ret.npy')
    npK = np.load('DepthParams/param_K.npy')
    npDist = np.load('DepthParams/param_dist.npy')
    depthMapProcessor = DepthMapProcessor(npRet,npK,npDist)

    w = UIManager(controlCamera,getFrame,depthMapProcessor)

    w.show()  # Mostramos la ventana

    sys.stdout.flush()
    sys.exit(App.exec_())
