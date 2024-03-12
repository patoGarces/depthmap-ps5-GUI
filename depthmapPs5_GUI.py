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

        self.controlCamera = _controlCamera
        self.getFrame = _getFrame
        self.depthMapProcessor = _depthMapProcessor
        # --------------------------------
        # Propiedades de la ventana
        # self.setFixedHeight(650)
        # self.setFixedWidth(800)
        self.setWindowTitle("Ps5 Camera GUI")
        self.setGrid()
        self.setMenuControlPanel()
        self.setVideoCameraView()
        self.applyStyles()

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

    def setMenuControlPanel(self):
        controlLayout = QVBoxLayout()
        controlLayout.setAlignment(Qt.AlignTop)

        # Cuadro de control de cámara arriba
        videoControlLayout = QVBoxLayout()
        videoControlLayout.setSpacing(10)
        videoControlLayout.setAlignment(Qt.AlignTop)

        self.gb_videoControl = QGroupBox("Control de cámara", parent=self)
        self.gb_videoControl.setLayout(videoControlLayout)
        self.gb_videoControl.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)

        # Label de status
        self.labelStatus = QLabel(parent=self.gb_videoControl)
        self.labelStatus.setText("Desconectado")
        self.labelStatus.setAlignment(Qt.AlignCenter)

        videoControlLayout.addWidget(self.labelStatus)

        # Boton para conectar la camara
        self.btnConnectCamera = QPushButton(text="Conectar cámara")
        self.btnConnectCamera.setEnabled(True)
        self.btnConnectCamera.clicked.connect(self.actionCameraBtn)
        videoControlLayout.addWidget(self.btnConnectCamera)

        # Combo box para seleccionar la resolución de la cámara
        self.comboResolution = QComboBox()
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
        videoControlLayout.addWidget(self.comboResolution)

        # Cuadro de control de DepthMap abajo
        depthControlLayout = QVBoxLayout()
        depthControlLayout.setSpacing(10)
        depthControlLayout.setAlignment(Qt.AlignTop)

        self.gb_depthControl = QGroupBox("Control de DepthMap", parent=self)
        self.gb_depthControl.setLayout(depthControlLayout)
        self.gb_depthControl.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)

        # Slider para ajustar el mix entre imagen RGB y depthMap
        self.labelDepthValue = QLabel("Depth mixer: 0.50", self)

        self.sliderMixerDepth = QSlider(Qt.Horizontal)
        self.sliderMixerDepth.setMinimum(1)
        self.sliderMixerDepth.setMaximum(100)
        self.sliderMixerDepth.setValue(50)
        self.sliderMixerDepth.setTickPosition(QSlider.TicksBelow)
        self.sliderMixerDepth.setTickInterval(10)
        self.sliderMixerDepth.valueChanged.connect(self.depthMixerChange)
        
        depthControlLayout.addWidget(self.labelDepthValue)
        depthControlLayout.addWidget(self.sliderMixerDepth)

        # Slider para ajustar blockSize
        self.labelBlockSize = QLabel("Block size: 5", self)

        self.sliderBlockSize = QSlider(Qt.Horizontal)
        self.sliderBlockSize.setMinimum(1)
        self.sliderBlockSize.setMaximum(20)
        self.sliderBlockSize.setValue(5)
        self.sliderBlockSize.setTickPosition(QSlider.TicksBelow)
        self.sliderBlockSize.setTickInterval(1)
        self.sliderBlockSize.valueChanged.connect(self.blockSizeChange)
        
        depthControlLayout.addWidget(self.labelBlockSize)
        depthControlLayout.addWidget(self.sliderBlockSize)

        # Slider para ajustar winsize
        self.labelWinSize = QLabel("window size: 5", self)

        self.sliderWinSize = QSlider(Qt.Horizontal)
        self.sliderWinSize.setMinimum(1)
        self.sliderWinSize.setMaximum(20)
        self.sliderWinSize.setValue(5)
        self.sliderWinSize.setTickPosition(QSlider.TicksBelow)
        self.sliderWinSize.setTickInterval(1)
        self.sliderWinSize.valueChanged.connect(self.winSizeChange)
        
        depthControlLayout.addWidget(self.labelWinSize)
        depthControlLayout.addWidget(self.sliderWinSize)

        # Agregar ambos grupos de controles al QVBoxLayout principal
        controlLayout.addWidget(self.gb_videoControl, alignment=Qt.AlignTop)
        controlLayout.addWidget(self.gb_depthControl, alignment=Qt.AlignTop)

        # Agregar el QVBoxLayout al GridLayout
        self.Grid_layout.addLayout(controlLayout, 0, 1, 1, 1)

    def setVideoCameraView(self):

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

        self.video_labelL.setAlignment(Qt.AlignCenter)
        self.video_labelR.setAlignment(Qt.AlignCenter)
        self.video_labelDepthMap.setAlignment(Qt.AlignCenter)

        self.videoLayout.addWidget(self.video_labelL, 0, 0, 1, 1)
        self.videoLayout.addWidget(self.video_labelR, 0, 1, 1, 1)
        self.videoLayout.addWidget(self.video_labelDepthMap, 1, 0, 1, 2)
        self.setLayout(self.videoLayout)

    def applyStyles(self):

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

        styleSliders = """
            QSlider {
                background-color: transparent;  /* Fondo transparente */
            }

            QSlider::groove:horizontal {
                border: 1px solid #7B7B7B;  /* Borde gris */
                height: 4px;  /* Altura del slider */
                background: #E0E0E0;  /* Fondo gris claro */
                margin: 0px;
            }

            QSlider::handle:horizontal {
                background: #BDBDBD;  /* Fondo gris más oscuro */
                border: 1px solid #7B7B7B;  /* Borde gris */
                width: 20px;  /* Ancho del control deslizante */
                margin: -8px 0; /* Margen para centrar el control deslizante verticalmente */
                border-radius: 10px;  /* Bordes redondeados del control deslizante */
            }

            QSlider::add-page:horizontal {
                background: #E0E0E0;  /* Fondo gris claro */
            }

            QSlider::sub-page:horizontal {
                background: #BDBDBD;  /* Fondo gris más oscuro */
            }
        """
        
        self.sliderMixerDepth.setStyleSheet(styleSliders)
        self.sliderBlockSize.setStyleSheet(styleSliders)
        self.sliderWinSize.setStyleSheet(styleSliders)

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

    def startVideoStream(self):
        try:
            self.getFrame.startStream(self.cameraResolution, self.cameraFps)
        except Exception as error :
            print('error', error)
            return False
        else:
            return True

    def stopVideoStream(self):
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

    def changeResolution(self,index):                   # optimizar
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

    def depthMixerChange(self,value):
        self.labelDepthValue.setText(f"Depth mixer: {value/100.00}")
        depthMapProcessor.changeMixDepthValue(value/100.00)

    def blockSizeChange(self,value):
        self.labelBlockSize.setText(f"Block size: {value}")
        depthMapProcessor.changeBlockSizeValue(value)

    def winSizeChange(self,value):
        self.labelWinSize.setText(f"window size: {value}")
        depthMapProcessor.changeWinSizeValue(value)


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
