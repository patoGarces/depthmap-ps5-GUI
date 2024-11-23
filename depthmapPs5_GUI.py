# --------------------------------
# Importamos librearías a utilizar
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.Qt import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt

import sys
import numpy as np
import logging
import cv2

from stl import mesh
from scipy.spatial import Delaunay

# from reactivex import create
from reactivex import operators as ops

from Utils.utils import CameraStatusUi
from controlCamera import ControlCamera 
from controlCamera import StatusCamera
from getFrame import GetFrame
from getFrame import Resolutions
from depthMapProcessor import DepthMapProcessor
from plotterClasses import DrawOutput2D
from plotterClasses import DrawOutput3D
from SerialManager import SerialManager

from plotterClasses import Communicator

VID_CAMERA = 'VID_05A9'

# Creamos la Clase de la Interfaz
class UIManager(QMainWindow):

    statusCameraUi = 0
    cameraResolution = Resolutions.RES_640x480
    cameraFps = 8

    def __init__(self,_depthMapProcessor):
        # --------------------------------
        # Esta línea es importante para que funcione el programa
        QMainWindow.__init__(self)
        super(QMainWindow, self).__init__(parent=None)

        self.controlCamera = ControlCamera()
        self.getFrame =  GetFrame()
        self.drawer2DWidget = DrawOutput2D()

        self.communicator = Communicator()  # Crear la instancia de Communicator
        self.drawer3DWidget = DrawOutput3D(self.communicator)
        # self.drawer3DWidget = DrawOutput3D()
        self.depthMapProcessor = _depthMapProcessor
        self.serialManager = SerialManager()

        self.cameraPose = (0.00,0.00,0.00,True)
        self.zFilterHeight = 0
        self.zFilterThickness = 0.5
        self.depthFilter = 2.5

        self.forwardPose = 0.0

        if not hasattr(self, 'nube_puntos'):
            self.output2D = []
        # self.output2D = []

        # --------------------------------
        self.setWindowTitle("Ps5 Camera GUI")
        # Propiedades de la ventana
        # self.setFixedHeight(650)
        # self.setFixedWidth(800)
        self.setGrid()
        self.setMenuControlPanel()
        self.setVideoCameraView()
        self.applyStyles()

        self.updateCameraStatus()
        self.setObserverGetFrames()

        self.adjustSize()

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
            "640x480 8fps",
            "640x480 30fps",
            "640x480 60fps",
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

        self.labelStatusTitle = QLabel(parent=self.gb_videoControl)
        self.labelStatusTitle.setText("Conexion serial:")
        self.labelStatusTitle.setAlignment(Qt.AlignLeft)
        videoControlLayout.addWidget(self.labelStatusTitle)

        # Combo box para seleccionar el puerto serial
        self.comboCommSerial = QComboBox()
        self.comboCommSerial.currentIndexChanged.connect(self.connectSerial)
        self.comboCommSerial.showPopup = self.updateSerialComms
        self.comboCommSerial.setEnabled(True)
        videoControlLayout.addWidget(self.comboCommSerial)

        self.labelStatusSerial = QLabel(parent=self.gb_videoControl)
        self.labelStatusSerial.setText("Desconectado")
        self.labelStatusSerial.setAlignment(Qt.AlignCenter)
        videoControlLayout.addWidget(self.labelStatusSerial)

        self.labelXYZSerial = QLabel(parent=self.gb_videoControl)
        self.labelXYZSerial.setText("X: -   Y: -    Z: -")
        self.labelXYZSerial.setAlignment(Qt.AlignCenter)
        videoControlLayout.addWidget(self.labelXYZSerial)

        # Cuadro de control de pointcloud
        pointCloudControlLayout = QVBoxLayout()
        pointCloudControlLayout.setSpacing(10)
        pointCloudControlLayout.setAlignment(Qt.AlignTop)

        self.gb_pointCloudControl = QGroupBox("Control de DepthMap", parent=self)
        self.gb_pointCloudControl.setLayout(pointCloudControlLayout)
        self.gb_pointCloudControl.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)

        self.labelZFilterHeight = QLabel(f"Z filter height: {self.zFilterHeight}", self)

        self.sliderZFilterHeight = QSlider(Qt.Horizontal)
        self.sliderZFilterHeight.setMinimum(-250)
        self.sliderZFilterHeight.setMaximum(250)
        self.sliderZFilterHeight.setValue(int(self.zFilterHeight*100))
        self.sliderZFilterHeight.setTickPosition(QSlider.TicksBelow)
        self.sliderZFilterHeight.setTickInterval(10)
        self.sliderZFilterHeight.valueChanged.connect(self.zFilterHeightChange)
        
        pointCloudControlLayout.addWidget(self.labelZFilterHeight)
        pointCloudControlLayout.addWidget(self.sliderZFilterHeight)

        self.labelZFilterThickness = QLabel(f"Z filter thickness: {self.zFilterThickness}", self)

        self.sliderZFilterThickness = QSlider(Qt.Horizontal)
        self.sliderZFilterThickness.setMinimum(1)
        self.sliderZFilterThickness.setMaximum(100)
        self.sliderZFilterThickness.setValue(int(self.zFilterThickness*100))
        self.sliderZFilterThickness.setTickPosition(QSlider.TicksBelow)
        self.sliderZFilterThickness.setTickInterval(10)
        self.sliderZFilterThickness.valueChanged.connect(self.zFilterThicknessChange)
        
        pointCloudControlLayout.addWidget(self.labelZFilterThickness)
        pointCloudControlLayout.addWidget(self.sliderZFilterThickness)

        self.labelDepthFilter = QLabel(f"Depth filter: {self.depthFilter}", self)

        self.sliderDepthFilter = QSlider(Qt.Horizontal)
        self.sliderDepthFilter.setMinimum(1)
        self.sliderDepthFilter.setMaximum(300)
        self.sliderDepthFilter.setValue(int(self.depthFilter*100))
        self.sliderDepthFilter.setTickPosition(QSlider.TicksBelow)
        self.sliderDepthFilter.setTickInterval(10)
        self.sliderDepthFilter.valueChanged.connect(self.depthFilterChange)


        # Boton para simular avance del robot
        self.btnForwardPose = QPushButton(text="avanzar 1mt")
        self.btnForwardPose.clicked.connect(self.testForwardPose)

        # Boton limpiar nube de puntos
        self.btnClearPointCloud = QPushButton(text="limpiar pointcloud")
        self.btnClearPointCloud.clicked.connect(self.testClearPointcloud)
        
        pointCloudControlLayout.addWidget(self.labelDepthFilter)
        pointCloudControlLayout.addWidget(self.sliderDepthFilter)
        pointCloudControlLayout.addWidget(self.btnForwardPose) 
        pointCloudControlLayout.addWidget(self.btnClearPointCloud)       

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
        controlLayout.addWidget(self.gb_pointCloudControl, alignment=Qt.AlignTop)
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
        
        # drawer 2D
        self.drawer2DWidget.setMinimumSize(400, 300)
        self.videoLayout.addWidget(self.drawer2DWidget, 2, 0, 1, 2)

        # drawer 3D
        self.drawer3DWidget.setMinimumSize(800, 800)
        self.videoLayout.addWidget(self.drawer3DWidget, 0, 2, 3, 3)

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
        self.sliderDepthFilter.setStyleSheet(styleSliders)
        self.sliderZFilterHeight.setStyleSheet(styleSliders)
        self.sliderZFilterThickness.setStyleSheet(styleSliders)

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
        # frameL, frameR = self.depthMapProcessor.rectifiedFrames(frameL,frameR)
        undisFrameL, undisFrameR = self.depthMapProcessor.downsampledFrames(frameL,frameR)
        # undisFrameL, undisFrameR = self.depthMapProcessor.getHorizontalStripe(frameL,frameR)
        
        depthMapColor,points,disp,disp_color,depthMap = self.depthMapProcessor.getStereoDepthmap(undisFrameL,undisFrameR)

        frame = cv2.cvtColor(undisFrameL, cv2.COLOR_BGR2RGB)                       # Convertir el fotograma a RGB
        self.video_labelL.setPixmap(self.convertFrameToQt(frame))                  # Mostrar el fotograma en el QLabel

        frame = cv2.cvtColor(undisFrameR, cv2.COLOR_BGR2RGB)                       # Convertir el fotograma a RGB
        self.video_labelR.setPixmap(self.convertFrameToQt(frame))                  # Mostrar el fotograma en el QLabel
        frame = cv2.cvtColor(depthMapColor, cv2.COLOR_BGR2RGB)                     # Convertir el fotograma a RGB

        frameWithLine = cv2.line(frame, (0, int(frame.shape[0]/2)), (frame.shape[1], int(frame.shape[0]/2)), (255, 255, 255), 1)
        self.video_labelDepthMap.setPixmap(self.convertFrameToQt(frameWithLine,self.video_labelDepthMap.width(),self.video_labelDepthMap.height()))           # Mostrar el fotograma en el QLabel

        # z_max = -2.3
        z_max = -self.depthFilter               # TODO: remover el '-' cuando este bien ubicada la nube de puntos espacialmente
        mask = (points[:, :, 2] >= z_max)
        points = points[mask]

        num_points = points.shape[0]  # Solo usamos el primer eje

        # Concatenamos los puntos homogéneos
        points_homogeneous = np.concatenate((points, np.ones((num_points, 1))), axis=1)

        # Rotación de la nube de puntos
        yaw = np.radians(self.cameraPose[2])
        Qyaw = np.float32([[np.cos(yaw)     , 0 , np.sin(yaw)   , 0],
                            [0              , 1 , 0             , self.forwardPose],
                            [-np.sin(yaw)   , 0 , np.cos(yaw)   , 0],
                            [0              , 0 , 0             , 1]])
        
        pitch = np.radians(self.cameraPose[0])
        Qpitch = np.float32([[  1     , 0               , 0                 , 0],
                            [   0     , np.cos(pitch)   , -np.sin(pitch)    , 0],
                            [   0     , np.sin(pitch)   , np.cos(pitch)     , 0],
                            [   0     , 0               , 0                 , 1]])

        roll = np.radians(self.cameraPose[1])

        print('Pitch: '+ str(round(pitch,2)) + '   Roll: '+ str(round(roll,2)) + '    Yaw: ' + str(round(yaw,2)) + '    Forward: ' + str(self.forwardPose))
        self.labelXYZSerial.setText('Pitch: '+ str(round(pitch,2)) + '   Roll: '+ str(round(roll,2)) + '    Yaw: ' + str(round(yaw,2)))
        # Multiplico por la matriz de rotación
        rotated_points = np.dot(points_homogeneous, Qyaw.T)
        rotated_points = np.dot(rotated_points, Qpitch.T)

        rotated_points = rotated_points[:, :3]          # Elimino la cuarta columna

        # print('size points: ' + str(np.size(points)) + 'disp max: ' +str(disp.max()))
        # mask = (disp > 0) & (disp < 255)     # Descarto puntos invalidos
        # points = points[mask]
        # print('size points mask: ' + str(np.size(points)))

        out_colors = cv2.cvtColor(disp_color, cv2.COLOR_BGR2RGB)[mask]

        rotated_points = rotated_points.reshape(-1, 3)         # Aplano a (800*1264, 3)

        # # Agrega los puntos rotados actuales a la lista acumulativa
        # self.output2D.append(rotated_points)
        # # Convierte la lista acumulada en un array de NumPy para su procesamiento
        # nube_puntos_acumulada = np.vstack(self.output2D)
        # self.drawer2DWidget.updatePlot(nube_puntos_acumulada,out_colors)

        

        self.drawer2DWidget.updatePlot(rotated_points,out_colors)

        y_min = self.zFilterHeight - self.zFilterThickness
        y_max = self.zFilterHeight + self.zFilterThickness    

        index_filter = (rotated_points[:, 1] >= y_min) & (rotated_points[:, 1] <= y_max)
        # Filtro los puntos que cumplen la condicion
        rotated_points = rotated_points[index_filter]
        out_colors = out_colors[index_filter]

        self.communicator.update_cloud.emit(rotated_points, out_colors)  # Emitir la señal

        # self.write_ply('point_cloud.ply', rotated_points, out_colors)
        # self.write_stl_with_delaunay('point_cloud.stl',rotated_points)

    def write_stl_with_delaunay(self,filename, points):
        # Realiza la triangulación de Delaunay en 3D
        delaunay = Delaunay(points)
        
        # Obtiene los triángulos de la malla generada por Delaunay
        triangles = delaunay.simplices

        # Crea la estructura de datos STL para almacenar los triángulos
        stl_data = np.zeros(len(triangles), dtype=mesh.Mesh.dtype)

        # Asigna los triángulos a la estructura STL
        for i, tri in enumerate(triangles):
            stl_data['vectors'][i] = points[tri]

        # Guarda la malla en un archivo STL
        point_cloud_mesh = mesh.Mesh(stl_data)
        point_cloud_mesh.save(filename)
        print(f"Archivo STL con superficie guardado en: {filename}")

    def write_stl(self,filename, points):
        # Crea una malla con un triángulo por punto
        # (No es ideal para una nube de puntos sin superficie, pero sirve para visualización básica)
        stl_data = np.zeros(len(points), dtype=mesh.Mesh.dtype)
        
        # Recorre cada punto y lo agrega como un triángulo mínimo
        for i, point in enumerate(points):
            stl_data['vectors'][i] = np.array([point, point, point])  # Triángulo "degradado"

        # Guarda la malla en un archivo STL
        cloud_mesh = mesh.Mesh(stl_data)
        cloud_mesh.save(filename)
        print(f"Nube de puntos guardada en: {filename}")

    # Genera el archivo PLY
    def write_ply(self,filename, points, colors):
        with open(filename, 'w') as f:
            f.write(f"ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write("end_header\n")
            for p, c in zip(points, colors):
                f.write(f"{p[0]} {p[1]} {p[2]} {c[0]} {c[1]} {c[2]}\n")

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
            self.getFrame.startStream(VID_CAMERA,self.cameraResolution, self.cameraFps)
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
            self.cameraResolution = Resolutions.RES_640x480
            self.cameraFps = 8
        elif (index == 1):
            self.cameraResolution = Resolutions.RES_640x480
            self.cameraFps = 30
        elif (index == 2):
            self.cameraResolution = Resolutions.RES_640x480
            self.cameraFps = 60
        if (index == 3):
            self.cameraResolution = Resolutions.RES_2560x800
            self.cameraFps = 8
        elif (index == 4):
            self.cameraResolution = Resolutions.RES_2560x800
            self.cameraFps = 30
        elif (index == 5):
            self.cameraResolution = Resolutions.RES_2560x800
            self.cameraFps = 60
        elif (index == 6):
            self.cameraResolution = Resolutions.RES_3448x808
            self.cameraFps = 8
        elif (index == 7):
            self.cameraResolution = Resolutions.RES_3448x808
            self.cameraFps = 30
        elif (index == 8):
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

    def zFilterHeightChange(self,value):
        self.labelZFilterHeight.setText(f"Z filter height: {value/100}")
        self.zFilterHeight = value/100                
    
    def zFilterThicknessChange(self,value):
        self.labelZFilterThickness.setText(f"Z filter thickness: {value/100}")
        self.zFilterThickness = value/100

    def depthFilterChange(self,value):
        self.labelDepthFilter.setText(f"Depth filter: {value/100}")
        self.depthFilter = value /100

    def updateSerialComms(self):
        self.listCommSerial = self.serialManager.getSerialComms()
        if (len(self.listCommSerial) == 0):
            self.comboCommSerial.clear()
        else:
            self.comboCommSerial.addItems(self.listCommSerial)

    def connectSerial(self,index):
        if (self.serialManager.connect(self.listCommSerial[index],115200,w.onNewPose)):
            self.labelStatusSerial.setText("Conectado")
        else:
            self.labelStatusSerial.setText("Desconectado")

    # Callback que se llama desde Serial.Manager
    def onNewPose(self, pose, error):
        if (error == False):
            self.cameraPose = pose
        else:
            self.labelStatusSerial.setText("Desconectado")
            self.comboCommSerial.clear()

    def testForwardPose(self):
        self.forwardPose -= 10.0

    def testClearPointcloud(self): 
        self.output2D = []

if __name__ == "__main__":
    App = QApplication(sys.argv)
    App.setStyle("Fusion")

    npRet = np.load('DepthParams/param_ret.npy')
    npK = np.load('DepthParams/param_K.npy')
    npDist = np.load('DepthParams/param_dist.npy')
    depthMapProcessor = DepthMapProcessor(npRet,npK,npDist)

    # logging.basicConfig(level=logging.NOTSET)

    w = UIManager(depthMapProcessor)

    w.show()

    sys.stdout.flush()
    sys.exit(App.exec_())
