# --------------------------------
# Importamos librearías a utilizar

from PyQt5.QtWidgets import *
from PyQt5.Qt import *
from PyQt5.QtGui import *
from PyQt5 import QtGui
from PyQt5.QtCore import *
import sys
from math import pi
import sympy as sp
import math as m
import serial
import time
import numpy as np

from controlCamera import ControlCamera 
from controlCamera import StatusCamera

# --------------------------------
# Creamos la Clase de la Interfaz


class Window(QMainWindow):
    def __init__(self):
        # --------------------------------
        # Esta línea es importante para que funcione el programa
        QMainWindow.__init__(self)
        super(QMainWindow, self).__init__(parent=None)

        # --------------------------------
        # Propiedades de la ventana

        self.setFixedHeight(650)
        self.setFixedWidth(800)
        self.setWindowTitle("Ps5 Camera GUI")
        # self.setWindowIcon(QtGui.QIcon("ScriptScanner/icono_intefaz.png"))

        self.controlCamera = ControlCamera()

        self.statusCamera = False

        # --------------------------------
        # Se agregan todos los widgets

        self.setGrid()
        self.setBoxes()
        self.setButtons()
        # self.setLabels()
        # self.setInputs()
        # self.setComboBox()
        # self.applyStyles()

    def setGrid(self):
        # --------------------------------
        # Pestaña Principal
        self.MainWidget = QWidget()
        self.setCentralWidget(self.MainWidget)
        self.Grid_layout = QGridLayout()
        self.Grid_layout.setSpacing(2)
        self.MainWidget.setLayout(self.Grid_layout)

    def setBoxes(self):
        # --------------------------------
        # Cuadro de ingresos de coordenadas espaciales

        self.lyt_inputs = QGridLayout()
        self.lyt_inputs.setSpacing(2)

        # self.gb_Inputs = QGroupBox("Final effector position", parent=self)
        # self.gb_Inputs.setLayout(self.lyt_inputs)

        # --------------------------------
        # Cuadro de botones de control

        self.lyt_Commands = QGridLayout()
        self.lyt_Commands.setSpacing(2)

        self.gb_Commands = QGroupBox("Control panel", parent=self)
        self.gb_Commands.setLayout(self.lyt_Commands)

        # # --------------------------------
        # # Cuadro de labels de variables articulares

        # self.lyt_JPositions = QGridLayout()
        # self.lyt_JPositions.setSpacing(2)

        # self.gb_JPositions = QGroupBox("Joint Positions", parent=self)
        # self.gb_JPositions.setLayout(self.lyt_JPositions)

        # # --------------------------------
        # # Cuadro de trayectorias

        # self.lyt_TypeMove = QGridLayout()
        # self.lyt_TypeMove.setSpacing(2)

        # self.gb_Movement = QGroupBox("Movement", parent=self)
        # self.gb_Movement.setLayout(self.lyt_TypeMove)

        # # Cuadro de configuracion de velocidades

        # self.lyt_speed = QGridLayout()
        # self.lyt_speed.setSpacing(2)

        # # self.gb_speed = QGroupBox("Config", parent=self)
        # # self.gb_speed.setLayout(self.lyt_speed)

        # # Cuadro de configuracion de articulaciones

        # self.lyt_art_value = QGridLayout()
        # self.lyt_art_value.setSpacing(2)

        # self.gb_artValue = QGroupBox("Set Joint Positions", parent=self)
        # self.gb_artValue.setLayout(self.lyt_art_value)

        # # --------------------------------
        # # Agregamos los boxes a los layouts correspondientes

        # self.Grid_layout.addWidget(self.gb_JPositions, 0, 0, 1, 5)
        self.Grid_layout.addWidget(self.gb_Commands, 0, 5, 1, 1)

        # # self.Grid_layout.addWidget(self.gb_Inputs, 1, 0, 1, 5)
        # # self.Grid_layout.addWidget(self.gb_speed, 1, 5, 1, 1)

        # self.Grid_layout.addWidget(self.gb_Movement, 2, 0, 1, 6)
        # self.Grid_layout.addWidget(self.gb_artValue, 3, 0, 1, 6)

    def setButtons(self):
        # --------------------------------
        # Botón para conexión/desconexión con esp32

        self.btnLoadFirm = QPushButton(text="Load Firmware")
        self.btnLoadFirm.setEnabled(True)
        self.btnLoadFirm.clicked.connect(self.loadFirmare)
        self.lyt_Commands.addWidget(self.btnLoadFirm, 0, 0, 1, 1)

        # --------------------------------
        # Botón de parada de emergencia

        self.btnTakeSnapshot = QPushButton(text="Take snapshot")
        self.btnTakeSnapshot.setEnabled(False)
        # self.btnTakeSnapshot.clicked.connect(self.EMER_STOP)
        self.lyt_Commands.addWidget(self.btnTakeSnapshot, 2, 0, 1, 1)



    def setLabels(self):
        # --------------------------------
        # Label para mostrar la variable articular Q1

        self.lbl_Q1 = QLabel(parent=self.gb_JPositions)
        self.lbl_Q1.setText("Q1")
        self.lbl_Q1.setFixedHeight(45)
        self.lbl_Q1.setAlignment(Qt.AlignCenter)

        # --------------------------------
        # Label para mostrar la variable articular Q2

        self.lbl_Q2 = QLabel(parent=self.gb_JPositions)
        self.lbl_Q2.setText("Q2")
        self.lbl_Q2.setFixedHeight(45)
        self.lbl_Q2.setAlignment(Qt.AlignCenter)

        # --------------------------------
        # Label para mostrar la variable articular Q3

        self.lbl_Q3 = QLabel(parent=self.gb_JPositions)
        self.lbl_Q3.setText("Q3")
        self.lbl_Q3.setFixedHeight(45)
        self.lbl_Q3.setAlignment(Qt.AlignCenter)
        # --------------------------------
        # Angulo Q1

        self.lbl_angQ1 = QLabel(parent=self.gb_JPositions)
        self.lbl_angQ1.setText("0.0°")
        self.lbl_angQ1.setFixedHeight(45)
        self.lbl_angQ1.setAlignment(Qt.AlignCenter)

        # --------------------------------
        # Angulo Q2

        self.lbl_angQ2 = QLabel(parent=self.gb_JPositions)
        self.lbl_angQ2.setText("0.0°")
        self.lbl_angQ2.setFixedHeight(45)
        self.lbl_angQ2.setAlignment(Qt.AlignCenter)

        # --------------------------------
        # Angulo Q3

        self.lbl_angQ3 = QLabel(parent=self.gb_JPositions)
        self.lbl_angQ3.setText("0.0°")
        self.lbl_angQ3.setFixedHeight(45)
        self.lbl_angQ3.setAlignment(Qt.AlignCenter)

        self.lyt_JPositions.addWidget(self.lbl_Q1, 0, 0, 1, 1)
        self.lyt_JPositions.addWidget(self.lbl_Q2, 0, 1, 1, 1)
        self.lyt_JPositions.addWidget(self.lbl_Q3, 0, 2, 1, 1)
        self.lyt_JPositions.addWidget(self.lbl_angQ1, 1, 0, 1, 1)
        self.lyt_JPositions.addWidget(self.lbl_angQ2, 1, 1, 1, 1)
        self.lyt_JPositions.addWidget(self.lbl_angQ3, 1, 2, 1, 1)

        # --------------------------------
        # Label para entrada de Posiciones X,Y,Z

        self.lbl_Px = QLabel(parent=self.gb_JPositions)
        self.lbl_Px.setText("Px")
        self.lbl_Px.setAlignment(Qt.AlignCenter)

        self.lbl_Py = QLabel(parent=self.gb_JPositions)
        self.lbl_Py.setText("Py")
        self.lbl_Py.setAlignment(Qt.AlignCenter)

        self.lbl_Pz = QLabel(parent=self.gb_JPositions)
        self.lbl_Pz.setText("Pz")
        self.lbl_Pz.setAlignment(Qt.AlignCenter)

        self.lyt_inputs.addWidget(self.lbl_Px, 0, 0, 1, 1)
        self.lyt_inputs.addWidget(self.lbl_Py, 1, 0, 1, 1)
        self.lyt_inputs.addWidget(self.lbl_Pz, 2, 0, 1, 1)

        # Label de error, por defecto oculto
        self.lbl_err_Pxyz = QLabel(parent=self.gb_JPositions)
        self.lbl_err_Pxyz.setStyleSheet(
            "color: red; font-size: 12px; font-family: 'Consolas'; "
        )
        self.lbl_err_Pxyz.setText("Revise los campos de entrada")
        self.lbl_err_Pxyz.setAlignment(Qt.AlignCenter)
        self.lbl_err_Pxyz.setVisible(False)
        self.lyt_JPositions.addWidget(self.lbl_err_Pxyz, 3, 0, 1, 3)

        # --------------------------------
        # Label para entrada de Posiciones Xi,Yi,Zi y Xf,Yf,Zf

        self.lbl_Pxi = QLabel(parent=self.gb_JPositions)
        self.lbl_Pxi.setText("Pxi")
        self.lbl_Pxi.setAlignment(Qt.AlignCenter)

        self.lbl_Pyi = QLabel(parent=self.gb_JPositions)
        self.lbl_Pyi.setText("Pyi")
        self.lbl_Pyi.setAlignment(Qt.AlignCenter)

        self.lbl_Pzi = QLabel(parent=self.gb_JPositions)
        self.lbl_Pzi.setText("Pzi")
        self.lbl_Pzi.setAlignment(Qt.AlignCenter)

        self.lbl_Pxf = QLabel(parent=self.gb_JPositions)
        self.lbl_Pxf.setText("Pxf")
        self.lbl_Pxf.setAlignment(Qt.AlignCenter)

        self.lbl_Pyf = QLabel(parent=self.gb_JPositions)
        self.lbl_Pyf.setText("Pyf")
        self.lbl_Pyf.setAlignment(Qt.AlignCenter)

        self.lbl_Pzf = QLabel(parent=self.gb_JPositions)
        self.lbl_Pzf.setText("Pzf")
        self.lbl_Pzf.setAlignment(Qt.AlignCenter)

        self.lbl_Pinicial = QLabel(parent=self.gb_JPositions)
        self.lbl_Pinicial.setText("Initial Point")
        self.lbl_Pinicial.setAlignment(Qt.AlignCenter)

        self.lbl_Pfinal = QLabel(parent=self.gb_JPositions)
        self.lbl_Pfinal.setText("Final Point")
        self.lbl_Pfinal.setAlignment(Qt.AlignCenter)

        self.lyt_TypeMove.addWidget(self.lbl_Pinicial, 0, 1, 1, 1)
        self.lyt_TypeMove.addWidget(self.lbl_Pxi, 1, 0, 1, 1)
        self.lyt_TypeMove.addWidget(self.lbl_Pyi, 2, 0, 1, 1)
        self.lyt_TypeMove.addWidget(self.lbl_Pzi, 3, 0, 1, 1)

        self.lyt_TypeMove.addWidget(self.lbl_Pfinal, 0, 3, 1, 1)
        self.lyt_TypeMove.addWidget(self.lbl_Pxf, 1, 2, 1, 1)
        self.lyt_TypeMove.addWidget(self.lbl_Pyf, 2, 2, 1, 1)
        self.lyt_TypeMove.addWidget(self.lbl_Pzf, 3, 2, 1, 1)

        self.lbl_points_qty = QLabel(parent=self.gb_JPositions)
        self.lbl_points_qty.setText("Points")
        self.lbl_points_qty.setAlignment(Qt.AlignCenter)

        self.lyt_TypeMove.addWidget(self.lbl_points_qty, 4, 0, 1, 1)

        self.lbl_Q1value = QLabel(parent=self.gb_artValue)
        self.lbl_Q1value.setText("Q1")
        self.lbl_Q1value.setAlignment(Qt.AlignCenter)

        self.lbl_Q2value = QLabel(parent=self.gb_artValue)
        self.lbl_Q2value.setText("Q2")
        self.lbl_Q2value.setAlignment(Qt.AlignCenter)

        self.lbl_Q3value = QLabel(parent=self.gb_artValue)
        self.lbl_Q3value.setText("Q3")
        self.lbl_Q3value.setAlignment(Qt.AlignCenter)

        self.lyt_art_value.addWidget(self.lbl_Q1value, 0, 0, 1, 1)
        self.lyt_art_value.addWidget(self.lbl_Q2value, 0, 1, 1, 1)
        self.lyt_art_value.addWidget(self.lbl_Q3value, 0, 2, 1, 1)

    def setInputs(self):
        self.inpt_Px = QLineEdit(parent=self.gb_Inputs)
        self.inpt_Px.setValidator(QDoubleValidator())
        self.inpt_Px.setMaxLength(10)
        self.inpt_Px.setAlignment(Qt.AlignRight)
        self.inpt_Px.setEnabled(False)
        self.inpt_Px.setText("0")

        self.inpt_Py = QLineEdit(parent=self.gb_Inputs)
        self.inpt_Py.setValidator(QDoubleValidator())
        self.inpt_Py.setMaxLength(10)
        self.inpt_Py.setAlignment(Qt.AlignRight)
        self.inpt_Py.setEnabled(False)
        self.inpt_Py.setText("0")

        self.inpt_Pz = QLineEdit(parent=self.gb_Inputs)
        self.inpt_Pz.setValidator(QDoubleValidator())
        self.inpt_Pz.setMaxLength(10)
        self.inpt_Pz.setAlignment(Qt.AlignRight)
        self.inpt_Pz.setEnabled(False)
        self.inpt_Pz.setText("0")

        self.lyt_inputs.addWidget(self.inpt_Px, 0, 1, 1, 1)
        self.lyt_inputs.addWidget(self.inpt_Py, 1, 1, 1, 1)
        self.lyt_inputs.addWidget(self.inpt_Pz, 2, 1, 1, 1)

        self.inpt_Pxi = QLineEdit(parent=self.gb_Inputs)
        self.inpt_Pxi.setValidator(QDoubleValidator())
        self.inpt_Pxi.setMaxLength(10)
        self.inpt_Pxi.setAlignment(Qt.AlignRight)
        self.inpt_Pxi.setEnabled(False)
        self.inpt_Pxi.setText("0")

        self.inpt_Pyi = QLineEdit(parent=self.gb_Inputs)
        self.inpt_Pyi.setValidator(QDoubleValidator())
        self.inpt_Pyi.setMaxLength(10)
        self.inpt_Pyi.setAlignment(Qt.AlignRight)
        self.inpt_Pyi.setEnabled(False)
        self.inpt_Pyi.setText("0")

        self.inpt_Pzi = QLineEdit(parent=self.gb_Inputs)
        self.inpt_Pzi.setValidator(QDoubleValidator())
        self.inpt_Pzi.setMaxLength(10)
        self.inpt_Pzi.setAlignment(Qt.AlignRight)
        self.inpt_Pzi.setEnabled(False)
        self.inpt_Pzi.setText("0")

        self.inpt_Pxf = QLineEdit(parent=self.gb_Inputs)
        self.inpt_Pxf.setValidator(QDoubleValidator())
        self.inpt_Pxf.setMaxLength(10)
        self.inpt_Pxf.setAlignment(Qt.AlignRight)
        self.inpt_Pxf.setEnabled(False)
        self.inpt_Pxf.setText("0")

        self.inpt_Pyf = QLineEdit(parent=self.gb_Inputs)
        self.inpt_Pyf.setValidator(QDoubleValidator())
        self.inpt_Pyf.setMaxLength(10)
        self.inpt_Pyf.setAlignment(Qt.AlignRight)
        self.inpt_Pyf.setEnabled(False)
        self.inpt_Pyf.setText("0")

        self.inpt_Pzf = QLineEdit(parent=self.gb_Inputs)
        self.inpt_Pzf.setValidator(QDoubleValidator())
        self.inpt_Pzf.setMaxLength(10)
        self.inpt_Pzf.setAlignment(Qt.AlignRight)
        self.inpt_Pzf.setEnabled(False)
        self.inpt_Pzf.setText("0")

        self.lyt_TypeMove.addWidget(self.inpt_Pxi, 1, 1, 1, 1)
        self.lyt_TypeMove.addWidget(self.inpt_Pyi, 2, 1, 1, 1)
        self.lyt_TypeMove.addWidget(self.inpt_Pzi, 3, 1, 1, 1)
        self.lyt_TypeMove.addWidget(self.inpt_Pxf, 1, 3, 1, 1)
        self.lyt_TypeMove.addWidget(self.inpt_Pyf, 2, 3, 1, 1)
        self.lyt_TypeMove.addWidget(self.inpt_Pzf, 3, 3, 1, 1)

        self.inpt_points_qty = QLineEdit(parent=self.gb_Inputs)
        self.inpt_points_qty.setValidator(QDoubleValidator())
        self.inpt_points_qty.setMaxLength(10)
        self.inpt_points_qty.setAlignment(Qt.AlignRight)
        self.inpt_points_qty.setEnabled(False)
        self.inpt_points_qty.setText("1")

        self.lyt_TypeMove.addWidget(self.inpt_points_qty, 4, 1, 1, 1)

        self.inptQ1pos = QLineEdit(parent=self.gb_Inputs)
        self.inptQ1pos.setValidator(QDoubleValidator())
        self.inptQ1pos.setMaxLength(10)
        self.inptQ1pos.setAlignment(Qt.AlignCenter)
        self.inptQ1pos.setText("0")
        self.inptQ1pos.setEnabled(False)

        self.inptQ2pos = QLineEdit(parent=self.gb_Inputs)
        self.inptQ2pos.setValidator(QDoubleValidator())
        self.inptQ2pos.setMaxLength(10)
        self.inptQ2pos.setAlignment(Qt.AlignCenter)
        self.inptQ2pos.setText("0")
        self.inptQ2pos.setEnabled(False)

        self.inptQ3pos = QLineEdit(parent=self.gb_Inputs)
        self.inptQ3pos.setValidator(QDoubleValidator())
        self.inptQ3pos.setMaxLength(10)
        self.inptQ3pos.setAlignment(Qt.AlignCenter)
        self.inptQ3pos.setText("0")
        self.inptQ3pos.setEnabled(False)

        self.lyt_art_value.addWidget(self.inptQ1pos, 1, 0, 1, 1)
        self.lyt_art_value.addWidget(self.inptQ2pos, 1, 1, 1, 1)
        self.lyt_art_value.addWidget(self.inptQ3pos, 1, 2, 1, 1)

    def setComboBox(self):
        self.combo_box_speed = QComboBox()
        velocidades = [
            "Speed 10",
            "Speed 20",
            "Speed 30",
            "Speed 40",
            "Speed 50",
            "Speed 60",
            "Speed 70",
            "Speed 80",
            "Speed 90",
            "Speed 100",
        ]
        self.combo_box_speed.addItems(velocidades)
        self.combo_box_speed.currentIndexChanged.connect(self.SEND_SPEED)
        self.combo_box_speed.setEnabled(False)
        self.lyt_speed.addWidget(self.combo_box_speed, 0, 0, 1, 1)

        self.combo_box_def_position = QComboBox()
        position = ["Custom", "Home", "Brazo horizontal", "Brazo arriba"]
        self.combo_box_def_position.addItems(position)
        self.combo_box_def_position.currentIndexChanged.connect(
            self.sendDefaultPosition
        )
        self.combo_box_def_position.setEnabled(False)
        self.lyt_inputs.addWidget(self.combo_box_def_position, 1, 3, 1, 1)

    def onClose(self, QCloseEvent):
        super().closeEvent(QCloseEvent)
        self.ser.close()

    

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

        self.btnLoadFirm.setStyleSheet(btn_estilo)
        self.btn_lineal_move.setStyleSheet(btn_estilo)
        self.btnTakeSnapshot.setStyleSheet(btn_estilo)
        self.btn_Move.setStyleSheet(btn_estilo)
        self.btn_MovetoArt.setStyleSheet(btn_estilo)

        self.btnLoadFirm.setFixedHeight(20)
        self.btn_lineal_move.setFixedHeight(20)
        self.btnTakeSnapshot.setFixedHeight(20)
        self.btn_Move.setFixedHeight(20)
        self.btn_MovetoArt.setFixedHeight(20)
        self.btn_MovetoArt.setFixedWidth(100)

        self.setStyleSheet("background-color:#F4F6F6;")


    def loadFirmare(self):
        # print("Iniciando carga de firmware")

        self.statusCamera = self.controlCamera.getCameraStatus()
        if( self.statusCamera == StatusCamera.CAMERA_NOT_CONNECTED ):
            print("Camera not found")
        elif( self.statusCamera == StatusCamera.CAMERA_CONNECTED_PENDING_FW ):
            print("Trying load firmware")
            if( self.controlCamera.loadFirmwareCamera() == True ):
                print("Camera connected!")
            else:
                print("Error trying connect")
        else:
            print("Camera already connect")
        
    



# --------------------------------
# Se ejecuta la interfaz

if __name__ == "__main__":
    App = QApplication(sys.argv)
    App.setStyle("Fusion")

    w = Window()

    w.show()  # Mostramos la ventana

    sys.stdout.flush()
    sys.exit(App.exec_())
