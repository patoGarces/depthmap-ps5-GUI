import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import *

import pyvista as pv
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from pyvistaqt import QtInteractor
import numpy as np
from PyQt5.QtCore import pyqtSignal, QObject


class Communicator(QObject):
    update_cloud = pyqtSignal(np.ndarray, np.ndarray)  # Señal para actualizar la nube de puntos

class DrawOutput3D(QWidget):
    def __init__(self, communicator):
        super().__init__()

        self.communicator = communicator
        self.communicator.update_cloud.connect(self.updatePointCloud)  # Conecto la señal para actualizar la nube de puntos

        # Creo el renderizador de PyVista usando QtInteractor
        self.plotter = QtInteractor(self)
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.plotter)
        self.setLayout(self.layout)

        self.plotter.set_background('white')  # Establece el color de fondo
        self.plotter.add_axes()  # Agrega ejes para la referencia

    def updatePointCloud(self, points, colors):
        # Limpio el renderizador
        self.plotter.clear()

        # Creo una nube de puntos
        point_cloud = pv.PolyData(points)

        # Normalizar colores a [0, 1]
        normalized_colors = colors / 255.0
        point_cloud.point_data['Colors'] = normalized_colors

        # Agrego la nube de puntos al renderizador
        self.plotter.add_mesh(point_cloud, scalars='Colors', render_points_as_spheres=True, point_size=5)

        # self.plotter.reset_camera()
        self.plotter.update()

class DrawOutput2D(QWidget):

    def __init__(self):
        super().__init__()

        self.layout = QVBoxLayout(self)

        # Creo una instancia de FigureCanvas
        self.canvas = FigureCanvas(Figure(figsize=(5, 3)))
        self.layout.addWidget(self.canvas)

        # Inicializo el gráfico
        self.ax = self.canvas.figure.add_subplot(111)
        self.scatter = None  # Inicializar como None para usarlo en updatePlot
        self.ax.grid(True)
        self.ax.set_title('Nube de puntos en el plano X-Z')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Z')
        # self.ax.legend(loc='lower left')


    def updatePlot(self,points,colors):
        # Limpio el gráfico anterior
        self.ax.clear()
        self.ax.grid(True)
        self.ax.set_title('Nube de puntos en el plano X-Z')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Z')

        xCoord = points[:,0]
        yCoord = points[:,2]

        # Creo el gráfico de dispersión con los datos de los puntos
        self.scatter = self.ax.scatter(xCoord, yCoord,s=1, c='b') # c=colors / 255.0)#)

        # Ajusto los límites si es necesario
        # self.ax.set_xlim([x_coords.min(), x_coords.max()])
        # self.ax.set_ylim([z_coords.min(), z_coords.max()])

        lim_min, lim_max = -3,3
        # self.ax.set_xlim([lim_min,lim_max])
        # self.ax.set_ylim([lim_min,lim_max])
        # self.ax.set_xlim([-3,3])
        # self.ax.set_ylim([0, 5])
        # Actualizar el lienzo del gráfico
        self.canvas.draw()