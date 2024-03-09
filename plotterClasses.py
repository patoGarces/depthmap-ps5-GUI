import numpy as np
import matplotlib.pyplot as plt
import cv2

class DrawOutput3D():
    
    def __init__(self) -> None:
        self.fig2 = plt.figure(figsize=(7,7))
        self.ax_3d = self.fig2.add_subplot(111, projection='3d')
        self.ax_3d.set_title("Nube de puntos 3d")
        self.ax_3d.set_zlabel('Distancia[MM]')    

        self.ax_3d.view_init(elev=-58., azim=-134.,roll = 45.)                                        

        self.grafico1 = self.ax_3d.scatter(0,0)
        
    def updatePointCloud(self,frameDepth,frameRgb):
        # frameBGR = cv2.cvtColor(frameRgb,cv2.COLOR_RGB2BGR)                         # Necesito los colores en formato BGR
        # color3d = frameBGR[:480][:640]                                               # Necesito una linea horizontal del frame, tomo una a la altura de X                                              
        # colorBGR= color3d/255                                                     # Para plotear el punto, el color debe estar normalizado como un float entre 0 y 1,EJEMPLO:[0,127,255] -> [0,0.5,1.00]
        # mapDepth = frameDepth[:480][:640]

        (width, height, b) = np.shape(frameRgb)
        paso = 10
        (X, Y) = np.meshgrid(range(0, height, paso), range(0,width, paso))

        self.grafico1.remove()
        self.grafico1 = self.ax_3d.scatter(X, Y, frameDepth[Y,X])#,c = colores)

class DrawOutput2D():
    
    def __init__(self) -> None:

        # Subplots 2d plot y radar
        self.fig1, (self.ax_plot, self.ax_radar) = plt.subplots(nrows=2,layout="constrained")

        # Subplot plot
        self.ax_plot.set_title('Grafico plot')
        self.ax_plot.set_ylabel('Distancia[mm]')
        self.plots1, = self.ax_plot.plot(0,0)
        self.ax_plot.grid()
        # Subplot radar
        self.ax_radar.set_title('Grafico radar')
        self.ax_radar.set_ylabel('Distancia[mm]')
        self.plots2, = self.ax_radar.plot(0,0)
        self.ax_radar.grid()
        
        # # Subplots roll y pitch
        # self.fig2, (self.ax_roll, self.ax_pitch) = plt.subplots(nrows=2,layout="constrained")
        
        # # Subplot pitch
        # self.ax_pitch.set_title('Angulo pitch')
        # self.ax_pitch.set_ylabel('Grados')
        # self.ax_pitch.set_xlabel('Frame')
        # self.ax_pitch.set_ylim(-100,100)
        # self.ax_pitch.grid()
        # self.plotsPitch, = self.ax_pitch.plot(0,0)

        # # Subplot roll
        # self.ax_roll.set_title('Angulo roll')
        # self.ax_roll.set_ylabel('Grados')
        # self.ax_roll.set_xlabel('Frame')
        # self.ax_roll.set_ylim(-100,100)
        # self.ax_roll.grid()
        # self.plotsRoll, = self.ax_roll.plot(0,0)
    
    def addSubplot(self,data):                                        
        self.plots1.remove()                                                        # Limpio solamente los puntos realizados anteriormente,es mas performante
        self.plots1 = self.ax_plot.stem(data,linefmt='--',markerfmt=",")

    def updateRadar(self,frameDepth,frameRgb,X):
        
        frameBGR = cv2.cvtColor(frameRgb,cv2.COLOR_RGB2BGR)                         # Necesito los colores en formato BGR
        colorLine = frameBGR[X][:640]                                               # Necesito una linea horizontal del frame, tomo una a la altura de X                                              
        colorBGR= colorLine/255                                                     # Para plotear el punto, el color debe estar normalizado como un float entre 0 y 1,EJEMPLO:[0,127,255] -> [0,0.5,1.00]

        mapDepth = frameDepth[X][:640]
        x = np.arange(0,640,1)                                                      # El ancho de la imagen a iterar

        self.plots2.remove()                                                        # Limpio solamente los puntos realizados anteriormente,es mas performante
        self.plots2 = self.ax_radar.scatter(x,mapDepth,s=1,c=colorBGR)              # Dibujo cada punto con maxDepth en el eje Y del grafico,tamaño 1 y con su respectivo color
        return

    # def addAccPlot(self,roll,pitch,n):
    #     # self.plotsRoll.remove()                                                        # Limpio solamente los puntos realizados anteriormente,es mas performante
    #     # self.plotsPitch.remove()
    #     self.plotsRoll = self.ax_roll.stem(n,roll)
    #     self.plotsPitch = self.ax_pitch.stem(n,pitch)

    def showPlot(self):
        # plt.ion()
        plt.show()
        plt.pause(0.0001)
