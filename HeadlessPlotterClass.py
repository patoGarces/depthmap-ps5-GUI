import matplotlib.pyplot as plt


class DrawOutput2DHeadless():

    def __init__(self):
        super().__init__()

        # Inicializo el gráfico
        self.fig, self.ax = plt.subplots(figsize=(5, 3))
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

        # lim_min, lim_max = -3,3
        # self.ax.set_xlim([lim_min,lim_max])
        # self.ax.set_ylim([lim_min,lim_max])
        # self.ax.set_xlim([-3,3])
        # self.ax.set_ylim([0, 5])
        # Actualizar el lienzo del gráfico
        plt.draw()
        plt.pause(0.0001)  