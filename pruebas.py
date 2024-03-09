import sys
import numpy as np
from matplotlib import pyplot as plt

from plotterClasses import DrawOutput2D

if __name__ == '__main__':
    draw_output_2d = DrawOutput2D()

    # Simula la obtención de datos
    points = np.random.rand(480, 640)

    try:
        height, width = points.shape[:2]  
        y = 240
        x = np.arange(0, width, 1)
        curvaNivel = points[y][x]
        
        # Llama a los métodos de la instancia de DrawOutput2D
        draw_output_2d.addSubplot(curvaNivel)
        draw_output_2d.showPlot()
        plt.show()

    except Exception as e:
        print("Error:", str(e))

    # Espera que se cierre la ventana
    plt.show()