import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread


class MyMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.canvas = FigureCanvas(Figure(figsize=(5, 3)))
        self.layout.addWidget(self.canvas)

        self.plot_button = QPushButton("Plot", self)
        self.plot_button.clicked.connect(self.plot)
        self.layout.addWidget(self.plot_button)

    def plot(self):
        t = np.linspace(0, 2 * np.pi, 100)
        y = np.sin(t)

        ax = self.canvas.figure.add_subplot(111)
        ax.plot(t, y)
        ax.set_title('Sine Wave')

        self.canvas.draw()


def main():
    app = QApplication(sys.argv)
    main_window = MyMainWindow()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()