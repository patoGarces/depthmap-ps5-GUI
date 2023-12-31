import sys
import cv2
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

class VideoWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.video_label = QLabel()  # Etiqueta para mostrar el video
        layout = QVBoxLayout()  # Diseño vertical
        layout.addWidget(self.video_label)
        self.setLayout(layout)
        
        # Iniciar la captura de video
        self.video_capture = cv2.VideoCapture(0)  # 0 para la cámara predeterminada
        
        # Iniciar el temporizador para actualizar el video
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # Actualizar cada 30 milisegundos

    def update_frame(self):
        ret, frame = self.video_capture.read()  # Capturar un fotograma de la cámara
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convertir el fotograma a RGB
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            convert_to_qt_format = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(convert_to_qt_format)
            pixmap = pixmap.scaled(self.video_label.width(), self.video_label.height(), Qt.KeepAspectRatio)
            self.video_label.setPixmap(pixmap)  # Mostrar el fotograma en el QLabel

    def closeEvent(self, event):
        self.video_capture.release()  # Liberar la captura de la cámara al cerrar la aplicación

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = QWidget()
    video_widget = VideoWidget()
    window.setLayout(QVBoxLayout())
    window.layout().addWidget(video_widget)
    window.setWindowTitle("Video de la cámara en PyQt5")
    window.show()
    sys.exit(app.exec_())