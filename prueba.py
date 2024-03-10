from getFrame import GetFrame
from reactivex import operators as ops
import cv2

# Crear una instancia de la clase GetFrame
# Puedes ajustar los parámetros según sea necesario
get_frame_instance = GetFrame(vid="your_camera_id", resolution=1, fps=30)

# Obtener el Subject
subject_get_frame = get_frame_instance.getSubjectGetFrame()

# Suscribirse al Subject para recibir los frames
subscription = subject_get_frame.pipe(
    ops.map(lambda frames: frames),  # Ajusta el mapeo según tus necesidades
).subscribe(
    on_next=lambda frames: procesar_y_mostrar(frames),
    on_error=lambda e: print(f"Error: {e}"),
    on_completed=lambda: print("Stream completado"),
)

# Iniciar el stream de frames
get_frame_instance.startStream()

# Puedes realizar otras operaciones aquí si es necesario

# Detener el stream cuando sea necesario
# get_frame_instance.stopStream()

# Después de hacer todo lo necesario, asegúrate de cerrar la suscripción
subscription.dispose()

# Función para procesar y mostrar los frames
def procesar_y_mostrar(frames):
    # Realiza cualquier procesamiento necesario antes de mostrar los frames
    frame_left, frame_right = frames
    # Puedes utilizar cv2.imshow u otra lógica según tus necesidades
    cv2.imshow("Frame izquierdo", frame_left)
    cv2.imshow("Frame derecho", frame_right)
    cv2.waitKey(1)  # Ajusta el valor de espera según sea necesario

# Asegúrate de liberar los recursos cuando hayas terminado
cv2.destroyAllWindows()