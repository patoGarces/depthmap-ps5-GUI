import open3d as o3d
import numpy as np

# Generar tu nube de puntos
points = np.random.rand(100, 3)  # Cambia esto por tus datos reales

# Crear un objeto PointCloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Función para crear flechas que representen los ejes X, Y, Z
def create_axes(length=1.0):
    lines = []
    colors = []

    # Eje X (rojo)
    lines.append([0, 1])
    colors.append([1, 0, 0])  # Rojo

    # Eje Y (verde)
    lines.append([0, 2])
    colors.append([0, 1, 0])  # Verde

    # Eje Z (azul)
    lines.append([0, 3])
    colors.append([0, 0, 1])  # Azul

    points = np.array([
        [0, 0, 0],     # Origen
        [length, 0, 0],  # Eje X
        [0, length, 0],  # Eje Y
        [0, 0, length]   # Eje Z
    ])

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    return line_set

# Crear el conjunto de flechas
axes = create_axes(length=1.0)  # Ajusta la longitud según sea necesario

# Visualizar la nube de puntos y las flechas de ternas
o3d.visualization.draw_geometries([pcd, axes])
