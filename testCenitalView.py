import pyvista as pv

# Cargar el archivo .ply
point_cloud = pv.read("point_cloud.ply")

# Crear el plotter y añadir la nube de puntos con los colores del archivo
plotter = pv.Plotter()
plotter.add_mesh(point_cloud, scalars="RGB", rgb=True, point_size=1)  # Usa "RGB" para los colores en el archivo

# Configurar los ejes y mostrar la nube de puntos
plotter.add_axes()
plotter.show()
