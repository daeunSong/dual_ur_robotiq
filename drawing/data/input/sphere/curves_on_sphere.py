import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Define sphere mesh
theta, phi = np.linspace(0, 2*np.pi, 50), np.linspace(0, np.pi, 25)
theta, phi = np.meshgrid(theta, phi)
x = np.sin(phi) * np.cos(theta)
y = np.sin(phi) * np.sin(theta)
z = np.cos(phi)

# Define maximum and minimum distance between curve points
max_dist = 0.5
min_dist = 0.2

# Define number of short curves
n_curves = 600

# Define number of points in each short curve
n_points = 10

# Plot sphere
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(x, y, z, color='gray', alpha=0.1)

lines = []

# Plot short curves
for i in range(n_curves):
    # Randomly select start point on sphere
    start_theta = np.random.uniform(0, 2*np.pi)
    start_phi = np.random.uniform(0, np.pi)
    start_x = np.sin(start_phi) * np.cos(start_theta)
    start_y = np.sin(start_phi) * np.sin(start_theta)
    start_z = np.cos(start_phi)
    start_vec = np.array([start_x, start_y, start_z])

    # Randomly select direction of curve endpoint
    end_vec = np.random.randn(3)
    end_vec = end_vec / np.linalg.norm(end_vec)

    # Scale direction vector to maximum distance
    dist = np.random.uniform(min_dist, max_dist)
    end_vec = start_vec + dist * end_vec
    end_vec = end_vec / np.linalg.norm(end_vec)

    # Generate curve points
    curve_x = np.linspace(start_vec[0], end_vec[0], n_points)
    curve_y = np.linspace(start_vec[1], end_vec[1], n_points)
    curve_z = np.linspace(start_vec[2], end_vec[2], n_points)

    # Project curve points onto sphere
    norm = np.sqrt(curve_x**2 + curve_y**2 + curve_z**2)
    proj_x = curve_x / norm
    proj_y = curve_y / norm
    proj_z = curve_z / norm

    if min(proj_z) > 0.75 :
        # Plot curve
        ax.plot(proj_x, proj_y, proj_z, color='black', linewidth=1)
        points = np.vstack([proj_x, proj_y, proj_z]).T
        lines.append(points)



# Set box aspect and turn off grid
ax.set_box_aspect([1, 1, 1])
ax.grid(False)

# Show plot
plt.show()

file_name = "curves_on_sphere_9.txt"   # Replace with your file name

center  = [0, 0, 0]
radius = 1.0

f = open(file_name, 'w')
for line in lines:
    for point in line:
        surface_normal = point - center
        surface_normal /= np.linalg.norm(surface_normal)
        surface_normal *= radius

        f.write('%s %s %s %s %s %s\n' % (str(point[0]), str(point[1]), str(point[2]), str(surface_normal[0]), str(surface_normal[1]),str(surface_normal[2])))
    f.write("End\n")
f.close()