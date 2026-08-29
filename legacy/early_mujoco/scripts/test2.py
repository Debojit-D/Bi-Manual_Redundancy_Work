import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define a non-convex function with multiple local maxima and minima
def nonconvex_function(x, y):
    return np.sin(3*x) * np.cos(3*y) + 0.5 * (x**2 - y**2)

# Generate grid points
x = np.linspace(-2, 2, 100)
y = np.linspace(-2, 2, 100)
X, Y = np.meshgrid(x, y)
Z = nonconvex_function(X, Y)

# Plot the non-convex function
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, Z, cmap='plasma', alpha=0.8)
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Function Value')
ax.set_title('3D Plot of a Non-Convex Function with Multiple Minima/Maxima')

plt.show()
