import pandas as pd
import numpy as np

# Load track
track = pd.read_csv('/home/fsai/Desktop/IPG_2025Modified/formula-carmaker-fs_2024Modifiednextold/formula-carmaker-fs_2024/FCM_Projects/FS_autonomous/ros/ros2_ws/src/MPC_CONTROLLERRRR/track.csv', header=None, names=['x', 'y'])

# Calculate dx and dy
dx = np.gradient(track['x'])
dy = np.gradient(track['y'])
ds = np.sqrt(dx**2 + dy**2)

# Calculate second derivatives
ddx = np.gradient(dx)
ddy = np.gradient(dy)

# Calculate curvature kappa = |dx*ddy - dy*ddx| / ds^3
kappa = np.abs(dx * ddy - dy * ddx) / (ds**3 + 1e-8)

# Find top 5 points with max curvature
max_kappa_indices = np.argsort(kappa)[-5:][::-1]

# Calculate cumulative arc length s
s = np.cumsum(ds)

print("Points with maximum curvature (Hairpins):")
for i in max_kappa_indices:
    print(f"Index: {i}, s: {s[i]:.2f}, x: {track['x'][i]:.2f}, y: {track['y'][i]:.2f}, kappa: {kappa[i]:.4f}")
