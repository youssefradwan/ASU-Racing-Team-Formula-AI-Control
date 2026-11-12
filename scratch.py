import pandas as pd
import numpy as np

# track.csv columns might not have 's' explicitly
track = pd.read_csv('track.csv', header=None)
# Let's assume columns are x, y, w_right, w_left
x = track.iloc[:, 0].values
y = track.iloc[:, 1].values

dx = np.diff(x)
dy = np.diff(y)
ds = np.sqrt(dx**2 + dy**2)
s = np.concatenate(([0], np.cumsum(ds)))

# Find index where s is around 271.7
idx = np.searchsorted(s, 271.7)
start_idx = max(0, idx - 10)
end_idx = min(len(s), idx + 10)

print(f"Track around s = 271.7 (index {idx}):")
for i in range(start_idx, end_idx):
    # calculate rough curvature
    if i > 0 and i < len(x) - 1:
        x1, y1 = x[i-1], y[i-1]
        x2, y2 = x[i], y[i]
        x3, y3 = x[i+1], y[i+1]
        
        area = x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2)
        dist1 = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        dist2 = np.sqrt((x2-x3)**2 + (y2-y3)**2)
        dist3 = np.sqrt((x3-x1)**2 + (y3-y1)**2)
        
        if dist1*dist2*dist3 == 0:
            kappa = 0
        else:
            kappa = 4 * area / (dist1 * dist2 * dist3)
    else:
        kappa = 0
        
    print(f"i={i}, s={s[i]:.2f}, x={x[i]:.2f}, y={y[i]:.2f}, kappa={kappa:.4f}")
