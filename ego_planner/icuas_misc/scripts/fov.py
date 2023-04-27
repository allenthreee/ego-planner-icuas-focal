import numpy as np

w = 640
h = 480
fx = 381.36246688113556
fy = 381.36246688113556
cx = 320.5
cy = 320.5
fov_x = np.rad2deg(2 * np.arctan2(w, 2 * fx))
fov_y = np.rad2deg(2 * np.arctan2(h, 2 * fy))
print("Field of View (degrees):")

print(fov_x)

print(fov_y)


