import numpy as np

lidarSignals = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8])


lidars = np.reshape(lidarSignals, (3,3))

smallest_in_regions = np.min(lidars, axis=1)   # Tìm phần tử nhỏ nhất trong mỗi hàng (mỗi vùng)

print(smallest_in_regions)

