import numpy as np

# define a set of points
points = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])

# compute the singular value decomposition of the points
U, S, V = np.linalg.svd(points)

# the slope of the plane is the ratio of the first and second elements
# of the last column of V
print(V)
slope = V[-1, 0] / V[-1, 1]

# print the slope
print(slope)