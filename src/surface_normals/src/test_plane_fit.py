import numpy as np
import matplotlib.pyplot as plt

# generate some random test points 
m = 20000 # number of points
delta = 0.01 # size of random displacement
origin = np.random.rand(3, 1) # random origin for the plane
basis = np.random.rand(3, 2) # random basis vectors for the plane
coefficients = np.random.rand(2, m) # random coefficients for points on the plane

# generate random points on the plane and add random displacement
points = basis @ coefficients \
         + np.tile(origin, (1, m)) \
         + delta * np.random.rand(3, m)

# now find the best-fitting plane for the test points

# subtract out the centroid and take the SVD
svd = np.linalg.svd(points - np.mean(points, axis=1, keepdims=True))

# Extract the left singular vectors
left = svd[0]
norm_vec = left[:, -1]


print(norm_vec)
print(norm_vec @ basis)