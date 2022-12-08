import numpy as np
a = np.array([0, 1, 0]).T
b = np.array([-0.72574764, -0.01228586,  0.68785131]).T
d = np.dot(a,b)
print(d)
print(np.arccos(d))
print(np.arccos(d)*180/3.141592)
# a = np.array([0, 0, 1]).T
# b = np.array([0, 1, 0]).T
# cross = np.cross(a,b)
# # print(cross)
# s = np.linalg.norm(cross)
# c = np.dot(a,b)
# # print(c)
# skew_sym = np.array([
#     [0, -cross[2], cross[1]],
#     [cross[2], 0, -cross[0]],
#     [-cross[1], cross[0], 0]])
# print(skew_sym)
# r = np.eye(3) + skew_sym + skew_sym**2*((1-c)/(s**2))
# print(r)
