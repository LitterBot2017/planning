import numpy as np

def homography(x,y):
    h_mat = np.array([[2.20112274361046e-05, -0.00104907565122139, 0.716837052934637], 
            [-0.00106749217814114, -2.52155668261608e-05, 0.697069869544084], 
            [-1.14045687505024e-07, 3.65086924495886e-07, 0.0153621383290117]])

    point = np.array([[x],[y],[1]])

    q = np.dot(h_mat,point)
    real_x = q[0]/q[2]
    real_y = q[1]/q[2]

    return float(real_x), float(real_y)

# print homography(216,142)
print homography(497,400)