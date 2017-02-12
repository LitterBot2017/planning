import numpy as np

def homography(x,y):
    h_mat = np.array([[2.20112274361411e-05, -0.00104907565122149, 0.716837052934665],
            [0.00106749217814106, 2.52155668263140e-05, -0.697069869544055],
            [-1.14045687493388e-07, 3.65086924461742e-07, 0.0153621383290128]])

    point = np.array([[x],[y],[1]])

    q = np.dot(h_mat,point)
    real_x = q[0]/q[2]
    real_y = q[1]/q[2]

    return float(real_x), float(real_y)

print homography(216,142)