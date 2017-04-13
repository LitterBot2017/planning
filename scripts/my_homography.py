import string
import math
import numpy as np

def homography(x,y):
    # h_mat = np.array([[2.20112274361046e-05, -0.00104907565122139, 0.716837052934637], 
    #         [-0.00106749217814114, -2.52155668261608e-05, 0.697069869544084], 
    #         [-1.14045687505024e-07, 3.65086924495886e-07, 0.0153621383290117]])

    # h_mat = np.array([[5.46746420977118e-06, 0.00186858537929480, -0.770939373319039],
    #                   [0.00187846177904567, -6.78829638396309e-06, -0.636755466418008], 
    #                   [-3.69398387440614e-07, 2.44271999503352e-07, -0.0137090607660856]])

    h_mat = np.array([[-0.000109081062020685, -0.00179116587204159, 0.864555067791766], 
                        [-0.00175609183828732, 0.000101439555606069, 0.502352545883134], 
                        [-1.76373377005859e-08, -1.42370718069173e-06, 0.0134216271701275]])

    # h_mat = np.array([[0.000192002622251096, 0.00308145301657239, -0.114684138090243], 
    #                     [0.00302882040810846, -0.000169892835332421, -0.993148051435149], 
    #                     [2.40497923689312e-07, 1.88279634116435e-06, 0.0220399991633980]])        

    point = np.array([[x],[y],[1]])
    q = np.dot(h_mat,point)
    print q
    real_x = q[0]/q[2]
    real_y = q[1]/q[2]

    return float(real_x), float(real_y)

if __name__=='__main__':
    input_string = raw_input('Please enter the pixel coordinates deliminated by spaces-->')
    xy_vals = string.split(input_string,' ')
    if len(xy_vals) < 2:
        print 'TOO FEW NUMBERS FOR COORDINATES IN 2D!!'
    else:
        x,y = homography(float(xy_vals[0]), float(xy_vals[1]))
        print "The following are the calculated values for the position"
        # print x,y coordinates
        x = x*0.875
        y = y*0.875
        print repr(x) + ' ' + repr(y)