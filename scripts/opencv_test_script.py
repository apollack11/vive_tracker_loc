import numpy as np
import cv2
import contextlib

@contextlib.contextmanager
def printoptions(*args, **kwargs):
    orig = np.get_printoptions()
    np.set_printoptions(*args, **kwargs)
    yield
    np.set_printoptions(**orig)

def se3print(g):
    with printoptions(formatter={'float': '{: 0.3f}'.format}, suppress=True):
        print g
    return

meas = np.asarray([[-0.034919, -0.428914], [-0.040377, -0.426840], [-0.033081, -0.413881], [-0.021826, -0.404873], [-0.010666, -0.413834], [-0.003464, -0.426832], [-0.008891, -0.428867]])
body_pts = np.asarray([[-0.028940, 0.056440, 0.088339], [-0.041441, 0.050931, 0.088416], [-0.025132, 0.025347, 0.068624], [-0.000110, -0.006894, 0.088114], [0.025292, 0.025325, 0.068651], [0.041290, 0.050926, 0.088467], [0.029076, 0.056476, 0.088285]])

npts = meas.shape[0]

########################
# ESTIMATE OBJECT POSE #
########################
# now let's calculate the image coordinates of the points (assuming that the
# "center" of the image is at (0,0) and the focal length is 1)
img_pts = np.tan(meas)

# note as described in the notes linked below, we cannot use a naked slice to
# extract points, so let's build the point locations in the body frame
# http://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
body_pts = np.ascontiguousarray(body_pts[:,0:3])

# solve the PnP problem to estimate object location in camera frame:
retval, rvec, tvec = cv2.solvePnP(body_pts, img_pts, np.eye(3), np.zeros(4))

body_pts_cont = np.ascontiguousarray(body_pts).astype(np.float32)
img_pts_cont = np.ascontiguousarray(img_pts, dtype=np.float32).reshape(npts,1,2)
rvec_ran, tvec_ran, inliers = cv2.solvePnPRansac(body_pts_cont, img_pts_cont, np.eye(3), np.zeros(4), iterationsCount=100, reprojectionError=0.1)

try:
    inliers = inliers.ravel()
except:
    pass

# print results
if retval:
    # print "Input SE(3):"
    # se3print(g_co)
    print "\r\n","Output SE(3) using PnP:"
    # convert rotation vector to a rotation Matrix
    R,_ = cv2.Rodrigues(rvec)
    # build SE(3)
    g_est = np.vstack((np.hstack((R, tvec)),[0,0,0,1]))
    se3print(g_est)
    print "\r\n","Output SE(3) using PnP w/ RANSAC:"
    # convert rotation vector to a rotation Matrix
    R,_ = cv2.Rodrigues(rvec_ran)
    # build SE(3)
    g_est_ran = np.vstack((np.hstack((R, tvec_ran)),[0,0,0,1]))
    se3print(g_est_ran)
    print "\ndetected inliers:", inliers
