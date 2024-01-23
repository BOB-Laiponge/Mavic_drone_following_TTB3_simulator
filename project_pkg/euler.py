import numpy as np

def euler_from_quaternion(qx, qy, qz, qw):
    roll = np.arctan2(2*(qw*qx + qy*qz), 1-2*(qx**2+qy**2))
    pitch = np.arcsin(2*(qw*qy-qz*qx))
    yaw = np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy**2 + qz ** 2))
    return [roll,yaw,pitch]