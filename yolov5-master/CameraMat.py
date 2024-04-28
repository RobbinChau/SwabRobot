import numpy as np
import cv2
# 通过九点标定获取的圆心相机坐标
STC_points_camera = np.array([
    [827,505],
    [1253,505],
    [1683,507],
    [824,953],
    [1251,954],
    [1679,956],
    [822,1380],
    [1248,1380],
    [1676,1383],
])
# 通过九点标定获取的圆心机械臂坐标
STC_points_robot = np.array([
    [98.55,-161.67],
    [125.01,-160.98],
    [151.82,-160.44],
    [98.04,-133.31],
    [125.03,-132.84],
    [151.85,-132.41],
    [97.54,-106.38],
    [124.58,-106.31],
    [151.55,-105.86],
])

x_camera=515
y_camera=205

m = cv2.estimateAffine2D(STC_points_camera, STC_points_robot)
print(m)
robot_x = (6.28034239e-02 * x_camera) + (-2.41714777e-04 * y_camera) + 4.65194662e+01
robot_y = (8.37900031e-04 * x_camera) + (6.26645056e-02 * y_camera) + -1.93747427e+02
print(robot_x, robot_y)



# [ 6.28034239e-02, -2.41714777e-04,  4.65194662e+01],
# [ 8.37900031e-04,  6.26645056e-02, -1.93747427e+02]