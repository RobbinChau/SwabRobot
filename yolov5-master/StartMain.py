import cv2
import keyboard  # using module keyboard
import GobalV
import time


#####################################################################
#     Robbin                                                        #
#         保留区                                                     #
#              Robbin                                               #
#                     Robbin                                        #
#                   保留区                                           #
#                         Robbin                                    #
#                           保留区                                   #
#                                 Robbin                            #
#                                   保留区                           #
#                                         Robbin                    #
#                                           保留区                   #
#                                                 Robbin            #
#                                                      Robbin       #
#                                                        保留区      #
#                                                            Robbin #
#                                                                   #
#####################################################################





GobalV._init()
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
#cap = cv2.VideoCapture(1)
cap.set(3, 2592);
cap.set(4, 1944);
#
#cap.set(3, 1280);
#cap.set(4, 720);

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print('You Clicked LEFT Button!')
        xy = "%d,%d" % (x, y)
        print(xy)

while True:#(cap.isOpened()):
    retval, frame = cap.read()
    cv2.namedWindow("Live", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    #cv2.resizeWindow("Live", 640, 480)
    cv2.imshow('Live', frame)
    cv2.setMouseCallback("Live", on_EVENT_LBUTTONDOWN)
    cv2.waitKey(1)
    if keyboard.is_pressed("SPACE"):  # if key 'SPACE' is pressed
        retval, frame = cap.read()
        path='./yolov5-master/runs/Cap/'
        cv2.imwrite(path+'capture'+'.jpg', frame)  # 写入图片
        print('You Pressed SPACE Key!')
        break   #finishing the loop

import detectII

#img = cv2.imread("yolov5-master/runs/detect/exp33/capture.jpg")
img = cv2.imread("yolov5-master/runs/Cap/capturede.jpg")
#C:\Users\Slytherin\Desktop\pythonProjectFB\yolov5-master\runs\detect\exp33
cv2.namedWindow("Image")
cv2.imshow("Image", cv2.resize(img, (640, 480)))
cv2.waitKey(0)

print(GobalV.get_value('class'),GobalV.get_value('conf'),GobalV.get_value('position'))
x_camera=GobalV.get_value('position')[0]
y_camera=GobalV.get_value('position')[1]
robot_x = (6.28034239e-02 * x_camera) + (-2.41714777e-04 * y_camera) + 4.65194662e+01
robot_y = (8.37900031e-04 * x_camera) + (6.26645056e-02 * y_camera) + -1.93747427e+02

print('【提示】平面特征坐标系坐标为：'+ str(robot_x) +'，'+ str(robot_y) )

GobalV.set_value('rx', robot_x)
GobalV.set_value('ry', robot_y)

import UR_Move
cv2.waitKey(0)  # 1 millisecond


