# Echo client program
import socket
import GobalV
import time
import serial
import struct
import numpy as np
import math


def AxisAngle_Matrix(a):
    x = a[0]
    y = a[1]
    z = a[2]
    rx = a[3]
    ry = a[4]
    rz = a[5]
    theta = ((rx ** 2) + (ry ** 2) + (rz ** 2)) ** 0.5
    ct = math.cos(theta)
    st = math.sin(theta)
    vt = 1.0 - math.cos(theta)
    rxt = a[3] / theta
    ryt = a[4] / theta
    rzt = a[5] / theta
    RKT = np.array([[rxt * rxt * vt + ct, rxt * ryt * vt - rzt * st, rxt * rzt * vt + ryt * st],
                    [rxt * ryt * vt + rzt * st, ryt * ryt * vt + ct, ryt * rzt * vt - rxt * st],
                    [rxt * rzt * vt - ryt * st, ryt * rzt * vt + rxt * st, rzt * rzt * vt + ct]])
    T = np.array([[RKT[0][0], RKT[0][1], RKT[0][2], x], [RKT[1][0], RKT[1][1], RKT[1][2], y],
                  [RKT[2][0], RKT[2][1], RKT[2][2], z], [0.0, 0.0, 0.0, 1.0]])
    return T


def posetrans(a, b):
    H = np.dot(AxisAngle_Matrix(a), AxisAngle_Matrix(b))
    # print(H)
    theta2=math.acos((H[0,0]+H[1,1]+H[2,2]-1)/2)
    Kx2=1/(2*math.sin(theta2))*(H[2,1]-H[1,2])
    Ky2 = 1 / (2 * math.sin(theta2)) * (H[0, 2] - H[2, 0])
    Kz2 = 1 / (2 * math.sin(theta2)) * (H[1, 0] - H[0, 1])
    rx2=Kx2*theta2
    ry2 = Ky2 * theta2
    rz2 = Kz2 * theta2
    return (H[0, 3], H[1, 3], H[2, 3],rx2,ry2,rz2)


def MoveL(p_x, p_y, p_z, acc, v):
    Plane_1 = [0.4294586340233349, -0.09334204111928597, 0.050197672609858926, -0.7233226016067035, 1.7479512130741546,
               -1.7693452515434942]
    Point_1 = [p_x, p_y, p_z, 0.027244604447062323,
               -0.008540681104123713, 0.22443098148683288]
    temp_h = posetrans(Plane_1, Point_1)
    Point_Base_Str = 'p[{},{},{},{},{},{}]'.format(temp_h[0], temp_h[1], temp_h[2],temp_h[3], temp_h[4], temp_h[5])
    print('基坐标系位姿将运行至：' + str(temp_h))
    #Point_Base_Str = 'p[{},{},{},0.7967,-2.7400,2.5964]'.format(temp_h[0], temp_h[1], temp_h[2])
    strI = 'movel({0}, a={1}, v={2})\n'.format(Point_Base_Str, acc, v)
    s.send(strI.encode())

    temp_s = [0, 0, 0]
    while abs(temp_h[0] - temp_s[0]) > 0.0001 or abs(temp_h[1] - temp_s[1]) > 0.0001 or abs(
            temp_h[2] - temp_s[2]) > 0.0001:
        temp_s = tx_socket()[0:3]
    # print('机器人到位')


def tx_socket():
    dic = {'MessageSize': 'i', 'Time': 'd', 'q target': '6d', 'qd target': '6d', 'qdd target': '6d',
           'I target': '6d',
           'M target': '6d', 'q actual': '6d', 'qd actual': '6d', 'I actual': '6d', 'I control': '6d',
           'Tool vector actual': '6d', 'TCP speed actual': '6d', 'TCP force': '6d', 'Tool vector target': '6d',
           'TCP speed target': '6d', 'Digital input bits': 'd', 'Motor temperatures': '6d', 'Controller Timer': 'd',
           'Test value': 'd', 'Robot Mode': 'd', 'Joint Modes': '6d', 'Safety Mode': 'd', 'empty1': '6d',
           'Tool Accelerometer values': '3d',
           'empty2': '6d', 'Speed scaling': 'd', 'Linear momentum norm': 'd', 'SoftwareOnly': 'd',
           'softwareOnly2': 'd',
           'V main': 'd',
           'V robot': 'd', 'I robot': 'd', 'V actual': '6d', 'Digital outputs': 'd', 'Program state': 'd',
           'Elbow position': '3d', 'Elbow velocity': '3d'}
    data = s.recv(1108)
    names = []
    ii = range(len(dic))
    for key, i in zip(dic, ii):
        fmtsize = struct.calcsize(dic[key])
        data1, data = data[0:fmtsize], data[fmtsize:]
        fmt = "!" + dic[key]
        names.append(struct.unpack(fmt, data1))
        dic[key] = dic[key], struct.unpack(fmt, data1)
    # print(names)
    # print(dic)
    return (dic['Tool vector actual'][1])


Ser1 = serial.Serial(port="COM3", baudrate=115200, timeout=0.5)
for i in range(10):
    data = Ser1.readline()

HOST = "192.168.0.2"
PORT = 30003
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

MoveL(0.128275,-0.161683,-0.134374, 0.5, 0.2)

MoveL(GobalV.get_value('rx') / 1000, GobalV.get_value('ry') / 1000, -0.13437, 0.1, 0.2)
time.sleep(1)
MoveL(GobalV.get_value('rx') / 1000, GobalV.get_value('ry') / 1000, -0.04037, 0.1, 0.1)
offs = 0
while (True):
    data_s = Ser1.readline()
    data_s = str(data_s)
    data_s = data_s.lstrip('b\'').rstrip("\\r\\n'")
    data_s = float(data_s)
    print('压力信号' + str(data_s))
    offs += 0.003
    MoveL(GobalV.get_value('rx') / 1000, GobalV.get_value('ry') / 1000, -0.04037 + offs, 0.1, 0.2)
    if abs(offs) > 0.1:
        print('【提示】检测到压力，退出进给循环')
        break



MoveL((GobalV.get_value('rx') / 1000)+0.010, (GobalV.get_value('ry') / 1000)+0, -0.04037 + offs, 0.5, 0.01)
MoveL((GobalV.get_value('rx') / 1000)+0.010, (GobalV.get_value('ry') / 1000)+0.010, -0.04037 + offs, 0.5, 0.01)
MoveL((GobalV.get_value('rx') / 1000)+0, (GobalV.get_value('ry') / 1000)+0.010, -0.04037 + offs, 0.5, 0.01)
MoveL((GobalV.get_value('rx') / 1000)-0.010, (GobalV.get_value('ry') / 1000)+0, -0.04037 + offs, 0.5, 0.01)
MoveL((GobalV.get_value('rx') / 1000)-0.010, (GobalV.get_value('ry') / 1000)-0.010, -0.04037 + offs, 0.5, 0.01)
MoveL((GobalV.get_value('rx') / 1000)+0, (GobalV.get_value('ry') / 1000)+0, -0.04037 + offs, 0.5, 0.01)
time.sleep(1)
MoveL(GobalV.get_value('rx') / 1000, GobalV.get_value('ry') / 1000, -0.04037, 0.1, 0.2)
MoveL(GobalV.get_value('rx') / 1000, GobalV.get_value('ry') / 1000, -0.13437, 0.1, 0.3)
print('【提示】已完成上呼吸道样本采集！')
s.close()
