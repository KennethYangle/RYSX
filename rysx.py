import time
import mmap
import numpy as np
import cv2.cv2 as cv2
from pymavlink.dialects.v20 import common as mavlink2

import win32gui, win32ui, win32con
from ctypes import windll

import sys
import PX4MavCtrlV4 as PX4MavCtrl
from fusion import KF

print(sys.executable)
isEmptyData = False
mav = PX4MavCtrl.PX4MavCtrler(20100)
mav.InitMavLoop()

lastTime = time.time()
startTime = time.time()
timeInterval = 0.01  #定时时钟，这里是0.01s执行一次
flag = 0
# parameters
width = 720
height = 405
channel = 4
min_prop = 0.000001
max_prop = 0.3
K_z = 0.001 * 640 / height
K_yawrate = 0.001 * 480 / width


def window_enumeration_handler(hwnd, window_hwnds):
    if win32gui.GetClassName(hwnd) == "UnrealWindow":
        window_hwnds.append(hwnd)

def sat(a):
    if np.linalg.norm(a) > 1:
        return a / np.linalg.norm(a)
    return a

#设置空白草地地形，两个窗口都设置，第一个窗口用于摄像头图像读取，第二个窗口用于效果预览
mav.sendUE4Cmd(b'RflyChangeMapbyName VisionRingBlank')
time.sleep(1)
#新建一个皮卡，设置位置和坐标，使用默认样式红色
mav.sendUE4Pos(100,52,0,[3,0,0],[0,0,np.pi/2])
time.sleep(0.5)
#将焦点切换到飞机1
mav.sendUE4Cmd(b'RflyChangeViewKeyCmd B 1',0)
time.sleep(0.5)  
#将视角切换到1视角（前置摄像头）
mav.sendUE4Cmd(b'RflyChangeViewKeyCmd V 1',0)
time.sleep(0.5)
#设置前置摄像头位置在相对机体中心[0.3,0,0.05]的地方
mav.sendUE4Cmd(b'RflyCameraPosAng 0.3 0 0.05 0 0 0',0)
# mav.sendUE4Cmd(b'RflyCameraPosAng 0.3 0 0.05 0 60.00 0',0)
time.sleep(0.5)    
#设置前置摄像头画面为720x405
mav.sendUE4Cmd(b'r.setres 720x405w',0)   


time.sleep(2)
window_hwnds = []
win32gui.EnumWindows(window_enumeration_handler, window_hwnds)
if len(window_hwnds)>0:
    print(window_hwnds)
    window_hwnds.sort()
    hWnd = window_hwnds[0]
    print(hWnd)

#hWnd = win32gui.FindWindow("UnrealWindow", None)

left, top, right, bot = win32gui.GetClientRect(hWnd)
windows_width = right - left
windows_height = bot - top

if hWnd and windows_width != width:
    hWnd = window_hwnds.sort(reverse=True)
    hWnd = window_hwnds[0]
    left, top, right, bot = win32gui.GetClientRect(hWnd)
    windows_width = right - left
    windows_height = bot - top
    print("Not the desired window.")

if hWnd and windows_width == 0 and windows_height == 0:
    print("The UE4 window cannot be in minimum mode.")
    sys.exit(1)


# 状态参数
dt = timeInterval
dt_INS = timeInterval
dt_Vis = 2*timeInterval
F = np.array([[1,0,0, dt,0,0, 0.5*dt*dt,0,0, 0,0,0],
              [0,1,0, 0,dt,0, 0,0.5*dt*dt,0, 0,0,0],
              [0,0,1, 0,0,dt, 0,0,0.5*dt*dt, 0,0,0],
              [0,0,0, 1,0,0, dt,0,0, 0,0,0],
              [0,0,0, 0,1,0, 0,dt,0, 0,0,0],
              [0,0,0, 0,0,1, 0,0,dt, 0,0,0],
              [0,0,0, 0,0,0, 1,0,0, 0,0,0],
              [0,0,0, 0,0,0, 0,1,0, 0,0,0],
              [0,0,0, 0,0,0, 0,0,1, 0,0,0],
              [0,0,0, 0,0,0, 0,0,0, 1,0,0],
              [0,0,0, 0,0,0, 0,0,0, 0,1,0],
              [0,0,0, 0,0,0, 0,0,0, 0,0,1]
              ])
B = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],
              [0,0,0,0],[0,0,0,0],[0,0,0,0],
              [0,0,0,0],[0,0,0,0],[0,0,0,0],
              [0,0,0,1],[0,0,0,0],[0,0,0,0]])
Q = np.identity(12) * 0.1
# 观测参数
H1 = np.identity(12)
H2 = np.array([[1,0,0, 0,0,0, 0,0,0, 0,0,0],[0,1,0, 0,0,0, 0,0,0, 0,0,0],[0,0,1, 0,0,0, 0,0,0, 0,0,0]])
R1 = np.diag([0.1,0.1,0.1, 0.1,0.1,0.1, 0.1,0.1,0.1, 0.1,0.1,0.1])
R2 = np.diag([0.01,0.01,0.01])
# 实例化滤波器
H = [H1, H2]
R = [R1, R2]
flt = KF(F, B, H, dt, Q, R)

# 主循环
cnt = 0
car_velocity = 10
P, D = 1, 0.5
u = np.array([0,0,0,0])
while True:
    # 通过sleep保持帧率
    lastTime = lastTime + timeInterval
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime)
    else:
        lastTime = time.time()
    cnt += 1
    #以下代码0.01s执行一次

    # 倾转旋翼，固定相机俯仰视角
    pitch = mav.uavAngEular[1]
    mav.sendUE4Cmd('RflyCameraPosAng 0.3 0 0.05 0 {:.2f} 0'.format(-pitch*180/np.pi).encode('utf8'),0)

    # 刷新皮卡位置和获取飞机位置速度
    car_pos = [3 + cnt * timeInterval * car_velocity, 0, 0]
    car_vel = [car_velocity, 0, 0]
    car_acc = [0, 0, 0]
    car_yaw = 0
    car_yva = [car_yaw, 0, 0]
    mav.sendUE4Pos(100, 52, 0, car_pos, [0,0,np.pi/2])
    mav_pos = mav.uavPosNED
    mav_vel = mav.uavVelNED
    mav_acc = [0, 0, 0]
    mav_yaw = mav.uavAngEular[2]
    mav_yva = [mav_yaw, 0, 0]
    # print("car_vel: {}\nmav_vel: {}\n".format(car_vel, mav_vel))

    # 量测数据
    dlt_pos = np.array(mav_pos) - np.array(car_pos)
    dlt_vel = np.array(mav_vel) - np.array(car_vel)
    dlt_acc = np.array(mav_acc) - np.array(car_acc)
    dlt_yva = np.array(mav_yva) - np.array(car_yva)
    x_gt = np.concatenate((dlt_pos, dlt_vel, dlt_acc, dlt_yva))
    z = dict()
    z[0] = x_gt + np.random.multivariate_normal(np.zeros(12), R1)
    if cnt % 2 == 0:
        z[1] = dlt_pos + np.random.multivariate_normal(np.zeros(3), R2)
    x_k_k = flt.update(sat(u), z)
    # print("x_gt: {}\n x_k_k: {}\n".format(x_gt, x_k_k))

    elapsed_time = time.time() - startTime
    if elapsed_time > 5 and flag == 0:
        #下列代码第5s的时触发一次，发送一个解锁信号，并设定一个前飞目标
        print("5s, Arm the drone")
        mav.initOffboard()
        time.sleep(0.5)
        mav.SendMavArm(True) #解锁命令
        mav.SendPosNED(0, -5, -5, np.pi/3) #飞到目标点 0, -5，-5, pi/3位置，命令范围15s
        flag = 1

    if elapsed_time > 15 and flag == 1:
        flag = 2
        #以下脚本在15s时触发一次
        print("开始任务")

    if elapsed_time > 15:
        u = np.array([car_velocity+P*(car_pos[0]-6-mav_pos[0])+D*(car_vel[0]-mav_vel[0]), P*(car_pos[1]-mav_pos[1]), P*(car_pos[2]-2-mav_pos[2]), P*(car_yaw-mav_yaw)])
        mav.SendVelNED(u[0], u[1], u[2], u[3])

    if elapsed_time > 80:
        #退出Offboard控制模式
        print("Send offboard stop")
        mav.endOffboard()
        time.sleep(1)

        #退出Mavlink数据接收模式
        print("Send Mavlink stop")
        mav.stopRun()
        time.sleep(1)
