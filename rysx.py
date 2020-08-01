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
from flow import StateMachine

print(sys.executable)
isEmptyData = False
mav = PX4MavCtrl.PX4MavCtrler(20100)
mav.InitMavLoop()

lastTime = time.time()
startTime = time.time()
timeInterval = 0.01  #定时时钟，这里是0.01s执行一次

# parameters
width = 720
height = 405
channel = 4

def window_enumeration_handler(hwnd, window_hwnds):
    if win32gui.GetClassName(hwnd) == "UnrealWindow":
        window_hwnds.append(hwnd)

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



# 主循环
mav.initOffboard()
time.sleep(0.5)
mav.SendMavArm(True) # 解锁命令

cnt = 0
car_velocity = 10
geo_fence = [-10,-10,200,10]   # 左下右上
sm = StateMachine(geo_fence)
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

    # 刷新皮卡位置和获取飞机位置速度
    car_pos = [3 + cnt * timeInterval * car_velocity, 0, 0]
    car_vel = [car_velocity, 0, 0]
    car_yaw = 0
    mav.sendUE4Pos(100, 52, 0, car_pos, [0,0,np.pi/2])
    mav_pos = mav.uavPosNED
    mav_vel = mav.uavVelNED
    mav_yaw = mav.uavAngEular[2]
    # print("car_vel: {}\nmav_vel: {}\n".format(car_vel, mav_vel))

    # 量测数据
    dlt_pos = np.array(car_pos) - np.array(mav_pos)
    dlt_vel = np.array(car_vel) - np.array(mav_vel)
    dlt_yaw = car_yaw - mav_yaw

    if mav.ch5>=1:
        mav.endOffboard()
        time.sleep(1)
    # else:
    #     mav.initOffboard()
    keys = [mav.ch5, mav.ch6, mav.ch9, mav.ch10]
    is_initialize_finish = True
    pos_info = {"mav_pos": mav_pos, "mav_vel": mav_vel, "mav_yaw": mav_yaw, "home_pos": [0,0,0-2], "rel_pos": dlt_pos, "rel_vel": dlt_vel, "rel_yaw": dlt_yaw}
    cmd = sm.update(keys, is_initialize_finish, pos_info, car_velocity)
    print(sm.state_name)
    print(cmd)
    if cmd is not None:
        if cmd == "failed":
            for i in range(10):
                mav.SendVelNED(0,0,0,0)
                time.sleep(0.01)
            mav.endOffboard()
            time.sleep(1)
            mav.stopRun()
            break
        else:
            mav.SendVelNED(cmd[0], cmd[1], cmd[2], cmd[3])