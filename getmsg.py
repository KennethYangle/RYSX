import time
import mmap
import numpy as np
import cv2.cv2 as cv2
from pymavlink.dialects.v20 import common as mavlink2

import win32gui, win32ui, win32con
from ctypes import windll

import sys
import PX4MavCtrlV4 as PX4MavCtrl

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

