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
    
    
# 返回句柄窗口的设备环境，覆盖整个窗口，包括非客户区，标题栏，菜单，边框
hWndDC = win32gui.GetWindowDC(hWnd)
# 创建设备描述表
mfcDC = win32ui.CreateDCFromHandle(hWndDC)
# 创建内存设备描述表
saveDC = mfcDC.CreateCompatibleDC()
# 创建位图对象准备保存图片
saveBitMap = win32ui.CreateBitmap()
# 为bitmap开辟存储空间
saveBitMap.CreateCompatibleBitmap(mfcDC, windows_width, windows_height)



def procssImage():
    #这里处理图像，并获取到速度控制指令
    saveDC.SelectObject(saveBitMap)
    # 0-保存整个窗口，1-只保存客户区。如果PrintWindow成功函数返回值为1
    result = windll.user32.PrintWindow(hWnd, saveDC.GetSafeHdc(), 1)
    signedIntsArray = saveBitMap.GetBitmapBits(True)
    img_rgba = np.frombuffer(signedIntsArray, dtype='uint8')
    img_rgba.shape = (windows_height, windows_width, 4)
    img_bgr = cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2RGB)
    img_bgr = cv2.resize(img_bgr, (width, height))

    p_i = calc_centroid(img_bgr)
    #print(p_i)
    #cv2.imshow("img", img_bgr)
    #cv2.waitKey(1)

    ctrl = controller(p_i) #返回前进，向右，向下，右偏航速度
    return ctrl

def calc_centroid(img):
    """Get the centroid and area of green in the image"""

    #hue_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low_range = np.array([0,0,80])
    high_range = np.array([100,100,255])
    th = cv2.inRange(img, low_range, high_range)
    dilated = cv2.dilate(th, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
    cv2.imshow("dilated", dilated)
    cv2.waitKey(1)

    M = cv2.moments(dilated, binaryImage=True)
    if M["m00"] >= min_prop*width*height:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return [cx, cy, M["m00"]]
    else:
        return [-1, -1, -1]

def sat(inPwm,thres=1):
    outPwm= inPwm
    for i in range(len(inPwm)):
        if inPwm[i]>thres:
            outPwm[i] = thres
        elif inPwm[i]<-thres:
            outPwm[i] = -thres
    return outPwm

def controller(p_i):
    # if the object is not in the image, search in clockwise
    if p_i[0] < 0 or p_i[1] < 0:
        return [0, 0, 0, 1]

    # found
    ex = p_i[0] - width / 2
    ey = p_i[1] - height / 2

    vx = 2 if p_i[2] < max_prop*width*height else 0
    vy = 0
    vz = K_z * ey
    yawrate = K_yawrate * ex

    return [vx, vy, vz, yawrate]

ctrlLast = [0,0,0,0]
cnt = 0
car_velocity = 10
P, D = 1, 0.5
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
    mav.sendUE4Pos(100, 52, 0, car_pos, [0,0,np.pi/2])
    mav_pos = mav.uavPosNED
    mav_vel = mav.uavVelNED
    car_yaw = 0
    mav_yaw = mav.uavAngEular[2]
    # print("car_vel: {}\nmav_vel: {}\n".format(car_vel, mav_vel))

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
        vel_d = [car_velocity+P*(car_pos[0]-6-mav_pos[0])+D*(car_vel[0]-mav_vel[0]), P*(car_pos[1]-mav_pos[1]), P*(car_pos[2]-2-mav_pos[2]), P*(car_yaw-mav_yaw)]
        mav.SendVelNED(vel_d[0], vel_d[1], vel_d[2], vel_d[3])


    if elapsed_time > 80:
        #退出Offboard控制模式
        print("Send offboard stop")
        mav.endOffboard()
        time.sleep(1)

        #退出Mavlink数据接收模式
        print("Send Mavlink stop")
        mav.stopRun()
        time.sleep(1)
