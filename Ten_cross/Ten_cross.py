import pyb, sensor , math,time
from image import SEARCH_EX, SEARCH_DS
import struct
from pyb import UART
from pid import PID

rho_pid = PID(p=0.37, i=0)
theta_pid = PID(p=0.001, i=0)
sensor.set_contrast(1)
sensor.set_gainceiling(16)
clock=time.clock()
THRESHOLD = (21, 0, -77, 5, -110, 127)
uart = UART(3,115200,bits=8, parity=None, stop=1, timeout_char = 1000)

roi=        [(0, 40, 20, 40),        #  左  x y w h
            (70, 40, 20, 40),       #  中
            (70,90,20,30),          #下
            (70,0,20,30),         #上
            (140,40,20,40)]         # 右

sensor.reset() # 初始化摄像头
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)#160x120
sensor.skip_frames(time=2000) # 跳过10帧，使新设置生效
sensor.set_auto_whitebal(True) # turn this off.
sensor.set_auto_gain(False) # must be turned off for color tracking
global cross
high_threshold=(0, 42)
output1=0

def sending_data(cx,cy,cw,ch):
    global uart;
    #frame=[0x2C,18,cx%0xff,int(cx/0xff),cy%0xff,int(cy/0xff),0x5B];
    #data = bytearray(frame)
    data = struct.pack("<bbhhhhb",      #格式为俩个字符俩个短整型(2字节)
                   0x2C,                      #帧头1
                   0x12,                      #帧头2
                   int(cx), # up sample by 4   #数据1
                   int(cy), # up sample by 4    #数据2
                   int(cw), # up sample by 4    #数据1
                   int(ch), # up sample by 4    #数据2
                   0x5B)
    uart.write(data);   #必须要传入一个字节数组

while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(strength = 1.7 , zoom = 1.0).binary([THRESHOLD]) # 二值化图像
    blob1=None #左
    blob2=None #中
    blob3=None #下
    blob4=None #上
    blbo5=None #右
    flag = [0,0,0,0,0]
    blob1=img.find_blobs([high_threshold],roi=roi[0],pixels_threshold=200,area_threshold=100,invert=1)
    blob2=img.find_blobs([high_threshold],roi=roi[1],pixels_threshold=200,area_threshold=100,invert=1)
    blob3=img.find_blobs([high_threshold],roi=roi[2],pixels_threshold=200,area_threshold=100,invert=1)
    blob4=img.find_blobs([high_threshold],roi=roi[3],pixels_threshold=200,area_threshold=100,invert=1)
    blob5=img.find_blobs([high_threshold],roi=roi[4],pixels_threshold=200,area_threshold=100,invert=1)
    output1=0
    if blob1:
        flag[0] = 1  #左边检测到红线
        #output1=1100
        print("left")
    if blob2:
        flag[1] = 1  #中间检测到红线
        #output1=1010
        print("middle")
    if blob3:
        flag[2] = 1  #右边检测到红线
        #output1=1110
        print("down")
    if blob4:
        flag[3] = 1  #中间检测到红线
        #output1=1011
        print("up")
    if blob5:
        flag[4] = 1  #右边检测到红线
        #output1=1001
        print("right")
    if blob1 and blob2 and blob3 and blob4 and blob5:
        output1=10
    elif blob1 and blob2 and blob3 and blob5:
        output1=1
    sending_data(0,0,0,output1)
    print(output1)
    line = img.get_regression([(100,100)], robust = True)
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color = 127)
        print(rho_err,line.magnitude(),rho_err)
        if line.magnitude()>8:
            rho_output = rho_pid.get_pid(rho_err,1)
            theta_output = theta_pid.get_pid(theta_err,1)
            output = rho_output + theta_output
            #if(output<0):
                #output = abs(output)
            OUTPUT = round(output)
            print(OUTPUT)
            sending_data(0,0,OUTPUT,output1)
    for rec in roi:
        img.draw_rectangle(rec, color=(255,0,0))#绘制出roi区域
