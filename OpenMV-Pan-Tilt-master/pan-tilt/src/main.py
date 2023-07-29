import sensor, image, time
import ustruct  # 导入 struct 模块

from pid import PID
from pyb import Servo
from pyb import UART

pan_servo=Servo(1)
tilt_servo=Servo(2)
uart= UART(3,115200)
uart.init(115200, bits=8, parity=None, stop=1)
pan_transform=0
tilt_transform=0
MAX_TRANSFORM = 1000  # 设置转动值的上限（根据需要进行调整）

pan_servo.calibration(500,2500,500)
tilt_servo.calibration(500,2500,500)

red_threshold  = (13, 49, 18, 61, 6, 47)

pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
#tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

def sending_data(cx,cy,cw,ch):
    global uart;
    #frame=[0x2C,18,cx%0xff,int(cx/0xff),cy%0xff,int(cy/0xff),0x5B];
    #data = bytearray(frame)
    data = ustruct.pack("<bbhhhhb",      #格式为俩个字符俩个短整型(2字节)
                   0x2C,                      #帧头1
                   0x12,                      #帧头2
                   int(cx), # up sample by 4   #数据1
                   int(cy), # up sample by 4    #数据2
                   int(cw), # up sample by 4    #数据1
                   int(ch), # up sample by 4    #数据2
                   0x5B)
    uart.write(data);   #必须要传入一个字节数组

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    blobs = img.find_blobs([red_threshold])
    if blobs:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2

        print("pan_error: ", pan_error)

        img.draw_rectangle(max_blob.rect()) # rect
        img.draw_cross(max_blob.cx(), max_blob.cy()) # cx, cy

        pan_output=pan_pid.get_pid(pan_error,1)/2
        tilt_output=tilt_pid.get_pid(tilt_error,1)
        print("pan_output",pan_output)
        print("tilt_output",tilt_output)
        pan_transform=pan_transform+pan_output
        tilt_transform=tilt_transform-tilt_output
        #sending_data(1,pan_transform,tilt_transform,3)
            # 限制 pan_output 和 tilt_output 的最大值
        pan_output = min(pan_output, MAX_TRANSFORM)
        pan_output = max(pan_output, -MAX_TRANSFORM)
        tilt_output = min(tilt_output, MAX_TRANSFORM)
        tilt_output = max(tilt_output, -MAX_TRANSFORM)
        pan_output=int(pan_output/3.1415926*180)
        tilt_output=int(tilt_output/3.1415926*180)
        #pan_output_bytes = struct.pack('f', pan_transform)
        #tilt_output_bytes = struct.pack('f', tilt_transform)
        FH = bytearray([0x2C,0x12,1,pan_output,tilt_output,0,0x5B])
        uart.write(FH)
        print("pan_output1",pan_output)
        print("tilt_output1",tilt_output)
        #FH = bytearray([0x2C, 0x12, 0]) + pan_output_bytes + tilt_output_bytes + bytearray([0, 0x5B])
        #uart.write(FH)
        #pan_servo.angle(pan_servo.angle()+pan_output)
        #tilt_servo.angle(tilt_servo.angle()-tilt_output)
