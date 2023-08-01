import sensor, image, time
import ustruct
from pid import PID
from pyb import Servo
from pyb import UART
pan_servo=Servo(1)
tilt_servo=Servo(2)
uart= UART(3,115200)
uart.init(115200, bits=8, parity=None, stop=1)
pan_transform=0
tilt_transform=0
MAX_TRANSFORM = 1000
pan_servo.calibration(500,2500,500)
tilt_servo.calibration(500,2500,500)
red_threshold  = (13, 49, 18, 61, 6, 47)
pan_pid = PID(p=0.07, i=0, imax=90)
tilt_pid = PID(p=0.05, i=0, imax=90)
x_pid = PID(p=0.5, i=1, imax=100)
h_pid = PID(p=0.05, i=0.1, imax=50)
size_threshold = 2000
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(10)
sensor.set_auto_whitebal(False)
sensor.set_hmirror (True)
sensor.set_vflip (True)
clock = time.clock()
def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob
def sending_data(cx,cy,cw,ch):
    global uart;
    data = ustruct.pack("<bbhhhhb",
                   0x2C,
                   0x12,
                   int(cx),
                   int(cy),
                   int(cw),
                   int(ch),
                   0x5B)
    uart.write(data);
while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([red_threshold])
    if blobs:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2
        print("pan_error: ", pan_error)
        img.draw_rectangle(max_blob.rect())
        img.draw_cross(max_blob.cx(), max_blob.cy())
        pan_output=pan_pid.get_pid(pan_error,1)/2
        tilt_output=tilt_pid.get_pid(tilt_error,1)
        #print("pan_output",pan_output)
        #print("tilt_output",tilt_output)
        pan_transform=pan_transform+pan_output
        tilt_transform=tilt_transform-tilt_output
        pan_output = min(pan_output, MAX_TRANSFORM)
        pan_output = max(pan_output, -MAX_TRANSFORM)
        tilt_output = min(tilt_output, MAX_TRANSFORM)
        tilt_output = max(tilt_output, -MAX_TRANSFORM)
        pan_output=int(pan_output/3.1415926*180)
        tilt_output=int(tilt_output/3.1415926*180)
        sending_data(0,pan_output,tilt_output,0)

        x_error = max_blob[5]-img.width()/2
        h_error = max_blob[2]*max_blob[3]-size_threshold
        print("x error: ", x_error)
        '''
        for b in blobs:
            # Draw a rect around the blob.
            img.draw_rectangle(b[0:4]) # rect
            img.draw_cross(b[5], b[6]) # cx, cy
        '''
        x_output=x_pid.get_pid(x_error,1)
        h_output=h_pid.get_pid(h_error,1)
        print("h_output",h_output)
        sending_data(-h_output-x_output,pan_output,tilt_output,-h_output+x_output)
        print("pan_output1",pan_output)
        print("tilt_output1",tilt_output)
