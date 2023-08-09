import sensor, image, time, pyb

# 初始化串口
uart = pyb.UART(3, 115200, timeout_char=1000)

# 配置图像传感器
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(10)
sensor.set_auto_whitebal(False)
sensor.set_hmirror (True)
#sensor.set_vflip (True)

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

# 配置颜色追踪器，使用LAB阈值范围
red_threshold = (21, 75, 18, 88, -9, 31) # LAB阈值范围
track_color = (255, 0, 0)
roi = (0, 0, sensor.width(), sensor.height())

#定义一个坐标点集合
points = [[80, 28], [240, 30], [240, 205], [60, 190], [65, 28]] #左上为第一的点，右上第二，右下第三，左下第四，第五又是第一点

flag_fd = 0 #flag_fd变量是第几个点

nx = 0 #当前光点坐标值
ny = 0

while True:
    # 捕获一帧图像
    img = sensor.snapshot()
    nx = 0  #当前光点坐标值每次循环前赋值成0
    ny = 0
    blobsy = img.find_blobs([red_threshold])
    #if blobs and blobs.cx()<265 and blobs.cy() <220 and blobs.cx()>20 and blobs.cy() >20 :
    for blobs in img.find_blobs([red_threshold]):
        if blobs.cx()<265 and blobs.cy() <220 and blobs.cx()>20 and blobs.cy() >20 :
            max_blob = find_max(blobsy)
            x = max_blob.cx()
            y = max_blob.cy()
        #r = c.r()  #这是光点的半径，可以用在发挥题目上，根据红绿斑点的半径来区分红绿点以及坐标
            nx = x
            ny = y

    # 延迟一段时间以减少CPU使用率，可以去掉
    pyb.delay(10)

    # 依次发布点
    dx = nx - points[flag_fd][0]  #计算当前点与目标点的差，flag_fd变量是第几个点
    dy = ny - points[flag_fd][1]
    #dx =  points[flag_fd][0] - nx
    #dy =  points[flag_fd][1] - ny
    if(nx != 0 and ny != 0):   #刚开始赋值成0，如果没识别到光点，就不进入
        if(dx<-10 or dx>10 or dy<-10 or dy>10):  #判断目标点与当前点的差，在目标的10范围外就一直发布偏差，范围内就发布下一个点
            #串口发送dx，dy
            print('红色激光点的中心坐标为({0}, {1}, {2})'.format(int(dx), int(dy),flag_fd))
            print((dx+500)/4,(dy+500)/4)
            output_bytes = bytearray([0xff, 0xfe,int((dx+500)/4), int((dy+500)/4),int(0),int(0),int(0),int(0),int(0),int(0)])  #(dy+500)/4是为了保证输出值在0~255之间，便于32串口读取
            #output_bytes = bytearray([0xff, 0xfe,int(dx), int(dy)])
            uart.write(output_bytes)
        else:
            #点++
            flag_fd = flag_fd + 1
            if (flag_fd >len(points)-1):
                flag_fd = 0

