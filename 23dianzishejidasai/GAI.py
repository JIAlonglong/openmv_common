
import sensor, image, time, pyb

# 初始化串口
uart = pyb.UART(3, 115200, timeout_char=1000)

sensor.reset()
sensor.set_pixformat(sensor.RGB565) # 灰度更快(160x120 max on OpenMV-M7)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
# 配置颜色追踪器，使用LAB阈值范围
red_threshold = (90, 0, 92, 5, -37, 127) # LAB阈值范围
track_color = (255, 0, 0)
roi = (0, 0, sensor.width(), sensor.height())

points = [[65, 28], [240, 30], [240, 195], [50, 200]]

flag_fd = 3

nx = 0
ny = 0
get_times = 0

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

while(True):
    img = sensor.snapshot()

    # 下面的`threshold`应设置为足够高的值，以滤除在图像中检测到的具有
    # 低边缘幅度的噪声矩形。最适用与背景形成鲜明对比的矩形。
    if get_times < 10:
        for r in img.find_rects(threshold = 20000):
            img.draw_rectangle(r.rect(), color = (255, 0, 0))  #画出矩形框
            get_times += 1
            i=0
            for p in r.corners():
                #nx_p=p.x()
                #ny_p=p.y()
                #img.draw_circle(p[0], p[1], 5, color = (0, 255, 0))  #圈出矩形框顶点
                print(p)
                points[i][0] = p[0]
                points[i][1] = p[1]
                i = i + 1
        print("iiiiiii=")
        print(points)  #打印出顶点坐标
    #串口发送给stm32，四个顶点的坐标，左上为第一次发的点，右上第二，右下第三，左下第四，把顶点 交给stm32，stm32控制舵机规划轨迹到对应坐标
        output_bytes = bytearray([0xff, 0xfe,int(points[3][0]/2), int(points[3][1]/2),int(points[2][0]/2), int(points[2][1]/2),int(points[1][0]/2), int(points[1][1]/2),int(points[0][0]/2), int(points[0][1]/2),0,0])
        uart.write(output_bytes)


    #本来要尝试视觉识别红点，可惜激光笔斑点太小，进入黑色更是没有了，openmv摄像头性能着实有限所以就改用顶点方案了
    img = sensor.snapshot()
    nx = 0
    ny = 0

    if get_times == 10:
        sensor.set_auto_whitebal(False)  # 关闭自动白平衡
        sensor.set_auto_gain(False, gain_db=0)  # 关闭自动增益并设置增益为0
        sensor.set_auto_exposure(False, exposure_us=1000)  # 关闭自动曝光并设置曝光时间为100us
        blobs = img.find_blobs([red_threshold])
        if blobs:
            max_blob = find_max(blobs)
                # 绘制红色激光点的圆形区域和中心点
            img.draw_circle(max_blob.cx(), max_blob.cy(), 5, (0,255,0))
            img.draw_cross(max_blob.cx(), max_blob.cy(), (0,255,0))

                # 发送红色激光点的中心坐标到串口
            x = max_blob.cx()
            y = max_blob.cy()
            nx = x
            ny = y
                #output_bytes = bytearray([0xff, 0xfe,int(x), int(y)])
                #uart.write(output_bytes)

                # 打印红色激光点的中心坐标
                #print('红色激光点的中心坐标为({0}, {1}, {2})'.format(x, y,flag_fd))
        # 延迟一段时间以减少CPU使用率
        pyb.delay(10)
        # 依次发布点
        dx = nx - points[flag_fd][0]
        dy = ny - points[flag_fd][1]

    #dx =  points[flag_fd][0] - nx
    #dy =  points[flag_fd][1] - ny
    #if(dx>10 or dy>10):
        #if(dx<-10 or dy<-10):
        if(nx != 0 and ny != 0):
            if(dx<-2 or dx>2 or dy<-2 or dy>2):
                #串口发送dx，dy
                print('红色激光点的中心坐标为({0}, {1}, {2})'.format(int(dx), int(dy),flag_fd))
                output_bytes = bytearray([0xff, 0xfe,int(points[3][0]/2), int(points[3][1]/2),int(points[2][0]/2), int(points[2][1]/2),int(points[1][0]/2), int(points[1][1]/2),int(points[0][0]/2), int(points[0][1]/2),int((dx+500)/4),int((dy+500)/4)])
                uart.write(output_bytes)
            else:
                #点++
                flag_fd = flag_fd - 1
                if (flag_fd == -1):
                    flag_fd = 3
        for p in points:
            img.draw_cross(p[0],p[1],(0,0,255))

