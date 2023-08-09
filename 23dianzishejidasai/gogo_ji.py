import sensor, image, time
from pid import PID
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_gainceiling(8)
clock = time.clock()
x_pid = PID(p=0.5, i=1, imax=100)
h_pid = PID(p=0.05, i=0.1, imax=50)
uart= UART(3,115200)
uart.init(115200, bits=8, parity=None, stop=1)
# 设置核函数滤波
kernel_size = 1  # kernel width = (size*2)+1, kernel height = (size*2)+1
kernel = [-1, -1, -1,
          -1, +8, -1,
          -1, -1, -1]

# 存储五个矩形的特征
rect_features = []
rect_corners_1= []
# 定义一个列表来存储已经巡过的直线的索引
visited_lines = []

# 黑色方框颜色范围
black_threshold = (0, 40)  # 颜色范围
enable_lens_corr = False # turn on for straighter lines...打开以获得更直的线条…
# 在OV7725 sensor上, 边缘检测可以通过设置sharpness/edge寄存器来增强。
# 注意:这将在以后作为一个函数实现
if sensor.get_id() == sensor.OV7725:
    sensor.__write_reg(0xAC, 0xDF)
    sensor.__write_reg(0x8F, 0xFF)

def get_target_position(line_segment):
    line_x1 = line_segment.x1()
    line_y1 = line_segment.y1()
    line_x2 = line_segment.x2()
    line_y2 = line_segment.y2()

    direction_vector = (line_x2 - line_x1, line_y2 - line_y1)
    length = line_segment.length()
    unit_direction_vector = (direction_vector[0] / length, direction_vector[1] / length)

    target_x = line_x1 + length * unit_direction_vector[0]
    target_y = line_y1 + length * unit_direction_vector[1]

    return (target_x, target_y)

def distance_between_points(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    distance = (dx**2 + dy**2)**0.5
    return distance

while True:
    clock.tick()
    img = sensor.snapshot()

    # 使用核函数滤波
    img.morph(kernel_size, kernel)

    # 对图像进行颜色过滤，只保留黑色方框内的像素
    img = img.binary([black_threshold])
    # 使用形态学腐蚀操作，减少其他颜色的影响
    img.erode(1, threshold=2)
    for c in circles = img.find_circles(threshold=2000, x_margin=10, y_margin=10, r_margin=10, r_min=2, r_max=100, r_step=2, r_multiplier=1.1):
        img.draw_circle((c.x(), c.y(),c.r())
    # 在图像上找到矩形轮廓，并打印矩形的四个角点坐标
    for r in img.find_rects(threshold=100000):
        img.draw_rectangle(r.rect(), color=(255, 0, 0))
        rect_corners = r.corners()
        print("Rectangle Corners:", rect_corners)
        for p in r.corners():
            img.draw_circle(p[0], p[1], 5, color=(0, 255, 0))

        # 计算矩形的特征
        rect_feature = {
            'left_up': rect_corners[0],
            'right_up': rect_corners[1],
            'right_down': rect_corners[2],
            'left_down': rect_corners[3]
        }
        distance_error=distance_between_points(c.x(),c.y(),r.x(),r.y())

        # 判断矩形是否大致相同
        is_similar = False
        for feature in rect_features:
            # 比较特征并设置相似度阈值
            similarity_threshold = 5  # 根据实际情况进行调整
            if (abs(rect_feature['left_up'][0] - feature['left_up'][0]) < similarity_threshold and
                    abs(rect_feature['left_up'][1] - feature['left_up'][1]) < similarity_threshold and
                    abs(rect_feature['right_up'][0] - feature['right_up'][0]) < similarity_threshold and
                    abs(rect_feature['right_up'][1] - feature['right_up'][1]) < similarity_threshold and
                    abs(rect_feature['right_down'][0] - feature['right_down'][0]) < similarity_threshold and
                    abs(rect_feature['right_down'][1] - feature['right_down'][1]) < similarity_threshold and
                    abs(rect_feature['left_down'][0] - feature['left_down'][0]) < similarity_threshold and
                    abs(rect_feature['left_down'][1] - feature['left_down'][1]) < similarity_threshold):
                is_similar = True
                break

        if not is_similar:
            # 如果不相似，则将新矩形的特征添加到列表中
            rect_features.append(rect_feature)

        # 如果已接收到五个相似的矩形，则打印信息并清空特征列表
        if len(rect_features) == 5:
            print("Received 5 similar rectangles!")
            avg_feature = {
                'left_up': (sum(r['left_up'][0] for r in rect_features) // 5, sum(r['left_up'][1] for r in rect_features) // 5),
                'right_up': (sum(r['right_up'][0] for r in rect_features) // 5, sum(r['right_up'][1] for r in rect_features) // 5),
                'right_down': (sum(r['right_down'][0] for r in rect_features) // 5, sum(r['right_down'][1] for r in rect_features) // 5),
                'left_down': (sum(r['left_down'][0] for r in rect_features) // 5, sum(r['left_down'][1] for r in rect_features) // 5),
            }
            print("Average Feature:", avg_feature)
            rect_features = []
            rect_corners_1= [avg_feature['left_up'][0]+10,avg_feature['left_up'][1]+10,abs(avg_feature['left_up'][0]-avg_feature['right_up'][0]+10),abs(avg_feature['left_up'][1]-avg_feature['left_down'][1]+10)]
         # 在检测到的矩形区域内找到直线
        #line_segments = img.find_lines(threshold=200, theta_margin=25, rho_margin=25, roi=[0,0,160,240])

        #if line_segments:
           #print("lines",len(line_segments))
           #for line in line_segments:
                    #img.draw_line(line.line(), color = 127)
           #for idx, line in enumerate(line_segments):
              #if idx not in visited_lines:
                        ## 计算目标位置
                  #target_x, target_y = get_target_position(line)
                        ## 获取图像的中心坐标
                  #image_center_x = img.width() // 2
                  #image_center_y = img.height() // 2

                        ## 计算边缘中心点相对于图像中心点的偏移量
                  #offset_x = target_x - image_center_x
                  #offset_y = target_y - image_center_y

                  #move_x_output=x_pid.get_pid(offset_x,1)
                  #move_h_output=h_pid.get_pid(offset_y,1)
                  #print('move_x_output',move_x_output)
                  #print('move_h_output',move_h_output)
                  FH = bytearray([0x2C,0x12,int(move_x_output),1,1,int(move_h_output),0x5B])
                  uart.write(FH)
                  if abs(offset_x) < 1 and abs(offset_y) < 1:
                      visited_lines.append(idx)

    time.sleep(0.1)
    print(clock.fps())
