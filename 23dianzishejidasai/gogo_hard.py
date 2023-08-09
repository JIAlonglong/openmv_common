import sensor, image, time
from pid import PID

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_gainceiling(8)
clock = time.clock()
x_pid = PID(p=0.5, i=1, imax=100)
h_pid = PID(p=0.05, i=0.1, imax=50)
# 设置核函数滤波
kernel_size = 1  # kernel width = (size*2)+1, kernel height = (size*2)+1
kernel = [-1, -1, -1,
          -1, +8, -1,
          -1, -1, -1]

# 存储五个矩形的特征
rect_features = []
visited_flags = []

# 黑色方框颜色范围
black_threshold = (0, 40)  # 颜色范围

# 在OV7725 sensor上, 边缘检测可以通过设置sharpness/edge寄存器来增强。
# 注意:这将在以后作为一个函数实现
if sensor.get_id() == sensor.OV7725:
    sensor.__write_reg(0xAC, 0xDF)
    sensor.__write_reg(0x8F, 0xFF)

# 对每个矩形的边缘点进行排序
def distance_to_top_left_corner(p):
    return (p[0] - rect_features[0]['left_up'][0])**2 + (p[1] - rect_features[0]['left_up'][1])**2

while True:
    clock.tick()
    img = sensor.snapshot()

    # 使用核函数滤波
    img.morph(kernel_size, kernel)

    # 对图像进行颜色过滤，只保留黑色方框内的像素
    img = img.binary([black_threshold])

    # 使用形态学腐蚀操作，减少其他颜色的影响
    img.erode(1, threshold=2)

    # 在边缘图像中找到边缘的像素坐标
    edge_points = img.find_rects(threshold=100000)

    # 对每个矩形的边缘点进行排序
    def distance_to_top_left_corner(p):
        return (p[0] - rect_features[0]['left_up'][0])**2 + (p[1] - rect_features[0]['left_up'][1])**2

    if edge_points:
        edge_points.sort(key=distance_to_top_left_corner)

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

            # 判断矩形是否大致相同
            is_similar = False
            for idx, feature in enumerate(rect_features):
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
                # 如果不相似，则将新矩形的特征添加到列表中，并设置该矩形的巡视标志为False
                rect_features.append(rect_feature)
                visited_flags.append(False)

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

                # 巡视标志全部设为False，准备开始巡视
                visited_flags = [False] * 5
                rect_features = []

        # 如果找到了边缘，开始巡视
        if edge_points:
         # 找到一个未巡视的点，进行巡视
            found_unvisited_point = False
            for i, p in enumerate(edge_points):
                if not visited_flags[i]:
                    edge_center_x, edge_center_y = p.cx(), p.cy()
                    found_unvisited_point = True
                    break

            if found_unvisited_point:
            # 获取图像的中心坐标
                image_center_x = img.width() // 2
                image_center_y = img.height() // 2

            # 计算边缘中心点相对于图像中心点的偏移量
                offset_x = edge_center_x - image_center_x
                offset_y = edge_center_y - image_center_y

                # 标记该点已经巡视过
                visited_flags[i] = True


    time.sleep(0.1)
    print(clock.fps())
