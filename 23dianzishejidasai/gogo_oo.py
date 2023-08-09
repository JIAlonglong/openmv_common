import sensor, image, time
from pid import PID
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_gainceiling(8)
from pyb import UART
clock = time.clock()
uart= UART(3,115200)
uart.init(115200, bits=8, parity=None, stop=1)
x_pid = PID(p=0.05, i=1, imax=100)
h_pid = PID(p=0.05, i=0.1, imax=50)
kernel_size = 1
kernel = [-1, -1, -1,
          -1, +8, -1,
          -1, -1, -1]
rect_features = []
black_threshold = (0, 40)
if sensor.get_id() == sensor.OV7725:
    sensor.__write_reg(0xAC, 0xDF)
    sensor.__write_reg(0x8F, 0xFF)

def distance_between_points(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    distance = (dx**2 + dy**2)**0.5
    return distance

def calculate_target(corners):
    sorted_corners = sorted(corners, key=lambda x: x[0]+x[1])
    target_points = []
    for i in range(4):
        x1, y1 = sorted_corners[i]
        x2, y2 = sorted_corners[(i+1)%4]
        distance = distance_between_points(x1, y1, x2, y2)
        num_points = int(distance)
        for j in range(num_points+1):
            t = j / num_points
            x = int(x1 + t * (x2 - x1))
            y = int(y1 + t * (y2 - y1))
            target_points.append((x, y))
    return target_points

def move_robot_to_target(target_x, target_y, threshold=5):
    # 获取图像的中心坐标
    image_center_x = img.width() // 2
    image_center_y = img.height() // 2

    # 计算边缘中心点相对于图像中心点的偏移量
    offset_x = target_x - image_center_x
    offset_y = target_y - image_center_y

    # 使用 PID 控制算法计算移动输出
    move_x_output = x_pid.get_pid(offset_x, 1)
    move_h_output = h_pid.get_pid(offset_y, 1)

    # 判断是否成功移动到目标位置
    success = abs(offset_x) < threshold and abs(offset_y) < threshold

    return move_x_output, move_h_output, success

FH = bytearray([0x2C,0x12,0,0,0,0,0,0,0,0,80,60,0x5B])
uart.write(FH)
while True:
    clock.tick()
    img = sensor.snapshot()
    img.morph(kernel_size, kernel)
    #img.find_edges(image.EDGE_CANNY, threshold=(50, 80))
    img = img.binary([black_threshold])
    img.erode(1, threshold=2)
    #line = img.get_regression([(100,100)], robust = True)
    for r in img.find_rects(threshold=110000):
        img.draw_rectangle(r.rect(), color=(255, 0, 0))
        rect_corners = r.corners()
        rect_left_up = rect_corners[0]
        print("Rectangle Corners:", rect_corners)
        for p in r.corners():
            img.draw_circle(p[0], p[1], 5, color=(0, 255, 0))
            #FH = bytearray([0x2C,0x12,int(rect_corners[0][0]),int(rect_corners[0][1]),int(rect_corners[1][0]),int(rect_corners[1][1]),int(rect_corners[2][0]),int(rect_corners[2][1]),int(rect_corners[3][0]),int(rect_corners[3][1]),0x5B])
            #uart.write(FH)
            success_1=False
            success_2=False
            success_3=False
            success_4=False
        ##move to left_high
            #if not success_1 and not success_2 and not success_3 and not success_4:
            move_x_output,move_h_output,success_1=move_robot_to_target(int(rect_corners[0][0]),int(rect_corners[0][1]))
            #if success_1:
            #move to right_high
                #move_x_output,move_h_output,success_2=move_robot_to_target(int(rect_corners[1][0]),int(rect_corners[1][1]))
                #success_1=False
            ##move to right_down
            #elif success_2:
                #move_x_output,move_h_output,success_3=move_robot_to_target(int(rect_corners[2][0]),int(rect_corners[2][1]))
                #success_2=False
            #elif success_3:
                #move_x_output,move_h_output,success_4=move_robot_to_target(int(rect_corners[3][0]),int(rect_corners[3][1]))
                #success_3=False
            #move_x_output=min(move_x_output,-128)
            #move_x_output=max(move_x_output,128)
            #move_h_output=min(move_h_output,-128)
            #move_h_output=max(move_x_output,128)
            FH = bytearray([0x2C,0x12,int(rect_corners[0][0]),int(rect_corners[0][1]),int(rect_corners[1][0]),int(rect_corners[1][1]),int(rect_corners[2][0]),int(rect_corners[2][1]),int(rect_corners[3][0]),int(rect_corners[3][1]),int(move_x_output+128),int(move_h_output+128),0x5B])
            uart.write(FH)
            print('move_x_output', move_x_output+128)
            print('move_h_output', move_h_output+128)
    time.sleep(0.1)
    print(clock.fps())
