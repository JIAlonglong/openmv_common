# Untitled - By: Dell - 周六 8月 5 2023

from pyb import Pin


p1_in = Pin('P7', Pin.IN, Pin.PULL_DOWN)#设置p_in为输入引脚，并开启上拉电阻
task_1 = p1_in.value() # get value, 0 or 1#读入p_in引脚的值
p2_in = Pin('P8', Pin.IN, Pin.PULL_DOWN)#设置p_in为输入引脚，并开启上拉电阻
task_2 = p2_in.value() # get value, 0 or 1#读入p_in引脚的值
p3_in = Pin('P9', Pin.IN, Pin.PULL_DOWN)#设置p_in为输入引脚，并开启上拉电阻
task_3 = p3_in.value() # get value, 0 or 1#读入p_in引脚的值

if task_1:
