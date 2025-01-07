import sensor, image, time, sys, pyb
import utime, math
from pyb import UART
from pid import PID
from pyb import Pin, Timer

# 初始化舵机引脚和定时器
pwma = Pin('P7')
pwmb = Pin('P8')
tim = Timer(4, freq=50)
pan_servo = tim.channel(1, Timer.PWM, pin=pwma)
tilt_servo = tim.channel(2, Timer.PWM, pin=pwmb)

# 初始化PID控制器
pan_pid = PID(p=0.31, i=0.045, d=0.015, imax=90)
tilt_pid = PID(p=0.31, i=0.045, d=0.015, imax=90)

# 颜色阈值
red_threshold = (0, 100, 5, 60, 5, 50)
white_threshold = (100, 85, -5, 5, -5, 5)

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, 10000)
sensor.skip_frames(time=2000)
sensor.set_vflip(True)
sensor.set_hmirror(True)
clock = time.clock()
sensor.set_windowing([0, 0, 640, 400])

# 初始化引脚
Pin0 = Pin('P0', Pin.IN, Pin.PULL_UP)
Pin1 = Pin('P1', Pin.IN, Pin.PULL_UP)
Pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)
Pin3 = Pin('P3', Pin.IN, Pin.PULL_UP)

# 初始化UART
uart = pyb.UART(3, 115200, timeout_char=1000)
uart.init(115200, bits=8, parity=None, stop=1)

# 重置舵机位置
def reset(x, y):
	pan_servo.pulse_width(x)
	tilt_servo.pulse_width(y)

# 找到最大的颜色块，并且防止反光（反光会导致颜色块数量增多，按照题目要求，应该选上面那个）
def find_max(blobs):
	max_size = 0
	high_blob1 = 480
	for blob in blobs:
		if blob[2] * blob[3] > max_size:
			if(blob.cy() < high_blob1):
				high_blob1 = blob.cy()
				max_blob = blob
				max_size = blob[2] * blob[3]
	return max_blob

# 控制舵机转动
def turn(move):
	if(move == 0):
		tilt_servo.pulse_width(tilt_servo.pulse_width())
		pan_servo.pulse_width(pan_servo.pulse_width())
	elif(move == 1):
		pan_servo.pulse_width(pan_servo.pulse_width() + 20)
	elif(move == 2):
		tilt_servo.pulse_width(tilt_servo.pulse_width() - 5)
	elif(move == 3):
		pan_servo.pulse_width(pan_servo.pulse_width() - 20)
	elif(move == 4):
		tilt_servo.pulse_width(tilt_servo.pulse_width() + 5)

# 初始化全局变量
red_x = 0
red_y = 0
nowarn = 0

# 获取红色物体的坐标
def red_coord():
	global img, red_x, red_y, nowarn
	red_blobs = img.find_blobs([red_threshold])
	if red_blobs:
		red_blob = find_max(red_blobs)
		red_x = red_blob[5]
		red_y = red_blob[6]
		img.draw_rectangle(red_blob.rect())
		img.draw_cross(red_blob.cx(), red_blob.cy())
		key = 2
		nowarn = 0
	else:
		nowarn = 1

# 初始化偏差和输出
pan_bias = 0
tilt_bias = 0
pan_output = 0
tilt_output = 0

# 计算偏差并调整舵机
def bias():
	global pan_bias, tilt_bias, pan_output, tilt_output, red_x, red_y
	replace = 0
	red_coord()  # 获取红色物体的坐标
	pan_bias = green_x - red_x  # 计算水平偏差
	tilt_bias = red_y - green_y  # 计算垂直偏差
	if(abs(pan_bias) <= 35) and (abs(tilt_bias) <= 35) and (nowarn == 0):
		# 如果偏差在允许范围内且没有警告，发送指令
		FH = bytearray([0xFC, 3, 0xFE])
		uart.write(FH)
	if(nowarn):
		# 如果没有检测到红色物体，发送停止指令
		FH = bytearray([0xFC, 0, 0xFE])
		uart.write(FH)
		print(1)
	pan_output = int(pan_pid.get_pid(pan_bias, 1))  # 计算水平输出
	tilt_output = int(tilt_pid.get_pid(tilt_bias, 1))  # 计算垂直输出
	pan_servo.pulse_width(pan_servo.pulse_width() + pan_output)  # 调整水平舵机
	tilt_servo.pulse_width(tilt_servo.pulse_width() - tilt_output)  # 调整垂直舵机
	red_x = green_x  # 更新红色物体的坐标
	red_y = green_y  # 更新红色物体的坐标
	pan_output = 0  # 重置水平输出
	tilt_output = 0  # 重置垂直输出
	pan_bias = 0  # 重置水平偏差
	tilt_bias = 0  # 重置垂直偏差

# 左侧扫描
def left_working_scan():
	global Run_State, left_location
	if (Pin0.value() == 0):
		utime.sleep_ms(20)
		if(Pin0.value() == 0):
			while(Pin0.value() == 0):
				Run_State = 1
				left_location = 1
				FH = bytearray([0xFC, 1, 0xFE])
				uart.write(FH)

# 右侧扫描
def right_working_scan():
	global Run_State, right_location
	if (Pin1.value() == 0):
		utime.sleep_ms(20)
		if(Pin1.value() == 0):
			while(Pin1.value() == 0):
				right_location = 1
				Run_State = 1
				FH = bytearray([0xFC, 1, 0xFE])
				uart.write(FH)

# 暂停扫描
def pause_scan():
	global Run_State, pause
	if (Pin2.value() == 0):
		utime.sleep_ms(20)
		if(Pin2.value() == 0):
			while(Pin2.value() == 0):
				pause = 1
				FH = bytearray([0xFC, 4, 0xFE])
				uart.write(FH)

# 初始化状态变量
taskflag = 0
Run_State = 0
left_location = 0
right_location = 0
pause = 0

# 重置舵机位置
reset(2800, 3000)
utime.sleep_ms(1000)

# 初始化绿色物体的坐标
green_x = 287
green_y = 287

# 主循环
while(True):
	clock.tick()
	img = sensor.snapshot()
	if(Run_State == 0):
		right_working_scan()  # 右侧扫描
		left_working_scan()   # 左侧扫描
	elif(Run_State == 1):
		pause_scan()  # 暂停扫描
		if(pause == 0):
			if(right_location == 1):
				if(taskflag == 0):
					turn(1)  # 向右转动舵机
					red_coord()  # 获取红色物体的坐标
					if(red_x and red_x != green_x):
						taskflag = 1
						FH = bytearray([0xFC, 2, 0xFE])
						uart.write(FH)
				elif(taskflag == 1):
					bias()  # 计算偏差并调整舵机
			elif(left_location == 1):
				if(taskflag == 0):
					turn(3)  # 向左转动舵机
					red_coord()  # 获取红色物体的坐标
					if(red_x and red_x != green_x):
						taskflag = 1
						FH = bytearray([0xFC, 2, 0xFE])
						uart.write(FH)
				elif(taskflag == 1):
					bias()  # 计算偏差并调整舵机
		elif(pause == 1):
			turn(0)  # 停止转动舵机
			FH = bytearray([0xFC, 0, 0xFE])
			uart.write(FH)
			sys.exit()  # 退出程序