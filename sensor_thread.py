#!/usr/bin/python3
# coding = utf-8

import time

from gps import *
from laser import Laser
from serial_port import SerialPortCommunication
from globalvar import *


def laserCheck(rec_buf):
	sum_rec = sum(rec_buf[:-1])  # 求和
	neg = hex(sum_rec ^ 0xffff)  # 取反
	data = neg[-2:]  # 最后一个字节
	check = int(data, 16) + 1
	# check = hex(check)
	return check


def gyroDataAnalysis(gyro_rec_buf, addr_sensor):
	RollH = gyro_rec_buf[3]
	RollL = gyro_rec_buf[4]
	PitchH = gyro_rec_buf[5]
	PitchL = gyro_rec_buf[6]
	if gyro_rec_buf[0] == addr_sensor and gyro_rec_buf[1] == 0x03:
		roll = int(((RollH << 8) | RollL)) / 32768 * 180
		pitch = int(((PitchH << 8) | PitchL)) / 32768 * 180
		if roll > 180:
			roll = (roll - 360)
		if pitch > 180:
			pitch = (pitch - 360)
		roll = round(roll, 2)
		pitch = round(pitch, 2)

		if roll is not None and pitch is not None:
			return roll, pitch
		else:
			return


def mid_filter(fil_list):
	""" 中值滤波 """
	rank_list = sorted(fil_list)
	return rank_list[int(len(rank_list) / 2)]


def gpsThreadFunc():
	log.debug("gpsThreadFunc 线程启动 ...")
	if SWITCH_DEVICE:
		GPS_COM = "com1"
	else:
		GPS_COM = "com11"
	GPS_REC_BUF_LEN = 138
	gps_msg_switch = LatLonAlt()
	gps_com = SerialPortCommunication(GPS_COM, 115200, 0.05)

	x_filter_list = []  # 初始化滤波列表
	y_filter_list = []
	h_filter_list = []
	yaw_filter_list = []
	filter_len = 7

	""" 等待数据准备 """
	while True:
		gps_data = GPSINSData()
		gps_rec_buffer = []
		time.sleep(0.001)

		gps_com.rec_data(gps_rec_buffer, GPS_REC_BUF_LEN)  # int
		if gps_data.gps_msg_analysis(gps_rec_buffer):
			gps_msg_switch.latitude, gps_msg_switch.longitude, gps_msg_switch.altitude, \
				gps_msg_switch.yaw, gps_msg_switch.yaw_state = gps_data.gps_typeswitch()

			y, x = LatLon2XY(gps_msg_switch.latitude, gps_msg_switch.longitude)
			h = gps_msg_switch.altitude

			x = round(x, 3)
			y = round(y, 3)
			h = round(h, 3)
			yaw = round(gps_msg_switch.yaw, 3)

			x_filter_list.append(x)
			y_filter_list.append(y)
			h_filter_list.append(h)
			yaw_filter_list.append(yaw)

			# 中值滤波
			x_mid = mid_filter(x_filter_list)
			y_mid = mid_filter(y_filter_list)
			h_mid = mid_filter(h_filter_list)
			yaw_mid = mid_filter(yaw_filter_list)

			if len(x_filter_list) == filter_len:
				gps_x = x_mid
				gps_y = y_mid
				gps_h = h_mid
				gps_yaw = yaw_mid
				gps_data_valid_flg = True

				# 设置全局变量
				my_lock.pSor_gps_main_lock.acquire()
				gl.set_value("gps_data_valid_flg", gps_data_valid_flg)
				gl.set_value("gps_x", gps_x)
				gl.set_value("gps_y", gps_y)
				gl.set_value("gps_h", gps_h)
				gl.set_value("gps_yaw", gps_yaw)
				my_lock.pSor_gps_main_lock.release()
				break
			else:
				print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx_filter_list:%s\n" % x_filter_list)
				print("yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy_filter_list:%s\n" % y_filter_list)
				print("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh_filter_list:%s\n" % h_filter_list)
		else:
			print("！！！！！！gps准备过程解析错误！！！！！！\n")

		time.sleep(0.1)

	""" 正常流程 """
	while True:
		gps_data = GPSINSData()
		gps_rec_buffer = []

		if gps_com.com.is_open:
			gps_is_open_led = True  # gps串口是否打开
			gps_com.rec_data(gps_rec_buffer, GPS_REC_BUF_LEN)  # int

			if gps_data.gps_msg_analysis(gps_rec_buffer):
				gps_msg_switch.latitude, gps_msg_switch.longitude, gps_msg_switch.altitude, \
					gps_msg_switch.yaw, gps_msg_switch.yaw_state = gps_data.gps_typeswitch()

				"""经纬度转高斯坐标"""
				y, x = LatLon2XY(gps_msg_switch.latitude, gps_msg_switch.longitude)
				h = gps_msg_switch.altitude

				x = round(x, 3)
				y = round(y, 3)
				h = round(h, 3)
				yaw = round(gps_msg_switch.yaw, 3)

				x_filter_list.append(x)
				y_filter_list.append(y)
				h_filter_list.append(h)
				yaw_filter_list.append(yaw)

				x_filter_list.pop(0)
				y_filter_list.pop(0)
				h_filter_list.pop(0)
				yaw_filter_list.pop(0)

				# 均值滤波
				x_mid = mid_filter(x_filter_list)
				y_mid = mid_filter(y_filter_list)
				h_mid = mid_filter(h_filter_list)
				yaw_mid = mid_filter(yaw_filter_list)

				# 设置全局变量
				my_lock.pSor_gps_main_lock.acquire()
				gl.set_value("gps_x", x_mid)
				gl.set_value("gps_y", y_mid)
				gl.set_value("gps_h", h_mid)
				gl.set_value("gps_yaw", yaw_mid)
				my_lock.pSor_gps_main_lock.release()

				# print("平面坐标:  %s\t %s\t %s\t 偏航角度 %s " % (x, y, h, yaw))
		else:
			gps_is_open_led = False

		my_lock.gpsLedLock.acquire()
		gl.set_value("gps_is_open_led", gps_is_open_led)
		my_lock.gpsLedLock.release()

		time.sleep(0.1)


def laserThreadFunc():
	log.debug("laserThreadFunc 线程启动 ...")
	if SWITCH_DEVICE:
		LASER3_COM = "com3"
	else:
		LASER3_COM = "com20"
	LASER_REC_BUF_LEN = 11
	laser3 = Laser(LASER3_COM)
	time.sleep(0.01)
	laser_min_distance = 0.1  # 激光安装最小距离
	laserLed = False

	""" 等待数据准备 """
	while True:
		laser_rec_buf = laser3.laser_read_data(LASER_REC_BUF_LEN)
		if laser_rec_buf is not None:
			# 校验
			check_out = laserCheck(laser_rec_buf)
			last_val = int.from_bytes(laser_rec_buf[-1:], byteorder='little', signed=False)
			if check_out == last_val:
				laser3_dist = laser3.get_distance(laser_rec_buf)
				if laser3_dist > laser_min_distance:
					laser_data_valid_flg = True
					# 设置全局变量
					my_lock.pSor_laser_main_lock.acquire()
					gl.set_value("dou_laser_len", laser3_dist)
					gl.set_value("laser_data_valid_flg", laser_data_valid_flg)
					my_lock.pSor_laser_main_lock.release()
					break
				else:
					print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLdou_laser_len:%s\n" % laser3_dist)
			else:
				print("!!!!!!!激光校验失败!!!!!!\n")
		time.sleep(0.05)

	""" 正常流程 """
	while True:
		if laser3.com_laser.com.is_open:
			laser_rec_buf = laser3.laser_read_data(LASER_REC_BUF_LEN)
			if laser_rec_buf is not None:
				# 校验
				check_out = laserCheck(laser_rec_buf)
				last_val = int.from_bytes(laser_rec_buf[-1:], byteorder='little', signed=False)
				if check_out == last_val:
					laser3_dist = laser3.get_distance(laser_rec_buf)
					if laser3_dist > laser_min_distance:
						# print("=======================挖斗激光%f" % laser3_dist)
						dou_laser_len = laser3_dist
						laserLed = True
						# 设置全局变量
						my_lock.pSor_laser_main_lock.acquire()
						gl.set_value("dou_laser_len", dou_laser_len)
						my_lock.pSor_laser_main_lock.release()
					else:
						laserLed = False
						print("激光距离错误\n")
				else:
					laserLed = False
					print("！！！！！ 激光校验失败 ！！！！\n")
		else:
			laserLed = False
		# 设置全局变量
		my_lock.laserLedLock.acquire()
		gl.set_value("laserLed", laserLed)
		my_lock.laserLedLock.release()

		time.sleep(0.15)
		# print("！！！！！laser线程！！！！！%s\n" % datetime.now().strftime('%H:%M:%S.%f'))


def _485ThreadFunc():
	log.debug("_485ThreadFunc 线程启动 ...")
	if SWITCH_DEVICE:
		BUS_COM = "com5"
	else:
		BUS_COM = "com7"
	GyroRecBufLen = (11 * 3)
	gyroChassisaddr_sensor = 0x50
	gyroBigArmaddr_sensor = 0x51
	gyroLittleArmaddr_sensor = 0x52

	bus485 = SerialPortCommunication(BUS_COM, 115200, 0.005)
	gyroChassisReadCmd = [gyroChassisaddr_sensor, 0x03, 0x00, 0x3d, 0x00, 0x03, 0x99, 0x86]
	gyroBigArmReadCmd = [gyroBigArmaddr_sensor, 0x03, 0x00, 0x3d, 0x00, 0x03, 0x98, 0x57]
	gyroLittleReadCmd = [gyroLittleArmaddr_sensor, 0x03, 0x00, 0x3d, 0x00, 0x03, 0x98, 0x64]

	""" 准备工作 """
	while True:
		gyroChassisAngle = []
		gyroBigArmAngle = []
		gyroLittleArmAngle = []

		chassis_data_ready = False
		big_data_ready = False
		little_data_ready = False

		# 全局变量初始化
		_485_data_valid_flg = False
		roll_2_di_pan = 0
		pitch_2_di_pan = 0
		roll_2_da_bi = 0
		roll_2_xiao_bi = 0

		chassisLen = bus485.send_data(gyroChassisReadCmd)
		time.sleep(0.001)
		if chassisLen == len(gyroChassisReadCmd):
			gyroChassisRecBuf = bus485.read_size(GyroRecBufLen)
			if gyroChassisRecBuf != b'':
				gyroChassisAngle = gyroDataAnalysis(gyroChassisRecBuf, gyroChassisaddr_sensor)
				chassis_data_ready = True
				roll_2_di_pan = gyroChassisAngle[0]
				pitch_2_di_pan = gyroChassisAngle[1]
			else:
				print("准备流程--地盘--接收数组为空\n")
		else:
			print("准备流程--地盘--地址发送失败\n")
		time.sleep(0.001)

		bigArmLen = bus485.send_data(gyroBigArmReadCmd)
		time.sleep(0.001)
		if bigArmLen == len(gyroBigArmReadCmd):
			gyroBigArmRecBuf = bus485.read_size(GyroRecBufLen)
			# TODO：判断条件
			if gyroBigArmRecBuf != b'':
				gyroBigArmAngle = gyroDataAnalysis(gyroBigArmRecBuf, gyroBigArmaddr_sensor)
				big_data_ready = True
				roll_2_da_bi = gyroBigArmAngle[0]
			else:
				print("准备流程--大臂--接收数组为空\n")
		else:
			print("准备流程--大臂--地址发送失败\n")
		time.sleep(0.001)

		littleArmLen = bus485.send_data(gyroLittleReadCmd)
		time.sleep(0.001)
		if littleArmLen == len(gyroLittleReadCmd):
			gyroLittleArmRecBuf = bus485.read_size(GyroRecBufLen)
			if gyroLittleArmRecBuf != b'':
				gyroLittleArmAngle = gyroDataAnalysis(gyroLittleArmRecBuf, gyroLittleArmaddr_sensor)
				little_data_ready = True
				roll_2_xiao_bi = gyroLittleArmAngle[0]
			else:
				print("准备流程--小臂--接收数组为空\n")
		else:
			print("准备流程--小臂--地址发送失败\n")

		# print("==============================485准备：", chassis_data_ready, big_data_ready, little_data_ready)
		if chassis_data_ready and big_data_ready and little_data_ready:
			_485_data_valid_flg = True
			# 设置全局变量
			my_lock.pSor_gyro_main_lock.acquire()
			gl.set_value("roll_2_di_pan", roll_2_di_pan)
			gl.set_value("pitch_2_di_pan", pitch_2_di_pan)
			gl.set_value("roll_2_da_bi", roll_2_da_bi)
			gl.set_value("roll_2_xiao_bi", roll_2_xiao_bi)
			gl.set_value("_485_data_valid_flg", _485_data_valid_flg)
			my_lock.pSor_gyro_main_lock.release()
			break
		else:
			print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCgyroChassisAngle:%s\n" % gyroChassisAngle)
			print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBgyroBigArmAngle:%s\n" % gyroBigArmAngle)
			print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLgyroLittleArmAngle:%s\n" % gyroLittleArmAngle)

		time.sleep(0.1)

	""" 正常流程 """
	while True:
		roll_2_di_pan = 0
		pitch_2_di_pan = 0
		roll_2_da_bi = 0
		roll_2_xiao_bi = 0

		gyroChassisAngle = []
		gyroBigArmAngle = []
		gyroLittleArmAngle = []
		time.sleep(0.001)
		if bus485.com.is_open:
			chassisLen = bus485.send_data(gyroChassisReadCmd)
			time.sleep(0.001)
			if chassisLen == len(gyroChassisReadCmd):
				gyroChassisRecBuf = bus485.read_size(GyroRecBufLen)
				if gyroChassisRecBuf != b'':
					gyroChassisAngle = gyroDataAnalysis(gyroChassisRecBuf, gyroChassisaddr_sensor)
					# print("===========================地盘角度：", gyroChassisAngle)
					gyroLedChassis = True
					roll_2_di_pan = gyroChassisAngle[0]
					pitch_2_di_pan = gyroChassisAngle[1]
				else:
					gyroLedChassis = False
					print("工作流程--底盘--接收数组为空\n")
			else:
				gyroLedChassis = False
				print("工作流程--底盘--发送失败\n")
			time.sleep(0.001)

			bigArmLen = bus485.send_data(gyroBigArmReadCmd)
			time.sleep(0.001)
			if bigArmLen == len(gyroBigArmReadCmd):
				gyroBigArmRecBuf = bus485.read_size(GyroRecBufLen)
				if gyroBigArmRecBuf != b'':
					gyroBigArmAngle = gyroDataAnalysis(gyroBigArmRecBuf, gyroBigArmaddr_sensor)
					# print("===========================大臂角度：", gyroBigArmAngle)
					gyroBigLed = True
					roll_2_da_bi = gyroBigArmAngle[0]
				else:
					gyroBigLed = False
					print("工作流程--大臂--接收数组为空\n")
			else:
				gyroBigLed = False
				# gl.set_value("gyroBigLed", False)
				print("工作流程--大臂--发送失败\n")
			time.sleep(0.001)

			littleArmLen = bus485.send_data(gyroLittleReadCmd)
			time.sleep(0.001)
			if littleArmLen == len(gyroLittleReadCmd):
				gyroLittleArmRecBuf = bus485.read_size(GyroRecBufLen)
				if gyroLittleArmRecBuf != b'':
					gyroLittleArmAngle = gyroDataAnalysis(gyroLittleArmRecBuf, gyroLittleArmaddr_sensor)
					# print("===========================小臂角度：", gyroLittleArmAngle)
					gyroLittleLed = True
					roll_2_xiao_bi = gyroLittleArmAngle[0]
				else:
					gyroLittleLed = False
					print("工作流程--小臂--接收数组为空\n")
			else:
				gyroLittleLed = False
				print("工作流程--小臂--发送失败\n")

			if gyroChassisAngle and gyroBigArmAngle and gyroLittleArmAngle:
				# 设置全局变量
				my_lock.pSor_gyro_main_lock.acquire()
				gl.set_value("roll_2_di_pan", roll_2_di_pan)
				gl.set_value("pitch_2_di_pan", pitch_2_di_pan)
				gl.set_value("roll_2_da_bi", roll_2_da_bi)
				gl.set_value("roll_2_xiao_bi", roll_2_xiao_bi)
				my_lock.pSor_gyro_main_lock.release()
		else:
			gyroLedChassis = False
			gyroBigLed = False
			gyroLittleLed = False

		my_lock.gyroLedLock.acquire()
		gl.set_value("gyroLedChassis", gyroLedChassis)
		gl.set_value("gyroBigLed", gyroBigLed)
		gl.set_value("gyroLittleLed", gyroLittleLed)
		my_lock.gyroLedLock.release()

		time.sleep(0.01)
		# print("！！！！！485线程！！！！！%s\n" % datetime.now().strftime('%H:%M:%S.%f'))
