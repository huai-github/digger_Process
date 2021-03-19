#!/usr/bin/python3
# coding = utf-8

import sys
import threading
import time
from multiprocessing import Process, Queue, Pipe
import socket
import json

from _4g_process import _4gProcess
from sensor_process import sensorProcess
from PyQt5.QtCore import Qt, QDateTime
from PyQt5.QtGui import QImage, QPixmap, QPainter
from PyQt5.QtWidgets import QApplication, QWidget
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import WinMaintn
import calculate_main
import digger_ui
import globalvar as gl
from globalvar import *
import threading

h = 480  # 画布大小
w = 550

x_min = 0
y_min = 0
x_max = 0
y_max = 0

zoom_x = 0
zoom_y = 0
delta = 5

sensor_data_dict = {}


class MyWindows(QWidget, digger_ui.Ui_Digger):
	def __init__(self):
		super().__init__()
		self.setupUi(self)

		self.imgLine = np.zeros((h, w, 3), np.uint8)  # 画布
		self.imgBar = np.zeros((h, w, 3), np.uint8)
		self.figure = plt.figure()  # 可选参数, facecolor为背景颜色
		self.canvas = FigureCanvas(self.figure)
		self.DeepList = [0, 0, 0, 0, 0]
		self.NumList = [0, 0, 0, 0, 0]

		# 校准按钮
		self.calibration_button.clicked.connect(self.calibrationBtnFunc)

	def calibrationBtnFunc(self):
		self.calibration_text.setText("已按下校准按钮。。。")
		self.hide()  # 隐藏此窗口
		# self.calibrationUi = runCalibrationUi.CalibrationUi()
		# self.calibrationUi.showFullScreen()  # 窗口全屏显示
		# self.calibrationUi.showMaximized()
		self.calibrationUi.show()  # 经第二个窗口显示出来

	def paintEvent(self, e):
		my_lock.pMain_win_main_lock.acquire()
		lftWinImg = gl.get_value("lftWinImg")
		RitWinImg = gl.get_value("RitWinImg")
		lftWinImgFlg = gl.get_value("lftWinImgFlg")
		RitWinImgFlg = gl.get_value("RitWinImgFlg")
		# current_work_high = gl.get_value("current_work_high")  # 当前点的施工高度
		dist = gl.get_value("dist")
		deep = gl.get_value("deep")
		my_lock.pMain_win_main_lock.release()

		my_lock.gpsStableLedLock.acquire()
		gps_stable_flag = gl.get_value("gps_stable_flag")  # gpsUI
		my_lock.gpsStableLedLock.release()

		my_lock.gpsLedLock.acquire()
		gps_is_open_led = gl.get_value("gps_is_open_led")  # gps串口是否打开
		my_lock.gpsLedLock.release()

		my_lock.laserLedLock.acquire()
		laserLed = gl.get_value("laserLed")  # laserUI
		my_lock.laserLedLock.release()

		my_lock.gyroLedLock.acquire()
		gyroLedChassis = gl.get_value("gyroLedChassis")  # 485UI
		gyroBigLed = gl.get_value("gyroBigLed")  # 485UI
		gyroLittleLed = gl.get_value("gyroLittleLed")  # 485UI
		my_lock.gyroLedLock.release()

		qp = QPainter()
		qp.begin(self)
		"""GPS信号指示灯"""
		if gps_stable_flag:
			self.gps_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:green")
		else:
			self.gps_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:red")

		"""边界信号指示灯"""
		if dist == -1:
			self.edge_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:red")
		elif dist == 1 or dist == 0:
			self.edge_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:green")
		else:
			self.edge_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:yellow")

		""" gps串口是否打开 """
		if gps_is_open_led:
			pass
		else:
			pass

		""" 深度指示灯 """
		if deep > 0:
			self.deep_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:green")
		else:
			self.deep_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:red")

		""" 激光指示灯 """
		if laserLed:
			self.laser_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:green")
		else:
			self.laser_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:red")

		""" 陀螺仪指示灯 """
		if gyroLedChassis:
			self.gyro_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:green")
		else:
			self.gyro_led.setStyleSheet(
				"min-width:40px;min-height:40px;max-width:40px;max-height:40px;border-radius:20px;border:1pxsolidblack;background:red")

		"""维护界面"""
		# 维护左窗口
		if lftWinImgFlg:
			QtImgLine = QImage(cv.cvtColor(lftWinImg, cv.COLOR_BGR2RGB).data,
			                   lftWinImg.shape[1],
			                   lftWinImg.shape[0],
			                   lftWinImg.shape[1] * 3,  # 每行的字节数, 彩图*3
			                   QImage.Format_RGB888)
			pixmapL = QPixmap(QtImgLine)
			self.leftLabel.setPixmap(pixmapL)
			self.leftLabel.setScaledContents(True)

		# 维护右窗口
		if RitWinImgFlg:
			QtImgLine = QImage(cv.cvtColor(RitWinImg, cv.COLOR_BGR2RGB).data,
			                   RitWinImg.shape[1],
			                   RitWinImg.shape[0],
			                   RitWinImg.shape[1] * 3,  # 每行的字节数, 彩图*3
			                   QImage.Format_RGB888)
			pixmapL = QPixmap(QtImgLine)
			self.rightLabel.setPixmap(pixmapL)
			self.rightLabel.setScaledContents(True)

		# """显示挖掘机ID"""
		# self.diggerID.setText(str(digger_id))

		"""显示时间"""
		date = QDateTime.currentDateTime()
		current_time = date.toString("yyyy-MM-dd hh:mm dddd")
		self.time.setText(current_time)

		qp.end()
		# 刷新
		self.setUpdatesEnabled(True)
		self.update()


def _4gUpdateThreadFunc():
	sk_recv_tsk = socket.socket()
	sk_recv_tsk.bind(("127.0.0.1", 9000))
	sk_recv_tsk.listen()
	conn_recv_tsk, addr_recv_tsk = sk_recv_tsk.accept()
	sk_recv_tsk.setblocking(False)
	while True:
		_4g_data_bytes = conn_recv_tsk.recv(1024)
		_4g_data_dict = json.loads(_4g_data_bytes)
		log.debug(light_blue(f"_4g_data_dict:{_4g_data_bytes}\n"))

		my_lock.pMain_win_4g_lock.acquire()
		gl.set_value("_4g_data_dict", _4g_data_dict)
		gl.set_value("_4g_data_valid_flg", True)
		my_lock.pMain_win_4g_lock.release()


def _4gSendThreadFunc():
	sk_send = socket.socket()
	sk_send.connect(("127.0.0.1", 9999))

	# 上报x,y,h,w--来自计算线程,维护线程
	my_lock.pMain_calc_4gSend_lock.acquire()
	o_min_flg = gl.get_value("o_min_flg")
	x_send = gl.get_value("x_send")  # 毫米
	y_send = gl.get_value("y_send")
	h_send = gl.get_value("h_send")
	my_lock.pMain_calc_4gSend_lock.release()

	my_lock.pMain_win_4gSend_lock.acquire()
	w_send = gl.get_value("o_w_4g")
	my_lock.pMain_win_4gSend_lock.release()

	_4g_send_dict = {
		"o_min_flg": o_min_flg,
		"x_send": x_send,
		"y_send": y_send,
		"h_send": h_send,
		"w_send": w_send,
	}

	_4g_send_bytes = json.dumps(_4g_send_dict)
	sk_send.send(bytes(_4g_send_bytes.encode('utf-8')))


def sensorUpdateThreadFunc():
	global sensor_data_dict
	while True:
		time.sleep(0.1)
		sensor_data_dict = sensor_q.get()
		my_lock.pMain_main_calc_lock.acquire()
		gl.set_value("sensor_data_dict", sensor_data_dict)
		my_lock.pMain_main_calc_lock.release()
		log.debug(light_blue(f"sensor_data_dict:{sensor_data_dict}\n"))

		if sensor_data_dict["sensor_data_valid_flg"]:
			break
		else:
			print("主进程--主线程--传感器未准备完成：%s\t:"
			      % (sensor_data_dict["sensor_data_valid_flg"]))

	while True:
		sensor_data_dict = sensor_q.get()

		my_lock.pMain_sensor_calc_lock.acquire()
		gl.set_value("sensor_data_dict", sensor_data_dict)
		my_lock.pMain_sensor_calc_lock.release()

		log.debug(light_blue(f"sensor_data_dict:{sensor_data_dict}\n"))

		time.sleep(0.1)


if __name__ == '__main__':
	gl.gl_init()
	sensor_q = Queue()
	sensor_p = Process(target=sensorProcess, args=(sensor_q, )).start()
	_4g_p = Process(target=_4gProcess).start()

	calculate_thread = threading.Thread(target=calculate_main.calcThreadFunc, daemon=True).start()
	WinMaintn_thread = threading.Thread(target=WinMaintn.WinMaintnFun, daemon=True).start()
	_4g_update_thread = threading.Thread(target=_4gUpdateThreadFunc, daemon=True).start()
	_4g_send_thread = threading.Thread(target=_4gSendThreadFunc, daemon=True).start()
	sensor_update_thread = threading.Thread(target=sensorUpdateThreadFunc, daemon=True).start()

	while True:
		my_lock.pMain_win_main_lock.acquire()
		main_win_data_valid_flg = gl.get_value("main_win_data_valid_flg")
		my_lock.pMain_win_main_lock.release()

		if main_win_data_valid_flg:
			break
			
	app = QApplication(sys.argv)
	mainWindow = MyWindows()
	mainWindow.show()
	sys.exit(app.exec_())




