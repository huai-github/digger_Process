#!/usr/bin/python3
# coding = utf-8

import json
import socket
import threading
import time
from datetime import datetime

from _4g import task_switch_dict, Heart, SendMessage, get_xyhw
from globalvar import *
import globalvar as gl
from serial_port import SerialPortCommunication


class TimeInterval(object):
	def __init__(self, start_time, interval, callback_proc, args=None, kwargs=None):
		self.__timer = None
		self.__start_time = start_time
		self.__interval = interval
		self.__callback_pro = callback_proc
		self.__args = args if args is not None else []
		self.__kwargs = kwargs if kwargs is not None else {}

	def exec_callback(self, args=None, kwargs=None):
		self.__callback_pro(*self.__args, **self.__kwargs)
		self.__timer = threading.Timer(self.__interval, self.exec_callback)
		self.__timer.start()

	def start(self):
		interval = self.__interval - (datetime.now().timestamp() - self.__start_time.timestamp())
		# print( interval )
		self.__timer = threading.Timer(interval, self.exec_callback)
		self.__timer.start()

	def cancel(self):
		self.__timer.cancel()
		self.__timer = None


def _4RecFunc():
	while True:
		_4g_recv_flg = False
		rec_buf = com_4g.read_line()
		time.sleep(0.01)
		if rec_buf != b'':
			_4g_recv_flg = True
			my_lock.p4g_rec_main_lock.acquire()
			gl.set_value("_4g_recv_flg", _4g_recv_flg)
			my_lock.p4g_rec_main_lock.release()
		# else:
		# 	print("\n --4g未接收到任务+++++++++++++++++++++++++++++++++++++++++++ \n")

		if _4g_recv_flg:
			rec_buf_dict = task_switch_dict(rec_buf)  # 转成字典格式
			# 保存xyhw列表
			sx_list, sy_list, sh_list, sw_list, ex_list, ey_list, eh_list, ew_list = get_xyhw(rec_buf_dict)
			if sx_list and sy_list and sh_list and sw_list and ex_list and ey_list and eh_list and ew_list:
				# 设置全局变量
				my_lock.p4g_rec_main_lock.acquire()
				gl.set_value('g_start_x_list', sx_list)
				gl.set_value('g_start_y_list', sy_list)
				gl.set_value('g_start_h_list', sh_list)
				gl.set_value('g_start_w_list', sw_list)
				gl.set_value('g_end_x_list', ex_list)
				gl.set_value('g_end_y_list', ey_list)
				gl.set_value('g_end_h_list', eh_list)
				gl.set_value('g_end_w_list', ew_list)
				gl.set_value("_4g_data_valid_flg", True)
				my_lock.p4g_rec_main_lock.release()
			else:
				print("\n--任务解析失败--\n")

			time.sleep(0.2)


def _4SendFunc():
	""" 间隔5秒发送一次心跳 """
	heart = Heart(TYPE_HEART, diggerId)
	heart.send_heart_msg(com_4g)
	start = datetime.now().replace(minute=0, second=0, microsecond=0)
	minute = TimeInterval(start, 5, heart.send_heart_msg, [com_4g])
	minute.start()
	minute.cancel()

	""" 上报 """
	while True:
		# 发送o点x,y,h,w
		my_lock.p4g_main_send_lock.acquire()
		_4g_send_dict = gl.get_value("_4g_send_dict")
		my_lock.p4g_main_send_lock.release()

		if _4g_send_dict:
			o_min_flg = _4g_send_dict["o_min_flg"]
			o_x_4g = _4g_send_dict["x_send"]
			o_y_4g = _4g_send_dict["y_send"]
			o_h_4g = _4g_send_dict["h_send"]
			o_w_4g = _4g_send_dict["w_send"]

			if o_min_flg:
				send = SendMessage(TYPE_SEND, diggerId, round(o_x_4g / 1000, 3), round(o_y_4g / 1000, 3),
				                   round(o_h_4g / 1000, 3), o_w_4g)  # m
				send_msg_json = send.switch_to_json()
				com_4g.send_data(send_msg_json.encode('utf-8'))
				com_4g.send_data('\n'.encode('utf-8'))
				my_lock.p4g_main_send_lock.acquire()
				gl.set_value("o_min_flg", False)
				my_lock.p4g_main_send_lock.release()
		time.sleep(0.2)


def _4gProcess():
	gl.gl_init()
	global COM_ID_4G, TYPE_HEART, TYPE_SEND, com_4g, diggerId
	if SWITCH_DEVICE:
		COM_ID_4G = "com13"
	else:
		COM_ID_4G = "com4"
	TYPE_HEART = 1  # 消息类型。1：心跳，2：上报
	TYPE_SEND = 2
	diggerId = 566609996553388032
	com_4g = SerialPortCommunication(COM_ID_4G, 115200, 0.1)

	# 接收任务socket
	sk_recv_tsk = socket.socket()
	sk_recv_tsk.connect(("127.0.0.1", 9000))
	# 上报消息socket
	sk_send = socket.socket()
	sk_send.bind(("127.0.0.1", 9999))
	sk_send.listen()
	conn_send, addr_send = sk_send.accept()
	sk_send.setblocking(False)

	_4g_recv_thread = threading.Thread(target=_4RecFunc, daemon=True).start()
	_4g_send_thread = threading.Thread(target=_4SendFunc, daemon=True).start()

	""" 4g 主线程 """
	while True:
		time.sleep(0.2)

		my_lock.p4g_rec_main_lock.acquire()
		_4g_recv_flg = gl.get_value("_4g_recv_flg")
		my_lock.p4g_rec_main_lock.release()

		if _4g_recv_flg:
			my_lock.p4g_rec_main_lock.acquire()
			gl.set_value("_4g_recv_flg", False)
			_4g_data_dict = {
				"_4g_data_valid_flg": gl.get_value("_4g_data_valid_flg"),
				"g_start_x_list": gl.get_value('g_start_x_list'),
				"g_start_y_list": gl.get_value('g_start_y_list'),
				"g_start_h_list": gl.get_value('g_start_h_list'),
				"g_start_w_list": gl.get_value('g_start_w_list'),
				"g_end_x_list": gl.get_value('g_end_x_list'),
				"g_end_y_list": gl.get_value('g_end_y_list'),
				"g_end_h_list": gl.get_value('g_end_h_list'),
				"g_end_w_list": gl.get_value('g_end_w_list'),
			}
			my_lock.p4g_rec_main_lock.release()

			_4g_data_bytes = json.dumps(_4g_data_dict)
			sk_recv_tsk.send(bytes(_4g_data_bytes.encode('utf-8')))

		# 上报
		_4g_send_bytes = conn_send.recv(1024)
		if _4g_send_bytes:
			_4g_send_dict = json.loads(_4g_send_bytes)
			my_lock.p4g_main_send_lock.acquire()
			gl.set_value("_4g_send_dict", _4g_send_dict)
			my_lock.p4g_main_send_lock.release()






