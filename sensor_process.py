#!/usr/bin/python3
# coding = utf-8

import threading
import time

import sensor_thread
import globalvar as gl
from globalvar import my_lock

from globalvar import *


def sensorProcess(sensor_q):
	gl.gl_init()

	gps_thread = threading.Thread(target=sensor_thread.gpsThreadFunc, daemon=True)
	_485_thread = threading.Thread(target=sensor_thread._485ThreadFunc, daemon=True)
	g_laser_dou_thread = threading.Thread(target=sensor_thread.laserThreadFunc, daemon=True)

	gps_thread.start()
	_485_thread.start()
	g_laser_dou_thread.start()

	while True:
		time.sleep(0.1)
		""" 准备流程 """
		my_lock.pSor_gps_main_lock.acquire()
		gps_data_valid_flg = gl.get_value("gps_data_valid_flg")
		gps_x = gl.get_value("gps_x")
		gps_y = gl.get_value("gps_y")
		gps_h = gl.get_value("gps_h")
		gps_yaw = gl.get_value("gps_yaw")
		my_lock.pSor_gps_main_lock.release()

		my_lock.pSor_laser_main_lock.acquire()
		laser_data_valid_flg = gl.get_value("laser_data_valid_flg")
		dou_laser_len = gl.get_value("dou_laser_len")
		my_lock.pSor_laser_main_lock.release()

		my_lock.pSor_gyro_main_lock.acquire()
		_485_data_valid_flg = gl.get_value("_485_data_valid_flg")
		roll_2_di_pan = gl.get_value("roll_2_di_pan")
		pitch_2_di_pan = gl.get_value("pitch_2_di_pan")
		roll_2_da_bi = gl.get_value("roll_2_da_bi")
		roll_2_xiao_bi = gl.get_value("roll_2_xiao_bi")
		my_lock.pSor_gyro_main_lock.release()

		if gps_data_valid_flg and laser_data_valid_flg and _485_data_valid_flg:
			sensor_data_dict = {
				"sensor_data_valid_flg": True,
				"gps_x": gps_x,
				"gps_y": gps_y,
				"gps_h": gps_h,
				"gps_yaw": gps_yaw,
				"dou_laser_len": dou_laser_len,
				"roll_2_di_pan": roll_2_di_pan,
				"pitch_2_di_pan": pitch_2_di_pan,
				"roll_2_da_bi": roll_2_da_bi,
				"roll_2_xiao_bi": roll_2_xiao_bi,
			}
			sensor_q.put(sensor_data_dict)
			break
		else:
			print("\n！！！！！传感器数据为准备完成:gps:%s\t 485:%s\t laser:%s\n" % (gps_data_valid_flg, _485_data_valid_flg, laser_data_valid_flg))

	""" 正常工作流程 """
	while True:
		my_lock.pSor_gps_main_lock.acquire()
		gps_x = gl.get_value("gps_x")
		gps_y = gl.get_value("gps_y")
		gps_h = gl.get_value("gps_h")
		gps_yaw = gl.get_value("gps_yaw")
		my_lock.pSor_gps_main_lock.release()

		my_lock.pSor_laser_main_lock.acquire()
		dou_laser_len = gl.get_value("dou_laser_len")
		my_lock.pSor_laser_main_lock.release()

		my_lock.pSor_gyro_main_lock.acquire()
		roll_2_di_pan = gl.get_value("roll_2_di_pan")
		pitch_2_di_pan = gl.get_value("pitch_2_di_pan")
		roll_2_da_bi = gl.get_value("roll_2_da_bi")
		roll_2_xiao_bi = gl.get_value("roll_2_xiao_bi")
		my_lock.pSor_gyro_main_lock.release()

		sensor_data_dict = {
			"sensor_data_valid_flg": True,
			"gps_x": gps_x,
			"gps_y": gps_y,
			"gps_h": gps_h,
			"gps_yaw": gps_yaw,
			"dou_laser_len": dou_laser_len,
			"roll_2_di_pan": roll_2_di_pan,
			"pitch_2_di_pan": pitch_2_di_pan,
			"roll_2_da_bi": roll_2_da_bi,
			"roll_2_xiao_bi": roll_2_xiao_bi,
		}
		sensor_q.put(sensor_data_dict)

		time.sleep(0.1)

