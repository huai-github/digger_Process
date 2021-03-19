#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading

# 切换设备：True 电脑  False 工控机
SWITCH_DEVICE = True


class MyLock(object):
    def __init__(self):
        self.pMain_main_calc_lock = threading.RLock()
        self.pMain_win_main_lock = threading.RLock()
        self.pMain_calc_win_lock = threading.RLock()
        self.pMain_win_4g_lock = threading.RLock()
        self.pMain_win_4gSend_lock = threading.RLock()
        self.pMain_sensor_calc_lock = threading.RLock()
        self.pMain_calc_4gSend_lock = threading.RLock()

        self.pSor_gps_main_lock = threading.RLock()
        self.pSor_laser_main_lock = threading.RLock()
        self.pSor_gyro_main_lock = threading.RLock()
        self.p4g_rec_main_lock = threading.RLock()
        self.p4g_main_send_lock = threading.RLock()

        self.gpsStableLedLock = threading.RLock()
        self.gpsLedLock = threading.RLock()
        self.laserLedLock = threading.RLock()
        self.gyroLedLock = threading.RLock()


my_lock = MyLock()


def gl_init():
    global _global_dict
    _global_dict = {}


def set_value(name, value):
    _global_dict[name] = value


def get_value(name, defValue = 0):
    try:
        return _global_dict[name]
    except KeyError:
        return defValue



from logger import setup_logger
from color import *
log = setup_logger('logging.log')

