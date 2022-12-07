#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
------------------------------------
# @Time    : 2022/10/9 16:16
# @Author  : Yueoei
# @File    : locator_client.py
# @Description : SLAM雷达坐标获取接口
------------------------------------
"""
import time

import config
from asyncio_tcp_client import EchoClientProtocol
from frequency import FrequencyDecoratorAsync
from frequency import FrequencyMonitorDecorator
import traceback
import asyncio
import json
import math
from pyquaternion import Quaternion  # 时间较长  #旋转向量用
import numpy as np
import threading
from tof_camera import tof_camera_data

formatAngle = lambda x: x + 2 * math.pi if x < -math.pi else x - 2 * math.pi if x > math.pi else x


class LocaorClient(EchoClientProtocol):
    def __init__(self, host, port, name=None):
        super().__init__(host, port, name)
        self.__lock = threading.Lock()
        self.command = b'GET /api/agv/info HTTP/1.1\r\nHost: 192.168.137.155:8222\r\n\r\n'
        self.data_buffer = b''
        self.error_count = 0
        self.normal_count = 0
        self.offset = [0.7927, 0, 0]
        self.cfg_radar_org_offset = 0
        # self.offset = config.locator_offset # 0.91
        self.tof_offset = [0.32, -0.019, 0]  # config.tof_camera_offset
        # self.tof_offset = [0, 0, 0]  # config.tof_camera_offset
        self.__value = [0] * 4
        self.judgment = 0
        self.state = False  # 兼容旧

    # 减小时间间隔所产生的误差
    @FrequencyDecoratorAsync(0.05)
    # @FrequencyMonitorDecorator(0.02)
    def analysis_data(self):
        while not self.data_queue.empty():
            self.data_buffer += self.data_queue.get()
            while (b'\n{' in self.data_buffer) and (b'}\r' in self.data_buffer):
                start = self.data_buffer.index(b'\n{') + 1
                end = self.data_buffer.index(b'}\r') + 1
                if end > start:
                    try:
                        msg_frame = self.data_buffer[start: end]
                        self.data_buffer = self.data_buffer[end:]
                        # print(msg_frame)
                        js_data = json.loads(msg_frame.decode('utf-8'))
                        x = js_data.get("data")[0]['x'] / 1000
                        y = js_data.get("data")[0]['y'] / 1000
                        angle = js_data.get("data")[0]['angle']  # [-180, 180]
                        phi = formatAngle((angle + self.cfg_radar_org_offset) / 180.0 * math.pi - self.offset[2])
                        values = np.array([x, y, 0]) - Quaternion(axis=[0, 0, 1], angle=phi).rotate(np.array([self.offset[0], self.offset[1], 0]))
                        if self.__value[0:3] == [values[0], values[1], phi]:
                            if self.error_count < 5:
                                self.error_count += 1
                            else:
                                self.judgment = 0
                                self.normal_count = 0
                        else:
                            self.error_count = 0
                            if self.normal_count < 3:
                                self.normal_count += 1
                            else:
                                self.judgment = 1

                        # TOF offset
                        phi_tof = phi  # formatAngle((angle + self.cfg_radar_org_offset) / 180000.0 * math.pi -self.tof_offset[2])
                        tmp_tof = np.array([x, y, 0]) - Quaternion(axis=[0, 0, 1], angle=phi).rotate(np.array([self.tof_offset[0], self.tof_offset[1], 0]))
                        tof_camera_data.send_tof = [tmp_tof[0], tmp_tof[1], phi_tof, tof_camera_data.tof_height]
                        tof_camera_data.send_to_tof()

                        self.set_value([values[0], values[1], phi, self.judgment])
                        print("x0: {}, y0: {}, angle: {}".format(x, y, angle))
                        print("x: {}, y: {}, angle: {}".format(values[0], values[1], math.degrees(phi)))
                    except:
                        print(traceback.format_exc())
                        self.data_buffer = b''
                else:
                    self.data_buffer = self.data_buffer[end:]

    def getValue(self):
        return self.get_value()

    def setValue(self, value):
        self.set_value(value)

    def set_value(self, value):
        self.__lock.acquire()
        self.__value = value.copy()
        self.__lock.release()

    def get_value(self):
        self.__lock.acquire()
        value = self.__value.copy()
        self.__lock.release()
        return value

    async def analysis_data_loop(self):
        while True:
            try:
                res = self.analysis_data()
                sleep_time = res[1]
            except:
                print(traceback.format_exc())
                # logger.error(traceback.format_exc())
                print(traceback.format_exc())
            await asyncio.sleep(sleep_time)

        # while True:
        #     if not self.data_queue.empty():
        #         print(self.data_queue.get())
        #     await asyncio.sleep(0.05)

    @FrequencyDecoratorAsync(0.066)
    def command_send(self):
        self.data_send(self.command)

    async def command_send_loop(self):
        while True:
            try:
                res = self.command_send()
                sleep_time = res[1]
            except:
                print(traceback.format_exc())
                # logger.error(traceback.format_exc())
            await asyncio.sleep(sleep_time)


locator_protocol = LocaorClient('192.168.137.155', 8222, "导航雷达")
from coroutines_client import Coroutines
coroutines = Coroutines()
coroutines.start()
coroutines.add_coroutines(locator_protocol.tcp_connect())
coroutines.add_coroutines(locator_protocol.analysis_data_loop())
coroutines.add_coroutines(locator_protocol.command_send_loop())

if __name__ == "__main__":
    from coroutines_client import Coroutines

    coroutines = Coroutines()
    coroutines.start()
    coroutines.add_coroutines(locator_protocol.tcp_connect())
    coroutines.add_coroutines(locator_protocol.analysis_data_loop())
    coroutines.add_coroutines(locator_protocol.command_send_loop())
    while 1:
        print(locator_protocol.get_value())
        time.sleep(0.1)