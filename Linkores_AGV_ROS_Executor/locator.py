import asyncio
import logging
import threading
import math
import time
import numpy as np
from pyquaternion import Quaternion  # 时间较长  #旋转向量用
import config
from log import logger
from tof_camera import tof_camera_data
async def do_connect(loop, host, port, protocol):
    while True:
        try:
            await loop.create_connection(lambda: protocol, host, port)
        except OSError as e:
            print("locator connect retrying in 3 seconds") #连接未成功，3秒后再重连
            await asyncio.sleep(5)
        else:
            break

schedule_offset_phi = 0.0  # 正值向逆时针方向旋转（单位：角度）
schedule_offset_x = 0.0  # 正值向正轴方向移动 （单位：m））
schedule_offset_y = 0.0


class Locator(asyncio.Protocol):
    def __init__(self, loop):
        self.loop = loop
        self.state = False
        # 雷达角度补偿
        self.cfg_radar_org_offset = 0 # 1° == 1000
        #[雷达到从动轮距离，雷达左右偏移（+ 增加左边），旋转角度]
        self.offset = config.locator_offset # 0.91
        self.tof_offset = config.tof_camera_offset
        self._lock = threading.Lock()
        self._value = [0, 0, 0]
    
    def getValue(self):
        self._lock.acquire()
        res = self._value.copy()
        self._lock.release()
        return res

    def setValue(self, value):
        self._lock.acquire()
        self._value = value.copy()
        self._lock.release()

    # 连接server后执行
    def connection_made(self, transport):
        self._transport = transport
        print('locator connected')
        self.host, self.port = transport.get_extra_info('peername')
        message = b'\x02sMN mNEVAChangeState 1\x03'
        transport.write(message)
        time.sleep(0.5)
        # print('Data sent: {}'.format(message))
        message = b'\x02sMN mNEVAChangeState 4\x03'
        transport.write(message)
        time.sleep(1)
        # print('Data sent: {}'.format(message))

    
    # 接收server返回的数据
    def data_received(self, data):
        global schedule_offset_phi
        global schedule_offset_x
        global schedule_offset_y
        # print('Data received: {}'.format(data))
        if data == b'\x02sMA mNEVAChangeState\x03\x02sAN mNEVAChangeState 0 1\x03\x02sMA mNEVAChangeState\x03\x02sAN mNEVAChangeState 0 4\x03':
            self.state = True
            return
        elif data == b'\x02sMA mNPOSGetPose\x03':
            return
        logger.info('RadarData received: {}'.format(data))

        formatAngle = lambda x: x + 2 * math.pi if x < -math.pi else x - 2 * math.pi if x > math.pi else x
        Hex32toInt = lambda x: int(x, 16) - ((int(x, 16) >> 31) << 32)
        try:
            strlst = data.decode().split('\x03')
            for msg in strlst:
                lst = msg.split(' ')
                if (len(lst) == 10 or len(lst) == 16) and lst[0] == '\x02sAN' and lst[1] == 'mNPOSGetPose':
                    x = Hex32toInt(lst[6]) / 1000.0
                    y = Hex32toInt(lst[7]) / 1000.0
                    phi = formatAngle((Hex32toInt(lst[8]) + self.cfg_radar_org_offset) / 180000.0 * math.pi - self.offset[2])
                    tmp = np.array([x, y, 0]) - Quaternion(axis=[0, 0, 1], angle=phi).rotate(np.array([self.offset[0], self.offset[1], 0]))
                    # TOF offset
                    phi_tof = formatAngle((Hex32toInt(lst[8]) + self.cfg_radar_org_offset) / 180000.0 * math.pi - self.tof_offset[2])
                    tmp_tof = np.array([x, y, 0]) - Quaternion(axis=[0, 0, 1], angle=phi).rotate(np.array([self.tof_offset[0], self.tof_offset[1], 0]))
                    tof_camera_data.send_tof = [tmp_tof[0], tmp_tof[1], phi_tof, tof_camera_data.tof_height]
                    tof_camera_data.send_to_tof()

                    # print(time.strftime('%H:%M:%S'), "aa", tmp[0], tmp[1], phi)
                    # self.setValue([tmp[0], tmp[1], phi])
                    offset_phi = round(schedule_offset_phi / 180 * math.pi, 3)
                    self.setValue([tmp[0] - schedule_offset_x, tmp[1] - schedule_offset_y, phi - offset_phi])
                    #print("X:", tmp[0], " Y:", tmp[1], " PHI", phi, "offset:", schedule_offset_x, schedule_offset_y, schedule_offset_phi)

        except Exception as e:
            raise e

    # 与server断开连接后执行
    def connection_lost(self, exc):
        print('radar disconnected')
        logger.warning("radar disconnected")
        # asyncio.async(do_connect(self.loop, self.host, self.port, self))
        # asyncio.get_event_loop().create_task(do_connect(self.loop, self.host, self.port, self))
        # asyncio.create_task(do_connect(self.loop, self.host, self.port, self))  # python3.7以上
        asyncio.ensure_future(do_connect(self.loop, self.host, self.port, self))

    def send_data(self):
        message = b'\x02sMN mNPOSGetPose 1\x03'
        self._transport.write(message)

ip = config.locator_ip
port = 2112
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
locator_protocol = Locator(loop)

def foo1_thread():
    loop.run_until_complete(do_connect(loop, ip, port, locator_protocol))
    loop.run_forever()
    loop.close()

def foo2_thread():
    while True:
        while locator_protocol.state:
            locator_protocol.send_data()
            time.sleep(0.125)
        time.sleep(1)


test_thread1 = threading.Thread(target=foo1_thread)
test_thread2 = threading.Thread(target=foo2_thread)
test_thread1.start()
test_thread2.start()
time.sleep(3)
print("雷达坐标获取延迟3秒，等待有效数据")

if __name__ == "__main__":
    def foo3_thread():
        while True:
            while locator_protocol.state:
                print(locator_protocol.getValue())
                time.sleep(0.125)
            time.sleep(1)
            
    test_thread3 = threading.Thread(target=foo3_thread)
    test_thread3.start()      

    while True:
        time.sleep(1)