import asyncio
import threading
import time
import logging
from logging import handlers
import pathlib
import os
#import config

class Logger(object):
    level_relations = {
        'notset':logging.NOTSET, #不用，使用情况跟warning一样
        'debug':logging.DEBUG,
        'info':logging.INFO,
        'warning':logging.WARNING,
        'error':logging.ERROR,
        'crit':logging.CRITICAL
    } #日志级别关系映射

    def __init__(self, filename, level='debug', when='M', backCount=(6 * 24 * 1), fmt='%(asctime)s - %(levelname)s: %(message)s'):
        self.logger = logging.getLogger(filename)
        format_str = logging.Formatter(fmt) #设置日志格式
        self.logger.setLevel(self.level_relations.get(level)) #设置日志级别
        stream_handler = logging.StreamHandler() #往屏幕上输出
        stream_handler.setFormatter(format_str) #设置屏幕上显示的格式
        file_handler = handlers.TimedRotatingFileHandler(filename=filename, when=when, interval=10, backupCount=backCount, encoding='utf-8') #往文件里写入#指定间隔时间自动生成文件的处理器
        #实例化TimedRotatingFileHandler
        #interval是时间间隔，backupCount是备份文件的个数，如果超过这个个数，就会自动删除，when是间隔的时间单位，单位有以下几种：
        # S 秒
        # M 分
        # H 小时、
        # D 天、
        # W 每星期（interval==0时代表星期一）
        # midnight 每天凌晨
        file_handler.setFormatter(format_str)#设置文件里写入的格式
        # self.logger.addHandler(stream_handler) #把对象加到logger里
        self.logger.addHandler(file_handler)

agv_log_dir = pathlib.Path("/home/pi/RadarLog")
if not agv_log_dir.is_dir():
    os.system("mkdir /home/pi/RadarLog")

log_file_name = "/home/pi/RadarLog/" + "radar" + ".log"
logger = Logger(log_file_name).logger


# 获取雷达日志
async def do_connect(loop, host, port, protocol):
    while True:
        try:
            await loop.create_connection(lambda: protocol, host, port)
        except OSError as e:
            print("client retrying connect in 3 seconds")  # 连接未成功，3秒后再重连
            await asyncio.sleep(1)
        else:
            break


def agv_log(*args):
    s = time.time()
    s = time.strftime("%y.%m.%d %H:%M:", time.localtime(s)) + str(float("{0:.3f}".format(s % 60)))
    print("[" + s + "]", *args)


class RadarClient(asyncio.Protocol):
    def __init__(self, loop):
        self.loop = loop
        self.connect_state = False
        self.ad_value = 0
        self.command_old = bytearray([0x55, 0xaa, 0x1e, 0x01])
        # 新雷达使用 port 6060
        self.command_new = bytearray([0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x88,0x01,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x92,0xEE,0xEE])

        self.send_time = None
        self.recv_time = None

    # 连接server后执行
    def connection_made(self, transport):
        self.connect_state = True
        self._transport = transport
        self.host, self.port = transport.get_extra_info('peername')
        print('Service Connected')
        self.send_data()
        self.recv_time = time.time()
        self.send_time = time.time()

    # 接收server返回的数据
    def data_received(self, data):
        self.recv_time = time.time()
        self.ad_value = data
        logger.info(data)
        #agv_log(data)
        # if len(data) != 11:
        #     self.ad_value = 0
        # else:
        #     self.ad_value = int.from_bytes(data[9:11], 'big')  # * 0.001

        # self.data_send()
        # time.sleep(0.2)

    # 与server断开连接后执行
    def connection_lost(self, exc):
        print('Service Disconnected')
        self.connect_state = False
        # asyncio.async(do_connect(self.loop, self.host, self.port, self))
        asyncio.ensure_future(do_connect(self.loop, self.host, self.port, self))

    def get_ad_value(self):
        # print(self.connect_state, self.send_time, self.recv_time)
        if self.connect_state:
            #self.send_data()
            pass

        if (not self.connect_state) or (self.send_time == None or self.recv_time == None) or abs(
                self.send_time - self.recv_time) > 1:  # 一般来说send_time 大于 recv_time
            self.send_time = time.time()
            return 0
        self.send_time = time.time()
        return self.ad_value

    def send_data(self):
        self._transport.write(self.command_old)
        time.sleep(0.05)
        self._transport.write(self.command_new)
        print("send", self.command_new)


if os.path.exists("_config_override.py"):
    from _config_override import cfg_nav350_ip # 旧版本
    ip = cfg_nav350_ip
else:
    from config import locator_ip
    ip = locator_ip
port = 7070
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
radar_client = RadarClient(loop=loop)

def foo1_thread():
    loop.run_until_complete(do_connect(loop, ip, port, radar_client))
    loop.run_forever()
    loop.close()


# def foo2_thread():
#     while True:
#         print(ad_client.ad_value)
#         time.sleep(0.2)

# def foo2_thread():
#     while True:
#         while ad_client.connect_state:
#             if ad_client.get_ad_value() != None:
#                 print(ad_client.ad_value)
#             time.sleep(0.2)
#         time.sleep(1)

test_thread1 = threading.Thread(target=foo1_thread)
# test_thread2 = threading.Thread(target=foo2_thread)
test_thread1.start()
# test_thread2.start()