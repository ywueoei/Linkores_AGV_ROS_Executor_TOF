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


def agv_logs(*args):
    s = time.time()
    s = time.strftime("%y.%m.%d %H:%M:", time.localtime(s)) + str(float("{0:.3f}".format(s % 60)))
    print("[" + s + "]", *args)



class EchoClientProtocol:
    def __init__(self, message, loop):
        self.message = message
        self.loop = loop
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport
        print('Send:', self.message)
        # self.transport.write(self.message)
        self.transport.sendto(self.message)

    def datagram_received(self, data, addr):
        logger.info(data.decode())
        # print("Close the socket")
        # self.transport.close()

    def error_received(self, exc):
        print('Error received:', exc)

    def connection_lost(self, exc):
        print("Socket closed, stop the event loop")
        loop = asyncio.get_event_loop()
        loop.stop()


if os.path.exists("_config_override.py"):
    from _config_override import cfg_nav350_ip # 旧版本
    ip = cfg_nav350_ip
else:
    from config import locator_ip
    ip = locator_ip
port = 6060


def foo_thread():
    # loop = asyncio.get_event_loop()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    message = "Hello World!"
    command = bytearray([0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x88,0x01,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x92,0xEE,0xEE])
    connect = loop.create_datagram_endpoint(
        lambda: EchoClientProtocol(command, loop),
        remote_addr=(ip, port))
    transport, protocol = loop.run_until_complete(connect)
    loop.run_forever()
    transport.close()
    loop.close()
   
test_thread1 = threading.Thread(target=foo_thread)
test_thread1.start()


