import logging
import time
from logging import handlers
import pathlib
import os
import config
import queue
import threading

class Logger(object):
    level_relations = {
        'notset':logging.NOTSET, #不用，使用情况跟warning一样
        'debug':logging.DEBUG,
        'info':logging.INFO,
        'warning':logging.WARNING,
        'error':logging.ERROR,
        'crit':logging.CRITICAL
    } #日志级别关系映射

    def __init__(self, filename, level='debug', when='M', backCount=(6 * 24 * 7), fmt='%(asctime)s - %(levelname)s: %(message)s'):
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

agv_log_dir = pathlib.Path("/home/pi/agv_log")
if not agv_log_dir.is_dir():
    os.system("mkdir /home/pi/agv_log")

log_file_name = "/home/pi/agv_log/" + config.cfg_car_Num + ".log"
# logger = Logger(log_file_name).logger


class my_logger():
    log_queue = queue.Queue(10000)
    def debug(self, msg):
        self.log_queue.put("debug" + str(msg))
    def info(self, msg):
        self.log_queue.put("info" + str(msg))
    def warning(self, msg):
        self.log_queue.put("warning" + str(msg))
    def error(self, msg):
        self.log_queue.put("error" + str(msg))

logger = my_logger()
def foo():
    log_obj = Logger(log_file_name).logger
    while True:
        this_time = time.time()
        while not logger.log_queue.empty():
            msg = logger.log_queue.get()
            if msg[:5] == "debug":
                log_obj.debug(msg[5:])
            elif msg[:4] == "info":
                log_obj.info(msg[4:])
            elif msg[:7] == "warning":
                log_obj.warning(msg[7:])
            elif msg[:5] == "error":
                log_obj.error(msg[5:])
            # print(logger.log_queue.qsize())
        time.sleep(0.001)
        #print("log_dtime", round(time.time() - this_time, 4))

t1 = threading.Thread(target=foo)
t1.start()

if __name__ == '__main__':
    while True:
        logger.debug('测试')
        # log.logger.info('普通信息')
        # log.logger.warning('警告')
        # log.logger.error('报错')
        # log.logger.critical('严重')
        time.sleep(1)
    # Logger('error.log', level='error').logger.error('error')