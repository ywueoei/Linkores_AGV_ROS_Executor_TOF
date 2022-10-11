import time
import threading
# from utils.log.log import logger

# # 实例化的时候传入频率，代表的是最大的频率
# class Frequency:
#     def __init__(self, frequency) -> None:
#         self._frequency = 1 / frequency
#         self.pre_time = time.time()

#     def delay(self):
#         this_time = time.time()
#         time_interval = this_time - self.pre_time
#         sleep_time = self._frequency - time_interval
#         if sleep_time > 0:
#             time.sleep(sleep_time)

#         self.pre_time = time.time()


# def frequency_decorator(_frequency):
#     frequency = 1 / _frequency

#     def foo1(func):
#         def foo2(*args, **kwargs):
#             pre_time = time.time()
#             # print(A)
#             res = func(*args, **kwargs)
#             # print(A)
#             this_time = time.time()
#             time_interval = this_time - pre_time
#             sleep_time = frequency - time_interval
#             if sleep_time > 0:
#                 time.sleep(sleep_time)
#             return res

#         return foo2

#     return foo1

class FrequencyMonitorDecorator:
    def __init__(self, frequency) -> None:
        self.frequency = frequency
        self.pre_time = time.time()
        
    def __call__(self, func):
        def foo(*args, **kwargs):
            time_interval = time.time() - self.pre_time
            # print(time_interval)
            self.pre_time = time.time()
            if time_interval > self.frequency:
                print(
                    "time out, {} - {} - {} - {}time out".format(
                        threading.current_thread().name, func.__name__, self.frequency, time_interval
                    )
                )
            # print("a")
            res = func(*args, **kwargs)
            # print("b")
            return res
        return foo

class FrequencyDecoratorAsync():
    def __init__(self, frequency) -> None:
        self.frequency = frequency
        self.pre_time = time.time()

    def __call__(self, func):
        def foo(*args, **kwargs):
            # print("A")
            res = func(*args, **kwargs)
            # print("B")
            time_interval = time.time() - self.pre_time
            # print("时间间隔：", time_interval)
            temp = (time_interval - self.frequency) # 根据正负值来判断每次循环的时间间隔是不是在规定时间范围内
            if temp < 0:
                temp = 0
            sleep_time = self.frequency - temp  # 时间延时要减去超过的部分
            # print("延时:", sleep_time)
            # print(time.time())
            if sleep_time <= 0:
                sleep_time = 0
            self.pre_time = time.time()
            return (res, sleep_time)

        return foo

class FrequencyDecorator():
    def __init__(self, frequency) -> None:
        self.frequency = frequency
        self.pre_time = time.time()

    def __call__(self, func):
        def foo(*args, **kwargs):
            # print("A")
            res = func(*args, **kwargs)
            # print("B")
            time_interval = time.time() - self.pre_time
            # print("时间间隔：", time_interval)
            temp = (time_interval - self.frequency) # 根据正负值来判断每次循环的时间间隔是不是在规定时间范围内
            if temp < 0:
                temp = 0
            sleep_time = self.frequency - temp  # 时间延时要减去超过的部分
            # print("延时:", sleep_time)
            # print(time.time())
            if sleep_time <= 0:
                sleep_time = 0
            self.pre_time = time.time()
            time.sleep(sleep_time)
            return res

        return foo
# a = Max_Frequency(20)
# while 1:
#     print(time.time())
#     a.delay()
