import threading

# from frequency import frequency_decorator
# from frequency_monitor import frequency_monitor_decorator
# from utils.thread.read_thread_id import get_tid
# from utils.thread.thread_check import ThreadCheck

class ThreadTemplate(threading.Thread):
    def __init__(self, thread_name=None):
        # threading.Thread.__init__(self)
        super().__init__()
        if thread_name is None:
            thread_name = self.__class__.__name__
        self.thread_name = thread_name

    # @frequency_decorator(1000)          # 当循环频率能大于N时才能做到
    # @frequency_monitor_decorator(1000)  # 检测执行频率是否在N以内
    def loop(self):
        i = 0
        while i < 10000:
            i += 1

    def run(self):
        # self.name = self.thread_name + "_" + str(get_tid())
        # self.t_check = ThreadCheck(self.name)
        while True:
            # self.t_check.thread_check_update()
            self.loop()

# a = MyThread()