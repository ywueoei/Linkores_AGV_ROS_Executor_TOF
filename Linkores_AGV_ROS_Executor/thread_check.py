
import threading
import time

class Thread_check():
    def __init__(self) -> None:
        self.thread_dict = dict()
        self._lock = threading.Lock()
        
    def add_thread(self, t_name):
        self._lock.acquire()
        self.thread_dict[t_name] = [0, time.strftime('%H:%M:%S')]
        self._lock.release()
    
    def thread_update(self, t_name):
        count = self.thread_dict[t_name][0] + 1
        if count >= 1000:
            count = 0
        self.thread_dict[t_name] = [count, time.strftime('%H:%M:%S')]

t_check = Thread_check()