import asyncio
from thread_template import ThreadTemplate

class Coroutines(ThreadTemplate):
    def __init__(self, name=None):
        super().__init__(name)
        self.__loop = asyncio.new_event_loop()

    def add_coroutines(self, coroutines):
        asyncio.run_coroutine_threadsafe(coroutines, self.__loop)
    
    def loop(self):
        asyncio.set_event_loop(self.__loop)
        self.__loop.run_forever()