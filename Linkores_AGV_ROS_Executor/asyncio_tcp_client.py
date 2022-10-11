import asyncio
import queue
import traceback
# from utils.log.log import logger

class EchoClientProtocol(asyncio.Protocol):
    def __init__(self, host, port, name):
        asyncio.Protocol.__init__(self)
        # loop = asyncio.get_event_loop() # < 3.7
        self.on_con_lost = None
        self.name = name
        self.host, self.port = host, port
        self.connection_state = False
        self.data_queue = queue.Queue(3)

    # 连接server后执行
    def connection_made(self, transport):
        print('{}, connected'.format(self.name))
        # logger.info('{}, connected'.format(self.name))
        self.host, self.port = transport.get_extra_info('peername')
        self._transport = transport
        self.connection_state = True

    # 接收server返回的数据
    def data_received(self, data):
        try:
            self.data_queue.put_nowait(data)
        except queue.Full:
            print("{}-队列已满".format(self.name))
            # logger.warning("{}-队列已满".format(self.name))
        except:
            # logger.error(traceback.format_exc())
            print(traceback.format_exc())

    async def analysis_data(self):
        pass
        # while True:
        #     if not self.data_queue.empty():
        #         print(self.data_queue.get())
        #     await asyncio.sleep(0.05)

    async def data_send_cyclical(self):
        pass


    # 与server断开连接后执行
    def connection_lost(self, exc):
        self.connection_state = False
        # logger.warning('{}, disconnected'.format(self.name))
        print('{}, disconnected'.format(self.name))
        self.on_con_lost.set_result(True)

    def data_send(self, data):
        # print(11)
        if self.connection_state:
            self._transport.write(data)

    async def tcp_connect(self):
        # loop = asyncio.get_running_loop() # >= 3.7版本python
        loop = asyncio.get_event_loop() # < 3.7
        while True:
            try:
                self.on_con_lost = loop.create_future()
                await loop.create_connection(lambda: self, self.host, self.port)
                if await self.on_con_lost: # 获取协程结束返回的状态
                    print("{} retrying reconnect after 3 seconds".format(self.name)) #连接未成功，3秒后再重连
                    # logger.error("{} retrying reconnect after 3 seconds".format(self.name)) #连接未成功，3秒后再重连
                    continue
            except OSError:
                print("{} retrying reconnect after 3 seconds".format(self.name)) #连接未成功，3秒后再重连
                # logger.error("{} retrying reconnect after 3 seconds".format(self.name)) #连接未成功，3秒后再重连
            except:
                print(traceback.format_exc())
                # logger.error(traceback.format_exc())

            await asyncio.sleep(3)