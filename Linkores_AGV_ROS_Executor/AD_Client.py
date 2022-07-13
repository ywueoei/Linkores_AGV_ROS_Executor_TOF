import asyncio
import threading
import time
import config


async def do_connect(loop, host, port, protocol):
    while True:
        try:
            await loop.create_connection(lambda: protocol, host, port)
        except OSError as e:
            print("AD_client retrying connect in 3 seconds") #连接未成功，3秒后再重连
            await asyncio.sleep(1)
        else:
            break

class AD_Client(asyncio.Protocol):
    def __init__(self, loop):
        self.loop = loop
        self.connect_state = False
        self.ad_value = 0
        self.command = bytearray([0x00, 0x03, 0x00, 0x00, 0x00, 0x06, 0x01, 0x04, 0x00, 0x00, 0x00, 0x01])
        self.send_time = None
        self.recv_time = None

    # 连接server后执行
    def connection_made(self, transport):
        self.connect_state = True
        self._transport = transport
        self.host, self.port = transport.get_extra_info('peername')
        print('AD Service Connected')
        self.recv_time = time.time()
        self.send_time = time.time()

    # 接收server返回的数据
    def data_received(self, data):
        self.recv_time = time.time()
        if len(data) != 11:
            self.ad_value = 0
        else:
            self.ad_value = int.from_bytes(data[9:11], 'big') # * 0.001
        
        # self.data_send()
        # time.sleep(0.2)

    # 与server断开连接后执行
    def connection_lost(self, exc):
        print('AD Service Disconnected')
        self.connect_state = False
        # asyncio.async(do_connect(self.loop, self.host, self.port, self))
        asyncio.ensure_future(do_connect(self.loop, self.host, self.port, self))
    
    def get_ad_value(self):
        # print(self.connect_state, self.send_time, self.recv_time)
        if (not self.connect_state) or (self.send_time == None or self.recv_time == None) or abs(self.send_time - self.recv_time) > 1: # 一般来说send_time 大于 recv_time
            return 0
        return self.ad_value
    
    def send_data(self):
        if self.connect_state:
            self._transport.write(self.command)
            self.send_time = time.time()


# ip = "192.168.1.33"
ip = config.ad_modular_ip
port = 502
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
ad_client = AD_Client(loop=loop)

def foo1_thread():
    loop.run_until_complete(do_connect(loop, ip, port, ad_client))
    loop.run_forever()
    loop.close()

def foo2_thread():
    while True:
        ad_client.send_data()
        time.sleep(0.1)

# def foo2_thread():
#     while True:
#         while ad_client.connect_state:
#             if ad_client.get_ad_value() != None:
#                 print(ad_client.ad_value)
#             time.sleep(0.2)
#         time.sleep(1)

test_thread1 = threading.Thread(target=foo1_thread)
test_thread2 = threading.Thread(target=foo2_thread)
test_thread1.start()
test_thread2.start()