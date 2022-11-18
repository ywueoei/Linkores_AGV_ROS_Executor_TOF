import asyncio
import threading
import math
import time
import os
import json
import copy
import config
from log import logger
import locator

# from uart_screen import screen_ctrl
version = "CN_V2.22.0630.1"
print("版本号{}".format(version))


async def do_connect(loop, host, port, protocol):
    while True:
        try:
            await loop.create_connection(lambda: protocol, host, port)
        except OSError as e:
            print("scheduler connect retrying in 3 seconds") #连接未成功，3秒后再重连
            await asyncio.sleep(5)
        else:
            break


class Scheduler_Connect(asyncio.Protocol):
    def __init__(self, loop, mac):
        self.loop = loop
        self.mac = mac
        self.state = False
        self.targetForkHeight = None
        self.task_flag = 0
        self.stop_mode = 0
        self.charge_status = 0
        self.power = 100
        self.FserialNumber = 0
        self.R_status = 0
        self.p2context = ""
        self.FisBlock = 9
        self.safemode = 0
        self.task_finish = 0
        self.fist = 0
        self.firstDotId = None
        self.resetState = 0
        self.rotateToRadians=None
        self.typeStr = None

        self._databuf = ''
        self.ControlVal = list()
        self.LastPath = [0.0, 0.0, 0.0, 0.0]
        self.parsePath = ""
        self.Flag = 0

        self.p1stamp = 0
        self.pathPart = -1
        self.total_point_list = []
        self.total_point_obj_list = []
        self.current_point_obj_list = []
        self._lock = threading.Lock()
        self.adjustDistance = 0
        self.stamp_list = []
        self.heart_beat = 0
        self.rotate_statue = False
        self.target_angle = 0
        self.sequenceId = None

    # 连接server后执行
    def connection_made(self, transport):
        self._transport = transport
        self.host, self.port = transport.get_extra_info('peername')
        print('Scheduler Service Connected.')

    # 接收server返回的数据
    def data_received(self, data):
        self.state = True
        # databuf用来处理多条数据粘连的消息
        # 之前的缓存+当前的数据
        data = self._databuf + data.decode('utf-8')

        # 分割数据
        while (data.__contains__('\n')): # '\n'是否包含在data中，如果在的活，默认收到了一条服务器消息
            process = data[0 : data.index('\n') + 1] # 将头一条消息取出来
            data = data[data.index('\n') + 1 : data.__len__()] # 留下剩下的消息
            self.analysis_schedulerdata(process)

        # 缓存剩下的粘连数据
        self._databuf = data

    # 数据解析
    def analysis_schedulerdata(self, jdata):
        #global DataCollection,FILE,TimeL
        pdata = json.loads(jdata)
        js_data = json.dumps(pdata)
        message = pdata.get("message")
        data = pdata.get("data")
        if message == "ARRIVE":
            return
        if message != "HEARTBEAT":
            print("服务器消息: {}".format(pdata))
        logger.info("服务器消息: {}".format(pdata))

        if message == "HEARTBEAT":
            if self.heart_beat < 9:
                self.heart_beat += 1
            else:
                self.heart_beat = 0

        elif message == "MPC_COMPENSATION":  # 货位点补偿
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac,
                                               "serialNumber": serialNumber})

            locator.schedule_offset_x = 0.0 if data.get("x") == None or data.get("x") == "" else round(data.get("x"), 4)
            locator.schedule_offset_y = 0.0 if data.get("y") == None or data.get("y") == "" else round(data.get("y"), 4)
            locator.schedule_offset_phi = 0.0 if data.get("phi") == None or data.get("phi") == "" else round(data.get("phi"), 4)
            logger.info("MPC_COMPENSATION X:{},Y:{},PHI:{}".format(locator.schedule_offset_x, locator.schedule_offset_y, locator.schedule_offset_phi))
            locator.schedule_offset_x = -0.10 if locator.schedule_offset_x < -0.10 else 0.10 if locator.schedule_offset_x > 0.10 else locator.schedule_offset_x
            locator.schedule_offset_y = -0.10 if locator.schedule_offset_y < -0.10 else 0.10 if locator.schedule_offset_y > 0.10 else locator.schedule_offset_y
            locator.schedule_offset_phi = -3 if locator.schedule_offset_phi < -3 else 3 if locator.schedule_offset_phi > 3 else locator.schedule_offset_phi


        elif message == "PARSEPATH":  # MPC算法新路径
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac, "serialNumber": serialNumber})
            self._lock.acquire()
            parsePath = data.get("parsePath")
            # print("parsePath:", parsePath)
            self.adjustDistance = data.get("adjustDistance")  # 调整路径距离
            if float(self.adjustDistance) > 0.0:
                self.adjustDistance = config.adjust_distance
            self.parsePath = parsePath +"," + str(self.adjustDistance)
            print("parsePath", self.parsePath)
            self._lock.release()

        elif message == "SPIN":   #原地旋转模式
            self.FserialNumber = data.get("serialNumber")
            self.rotateToRadians = int(data.get("toRadians") * 180 / math.pi)
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac, "serialNumber": self.FserialNumber})
            self.task_flag = 5  #旋转中

        elif message == "STOP":
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac, "serialNumber": serialNumber})
            self.stop_mode = data.get("mode")

        elif message == "HEARBEAT":
            timestamp = data.get("timestamp")

        elif message == "START":
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac,
                                               "serialNumber": serialNumber})
            self.stop_mode = 0

        elif message == "TASK":  # 主要给小车提供屏幕显示信息
            logger.info("版本号{}".format(version))
            serialNumber = data.get("serialNumber")
            # 通用应答
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac,
                                               "serialNumber": serialNumber})
            taskSubsequence = data.get("taskSubsequence")
            # self.taskId = info.get("taskId")
            self.sequenceId = taskSubsequence.get("sequenceId")
            self.startName = taskSubsequence.get("startName")
            self.endName = taskSubsequence.get("endName")
            self.typeStr = taskSubsequence.get("typeStr")
            stateStr = taskSubsequence.get("stateStr")
            progressStr = taskSubsequence.get("progressStr")
            priority = taskSubsequence.get("priority")
            message = data.get("message")
            weight = taskSubsequence.get("weight")
            if self.startName == None:
                self.startName = "-"
            if self.endName == None:
                self.endName = "-"

            self.task_finish = 0

            self._lock.acquire()
            self.total_point_list = []
            self.total_point_obj_list = []
            self.current_point_obj_list = []
            self.stamp_list = []
            self._lock.release()

        elif message == "TASK_FINISH":
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac, "serialNumber": serialNumber})
            self.task_finish = 1
            self.resetState = 1
            self.sequenceId = None
            self.typeStr = None

            self._lock.acquire()
            self.pathPart = -1
            self.total_point_list = []
            self.total_point_obj_list = []
            self.current_point_obj_list = []
            self.stamp_list = []
            self._lock.release()

        elif message == "PATH":
            from uart_screen import screen_ctrl
            self.task_flag = 1
            screen_ctrl.screenmsg_taskcont_od["调度警告"] = " "  # 清除交通管制显示
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac, "serialNumber": serialNumber})
            subsequenceNumber = data.get("subsequenceNumber")
            this_pathPart = data.get("pathPart") #任务片段号
            path = data.get("path")
            # print("path", path)
            if subsequenceNumber == self.sequenceId and this_pathPart == self.pathPart + 1:
                self.pathPart = this_pathPart
                #根据stamp排序
                self.this_point_list = sorted(path, key=lambda p: -p.get("stamp"), reverse=True)
                # 首尾相连的去掉第二段路径的第一个点
                # if len(self.total_point_list) and self.this_point_list[0].get("stamp") == self.total_point_list[-1].get("stamp"):
                #     self.this_point_list = self.this_point_list[1:]
                #     print("PATH-----------------------------------------------")

                # # 非首尾相连的直接拼接
                #     #pass

                # 拼接
                self.total_point_list += self.this_point_list
                logger.info("所有路径：{}".format(self.total_point_list))
                self._lock.acquire()
                for point in self.this_point_list:
                    # 通过stamp_list去除重复的点
                    if point.get('stamp') not in self.stamp_list:
                        self.stamp_list.append(point.get('stamp'))
                        p = Point(point.get('x'), point.get('y'), point.get('angle'), \
                                subsequenceNumber, point.get('stamp'), \
                                point.get('dotId'), point.get('nodeNum'), \
                                point.get('dotNum'), \
                                point.get('speed'))
                        # self.total_point_obj_list.append(p)
                        self.current_point_obj_list.append(p)
                self._lock.release()

                # print("所有点：", self.total_point_obj_list)
                # print("剩余点：", self.current_point_obj_list)
            else:
                print("log, error of subsequenceNumber or pathPart")
                logger.error("error of subsequenceNumber or pathPart")
                pass

        elif message == "CHARGE":
            self.FserialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac, "serialNumber": self.FserialNumber})
            self.charge_status = data.get("status")

            if self.charge_status == 1:
                self.task_flag = 3

            self.power = data.get("power")

        elif message == "FORK_HEIGHT":
            from uart_screen import screen_ctrl
            self.FserialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac, "serialNumber": self.FserialNumber})
            height = data.get("height")
            self.FisBlock = data.get("isBlock")

            self.targetForkHeight = round(height, 3)
            screen_ctrl.screenmsg_taskcont_od["货叉控制"] = "[货叉高度--> {}]".format(height, '.3f')
            self.task_flag = 2
            if self.FisBlock == 0:
                self.senddata_to_scheduler("FINISH", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac, "sequenceId": self.sequenceId, "serialNumber": self.FserialNumber, "stamp": self.p1stamp, "status": 4})
                self.FisBlock = 9
                screen_ctrl.screenmsg_taskcont_od["货叉控制"] = "[货叉控制 - ]"



        elif message == "SYNC_TIME":
            settime = data.get("timestamp")
            strtime = time.strftime("sudo date -s '%Y-%m-%d %H:%M:%S'", time.localtime(settime))
            # print("SYNC_TIME", settime, strtime)
            os.system(strtime) #设置linux系统时间，与服务器一致,  仿真时不需要
            car_parod = {"timestamp": str(round(time.time() * 1000)), "name": config.cfg_car_Num, "mac": self.mac,
                         "security": "linkores", "mapId": config.cfg_Map_ID, "edgeOffset": config.cfg_nav350_edgeoffset,
                         "loadOffset": config.cfg_nav350_goodoffset, "typeRegister": 0}

            self.senddata_to_scheduler("REGISTER", car_parod)

        elif message == "REGISTER":
            self.R_status = data.get("status")
            message = data.get("message")
            # print("REGISTER ACK message:", self.R_status, message)

        elif message == "RESET":
            self.sequenceId = None
            self.typeStr = None
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac,
                                               "serialNumber": serialNumber})
            self.resetState = 1

            self._lock.acquire()
            self.rotate_statue = False
            self.targetForkHeight = None
            self.adjustDistance = 0  # 调整路径距离
            self.charge_status = 0
            self.p2context = ""
            self.pathPart = -1
            self.total_point_list = []
            self.total_point_obj_list = []
            self.current_point_obj_list = []
            self.stamp_list = []
            self._lock.release()

        elif message == "LOAD_VERIFICATION":  # 货物开关模式
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac,
                                               "serialNumber": serialNumber})
            self.p2context = data.get("mode")
            print("货叉校验：", self.p2context)
            # self.senddata_to_scheduler("FINISH", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac,
            #                                       "sequenceId": self.sequenceId, "serialNumber": serialNumber,
            #                                       "stamp": self.p1stamp, "status": 4})

        elif message == "SECURITY_CONTROL":
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac,
                                               "serialNumber": serialNumber})
            # self.senddata_to_scheduler("FINISH", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac,
            #                                     "sequenceId": self.sequenceId, "serialNumber": serialNumber,
            #                                     "stamp": self.p1stamp, "status": 4})
            # self.task_flag = 4
            logger.info("雷达模式切换, 当前模式：{}，目标模式：{}".format(self.safemode, data.get("mode")))
            self.safemode = data.get("mode")
            isBlock = data.get("isBlock")
            # print("SECURITY_CONTROL data:", self.safemode, isBlock)

        elif message == "SCREEN_CONTROL":
            from uart_screen import screen_ctrl
            # 目前发现只有“交通管制”用到
            serialNumber = data.get("serialNumber")
            self.senddata_to_scheduler("ACK", {"timestamp": str(round(time.time() * 1000)), "mac": self.mac,
                                               "serialNumber": serialNumber})
            content = data.get("content")
            screen_ctrl.screenmsg_taskcont_od["调度警告"] = "[" + content + "]"
            print("content", content)

    def get_three_point(self):
        p1, p2, p3 = None, None, None
        self._lock.acquire()
        if len(self.current_point_obj_list) >= 3:
            p1 = self.current_point_obj_list[0]
            p2 = self.current_point_obj_list[1]
            p3 = self.current_point_obj_list[2]
        elif len(self.current_point_obj_list) >= 2:
            p1 = self.current_point_obj_list[0]
            p2 = self.current_point_obj_list[1]
        elif len(self.current_point_obj_list) >= 1:
            p1 = self.current_point_obj_list[0]
        self._lock.release()
        return p1, p2, p3, self.pathPart

    def del_point(self):
        logger.info("去掉一个点")
        self._lock.acquire()
        if len(self.current_point_obj_list) >= 2:
            del(self.current_point_obj_list[0])
        self._lock.release()

    def get_current_point_obj_list(self):
        self._lock.acquire()
        temp_list = copy.deepcopy(self.current_point_obj_list)
        self._lock.release()
        return temp_list

    def get_parsePath(self):
        self._lock.acquire()
        adjustDistance = self.adjustDistance
        parsePath = self.parsePath
        self._lock.release()
        return adjustDistance, parsePath

    def senddata_to_scheduler(self, pmessage, pdata):
        # STATE, ACK, FINISH, ARRIVE, ARRIVE_NEW

        send = {'message': pmessage, 'data': pdata}
        jsend = json.dumps(send)
        self.data_send(jsend)
        if pmessage == "FINISH":
            logger.info("发送FINISH消息：{}".format(send))
        # print("send_to_scheduler", jsend)

    def data_send(self, data):
        self._transport.write((data + "\n").encode())

    # 与server断开连接后执行
    def connection_lost(self, exc):
        print('disconnected')
        # asyncio.async
        asyncio.ensure_future(do_connect(self.loop, self.host, self.port, self))

class Point():
    def __init__(self, _x, _y, _phi, _sequenceId, _stamp, _dotId, _nodeNum, _dotNum, _speed) -> None:
        self.sequenceId = _sequenceId
        self.stamp = _stamp
        self.dotId = _dotId
        self.nodeNum = _nodeNum
        self.dotNum = _dotNum
        self.x = _x
        self.y = _y
        self.phi = _phi
        self.speed = _speed

    def __str__(self):
        return "[sequenceId: {}, stamp: {}, dotId: {}, x: {}, y: {}, phi: {}, speed: {}]".format(self.sequenceId, self.stamp, self.dotId, self.x, self.y, self.phi, self.speed)

ip = config.scheduler_ip
port = 10011
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
mac = config.mac
scheduler_protocol = Scheduler_Connect(loop=loop, mac=mac)
def foo1_thread():
    loop.run_until_complete(do_connect(loop, ip, port, scheduler_protocol))
    loop.run_forever()
    loop.close()

test_thread1 = threading.Thread(target=foo1_thread)
test_thread1.start()


if __name__ == "__main__":
    x = 9.568
    y = 1.408
    phi = -math.pi
    forkHeight = 0
    speed = 0
    Battery_SOC = 100

    hw = 1
    park_state = 0 # 0是停车，1是运行状态
    mode = 1  # 0 手动 or 1 自动模式
    safety_mode = 0
    task_flag = 9  # 无任务
    realtime_dataod = {"timestamp": str(round(time.time()*1000)), "mac": mac, "mapId": config.cfg_Map_ID, \
                        "x": x , "y": y, "angle":phi, "height": forkHeight,\
                        "power": Battery_SOC, "loading": hw, "parking": park_state, \
                        "speed":speed, "mode": mode, \
                        "safety":safety_mode, "status":200, "workStatus": task_flag \
                        }
    # od = {"timestamp": str(round(time.time()*1000)), "mac": sMAC, "sequenceId": p2.sequenceId, "dotId": p2.dotId, "nodeNum": p2.nodeNum, "dotNum": p2.dotNum, "stamp": p2.stamp, "pathPart": commander.pathPart}

    def foo2_thread():
        while True:
            while scheduler_protocol.state:
                scheduler_protocol.senddata_to_scheduler("STATE", realtime_dataod)
                print("123")
                time.sleep(0.5)
            time.sleep(1)

    # def foo3_thread():
    #     while True:
    #         while scheduler_protocol.state:
    #             print(scheduler_protocol.getValue())
    #             time.sleep(0.125)
    #         time.sleep(1)

    test_thread2 = threading.Thread(target=foo2_thread)
    # test_thread3 = threading.Thread(target=foo3_thread)
    test_thread2.start()
    # test_thread3.start()      

    # while True:
    #     while len(scheduler_protocol.total_point_list):
    #         scheduler_protocol.total_point_list[0]

    #     time.sleep(1)