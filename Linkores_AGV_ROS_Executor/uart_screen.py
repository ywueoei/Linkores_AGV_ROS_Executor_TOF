# -*-coding:utf-8 -*-
import time
import serial
import os
import threading
import collections
from thread_check import t_check
from security_alarm import alarm_msglist
'''
ACTIONCODE_INIT =1
ACTIONCODE_UPDATE =0
ACTIONCODE_SETTING =2
ACTIONCODE_ANSWER =3

ACTIONCODE_SHOW=4
ACTIONCODE_HIDE=5

'''
_serial = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)


class Screen_ctrl():
    def __init__(self):
        self.ser = _serial
        self.serial_count = 0

        self.safe_level_list = []
        self.screenmsg_item_safe_list = []
        self.is_taskcont_refresh = True

        self.screenmsg_item_taskcont_list = []

        self.forkqianqing_houyang_flag = 0
        self.forkqianqing_houyang_speed = 0
        self.jiaotongguanzhi_flag = 0
        self.taskreset_flag = 0
        self.charge_flag = 0
        self.reset_flag = 0
        self.clear_rcv_count = 0
        self.cacel_task_flag = 0
        self.finish_task_flag = 0

        #self.timeout_clearflag = 0

        self.rcv_angle = 0
        self.rcv_dig = 0
        self.data = 0
        self.rcv_press_board = 0

        self.ACTIONCODE_INIT = 1
        self.ACTIONCODE_UPDATE = 0
        self.ACTIONCODE_SETTING = 2
        self.ACTIONCODE_ANSWER = 3
        self.ACTIONCODE_SHOW = 4
        self.ACTIONCODE_HIDE = 5
        self.hand_ctrl_flag = False
        
        #消息ID
        self.screenmsg_id_od = dict(
            taskcont=0x10,
            taskcont_botton_tips=0x11,
            taskcont_win2tips=0x12,
            safecont=0x18,
            infopagecont=0x28,
            infopagetitle=0x29,
            buttoncont=0x40,
            config1=0x30,
        )

        #安全区域内容字典
        self.screenmsg_safe_od = {
            # key :(safelevel ,content)
            "safecbjt": [3, "触边胶条触发~", 4001],
            "safehcz": [3, "左货叉遇障碍~", 4002],
            "safehcy": [3, "右货叉遇障碍~", 4003],
            "safeztz": [3, "左支腿遇障碍~", 4004],
            "safezty": [3, "右支腿遇障碍~", 4005],

            "safezbz1zone": [3, "右安全雷达危险区触发~", 4006],
            "safezbz2zone": [2, "右安全雷达警告2区触发~", 0],
            "safezbz3zone": [2, "右安全雷达警告1区触发~", 0],

            "safeybz1zone": [3, "左安全雷达危险区触发~", 4006],
            "safeybz2zone": [2, "左安全雷达警告2区触发~", 0],
            "safeybz3zone": [2, "左安全雷达警告1区触发~", 0],

            "safesbz1zone": [3, "3号安全雷达高危险区触发~", 4006],
            "safesbz2zone": [2, "3号安全雷达警告2区触发~", 0],
            "safesbz3zone": [2, "3号安全雷达警告3区触发~", 0],

            "safexbz1zone": [3, "4号安全雷达危险区触发~", 4006],
            "safexbz2zone": [2, "4号安全雷达警告2区触发~", 0],
            "safexbz3zone": [2, "4号安全雷达警告3区触发~", 0],

            "load_buthw0_nearend": [2, "要求载货中但检测无货NearEnd ", 4009],
            "unload_buthw1_nearend": [2, "要求无货但检测有货NearEnd ", 4010],
            "unloadend_buthw1_nearend": [2, "卸货完成但检测有货 ", 4011],
            "loadend_buthw0_nearend": [2, "装货完成但检测无货 ", 4012],

            "load_buthw0_halfway": [2, "要求有货但检测无货~", 4013],
            "unload_buthw1_halfway": [2, "要求无货但检测有货~", 4014],
            "unloadend_buthw1_halfway": [2, "取货，要求无货但有货~", 4015],
            "loadend_buthw0_halfway": [2, "退出，要求无货但有货~", 4016],
            "load_err": [2, "取货，货物突出5cm以上~", 4017],

            "xiaojiting_226can": [3, "##电机控制器连接断开，请检查急停按钮是否被按下。", 5000],
            "height_err": [3, "拉绳编码器数值异常 ", 0],
            "outofctrl": [3, "车辆偏差过大~", 5001],
            "battery_low": [3, "低电量警告，请及时充电~", 5002],
            "curtis_1232_err": [3, "行走控制器故障（故障号详见系统信息页）~", 5003],
            "curtis_1253c_err": [3, "泵控故障（故障号详见系统信息页）~", 5004],
            "nav_err": [3, "导航雷达错误", 5005],
            "scheduler_err": [3, "调度服务器错误", 5006],
            "charge_butnobms_err": [3, "无BMS,不执行充电", 5007],
            "locator_err": [3, "导航数据错误#", 5008],
            "controller_err": [3, "控制器初始化失败或回中未成功,请检查小急停或切自动等待回中完成#", 5009],
            "stop": [3, "调度系统停止请求", 0],
            "mpc_not_connected": [3, "MPC求解器未连接", 5011],
            "huochalowestpoint_err": [3, "货叉最低点测试错误", 0],
            "ctrler_can_err": [3, "主控提示CAN错误", 5012],
            "stall_err": [3, "舵轮堵转~", 5013],
            "wheel_slip": [3, "#舵轮打滑或空转,请排查故障,切手动消除#", 5014],
            "situ_timeout": [3, "#任务中,车辆停止超时,请移动车辆清除故障#", 5015],
        }

        
        #系统信息界面字典，直接改字典的value即可
        self.screenmsg_infopage_od = {}
        self.screenmsg_infopage_od = collections.OrderedDict()  # 有序字典
        self.screenmsg_infopage_od["电池:"] = "0%使用中"#位置不能变
        self.screenmsg_infopage_od["时间:"] = "0" #2019
        #self.screenmsg_infopage_od["速度(km/h):"] = "0"
        self.screenmsg_infopage_od["舵轮角度:"] = "0"  # 位置不能变，不能带符号
        self.screenmsg_infopage_od["MPC_info:"] = "--"
        self.screenmsg_infopage_od["货叉状态:"] = "0"
        self.screenmsg_infopage_od["货叉高度:"] = "0"
        self.screenmsg_infopage_od["横向偏差:"] = "0"
        self.screenmsg_infopage_od["角度偏差:"] = "0"
        self.screenmsg_infopage_od["舵轮偏差:"]="0"
        self.screenmsg_infopage_od["路段长度:"]="0"
        self.screenmsg_infopage_od["剩余长度:"]="0"
        self.screenmsg_infopage_od["任务消息:"] = "0"
        self.screenmsg_infopage_od["货叉数据:"] = "0"
        self.screenmsg_infopage_od["货叉电压:"] = "0"
        self.screenmsg_infopage_od["网口地址(ip):"] = "0"
        self.screenmsg_infopage_od["WiFi地址(ip):"] = "0"
        self.screenmsg_infopage_od["导航雷达地址(ip):"] = "0"
        self.screenmsg_infopage_od["服务器地址(ip):"] = "0"
        self.screenmsg_infopage_od["MAC地址:"] = "0"
        self.screenmsg_infopage_od["X:"] = "0"
        self.screenmsg_infopage_od["Y:"] = "0"
        self.screenmsg_infopage_od["Angle:"] = "0"
        self.screenmsg_infopage_od["心跳:"] = "0"
        self.screenmsg_infopage_od["速度(km/h):"] = "0"
        self.screenmsg_infopage_od["S_info:"] = "-"
        self.screenmsg_infopage_od["地图ID:"] = "0"
        self.screenmsg_infopage_od["Parking:"] = "-"
        self.screenmsg_infopage_od["ping:"] = "0"
        self.screenmsg_infopage_od["单体最高电压:"] = "0"
        self.screenmsg_infopage_od["行走故障:"] = "0"
        self.screenmsg_infopage_od["泵控故障:"] = "0"
        self.screenmsg_infopage_od["安全模式:"] = "0"
        self.screenmsg_infopage_od["CPU温度:"] = "0"


        self.screenmsg_taskcont = "任务初始化"
        #任务界面字典，直接改字典的value即可
        self.screenmsg_taskcont_od = {}
        self.screenmsg_taskcont_od = collections.OrderedDict()  # 有序字典
        self.screenmsg_taskcont_od["开头"] = ""
        self.screenmsg_taskcont_od["服务器"] = "服务器连接中..."
        self.screenmsg_taskcont_od["导航雷达"] = "导航雷达连接中..."
        self.screenmsg_taskcont_od["服务器消息"] = ""
        self.screenmsg_taskcont_od["调度警告"] = ""
        self.screenmsg_taskcont_od["货叉控制"] = "[货叉控制: - ]"

        # button上的内容，"-"为隐藏
        self.botton_od = {}
        self.botton_od = collections.OrderedDict()  # 有序字典

        self.botton_od["botton1"]= "-"#"完成"
        self.botton_od["botton2"]="-"#"取消"
        self.botton_od["botton3"]="-"#"确认"
        self.botton_od["botton4"]="-"#"取消"

        '''
        self.botton_od["botton1"] = "完成"
        self.botton_od["botton2"] = "取消"
        self.botton_od["botton3"] = "确认"
        self.botton_od["botton4"] = "取消"
        '''
        
        #初始化
        self.screenmsg_defaultpage_init_msg()
        self.screenmsg_safe_init_msg()
        self.screenmsg_infopage_init_msg()
        self.screen_taskpage_init_msg()
        self.screenmsg_button_init_msg()

    def bytes2hexstr(self, s):
        return "b'%s'" % ''.join('\\x%.2x' % x for x in s)

    def Int2byte4(self, input_data):
        pByte = [0, 0, 0, 0]
        for i in range(4):
            pByte[i] = (input_data >> 8 * (3 - i) & 0xff)
        return pByte

    def Short2byte2(self, input_data):
        pByte = [0, 0]
        for i in range(2):
            pByte[i] = (input_data >> 8 * (1 - i) & 0xff)
        return pByte

    # def getCheckSum(self, pData):
    #     sum = 0
    #     output = [0, 0, 0, 0]
    #     for i in range(len(pData)):
    #         sum = sum + pData[i]
    #     output = self.Int2byte4(~sum + 1)
    #     return output[3]

    def sendmsg2screen(self, msg_id, actioncode, msgbody_bytes):
        self.serial_count = (self.serial_count + 1)&0xFF
        data = bytes(
            [0xFF, 0x55] + self.Short2byte2(msg_id) + self.Short2byte2(len(msgbody_bytes) + 6) + [actioncode] + self.Int2byte4(
                int(time.time())) + [
                self.serial_count]) + msgbody_bytes
        #print("screen: check ", self.getCheckSum(data), "len body =", len(msgbody_bytes))
        data = data + bytes([self.getCheckSum(data,len(data))])
        #print("screen:",self.bytes2hexstr(data))
        self.ser.write(data)

    def screenmsg_item_safe_append(self, item):
        if item not in self.screenmsg_item_safe_list:
            self.screenmsg_item_safe_list.append(item)
            self.safe_level_list.append(self.screenmsg_safe_od[item][0])
            alarm_msglist.alarm_list_append(self.screenmsg_safe_od[item][2])
            print("ALARM 1-add", self.screenmsg_safe_od[item][2], self.screenmsg_safe_od[item][1])
            #print("screen safe_level_list append ", self.screenmsg_safe_od[item][0])

    def screenmsg_item_safe_rm(self, item):
        if item in self.screenmsg_item_safe_list:
            self.screenmsg_item_safe_list.remove(item)
            self.safe_level_list.remove(self.screenmsg_safe_od[item][0])
            alarm_msglist.alarm_list_rm(self.screenmsg_safe_od[item][2])
            print("ALARM 1-rm", self.screenmsg_safe_od[item][2], self.screenmsg_safe_od[item][1])
            #print("screen safe_level_list remove ", self.screenmsg_safe_od[item][0])


    def screenmsg_item_taskcont_append(self, item):
        if item not in self.screenmsg_item_taskcont_list:
            self.screenmsg_item_taskcont_list.append(item)


    def screenmsg_item_taskcont_rm(self, item):
        if item in self.screenmsg_item_taskcont_list:
            self.screenmsg_item_taskcont_list.remove(item)

    def screenmsg_safe_init_msg(self):
        screenmsg_safe_body = "安全状态:初始化..."
        self.safe_level_list = []
        action_code = 1
        body = screenmsg_safe_body
        # print(body)
        ebody = bytes([0]) + body.encode("utf-8") + bytes([0])
        self.sendmsg2screen(self.screenmsg_id_od['safecont'], action_code, ebody)
        time.sleep(0.01)

    def screenmsg_infopage_init_msg(self):
        screenmsg_infotitle_body = ""
        screenmsg_infotitle_body_list = list(self.screenmsg_infopage_od.keys())
        for i in range(len(screenmsg_infotitle_body_list)):
            screenmsg_infotitle_body = screenmsg_infotitle_body + screenmsg_infotitle_body_list[i] + "|"
        if len(screenmsg_infotitle_body):
            action_code = 1
            body = screenmsg_infotitle_body
            # print(body)
            ebody = body.encode("utf-8") + bytes([0])
            self.sendmsg2screen(self.screenmsg_id_od['infopagetitle'], action_code, ebody)
        time.sleep(0.01)
        screenmsg_infodata_body = ""
        screenmsg_infodata_body_list = list(self.screenmsg_infopage_od.values())
        for i in range(len(screenmsg_infodata_body_list)):
            screenmsg_infodata_body = screenmsg_infodata_body + screenmsg_infodata_body_list[i] + "|"
        if len(screenmsg_infodata_body):
            action_code = 1
            body = screenmsg_infodata_body
            # print(body)
            ebody = body.encode("utf-8") + bytes([0])
            self.sendmsg2screen(self.screenmsg_id_od['infopagecont'], action_code, ebody)
        time.sleep(0.01)

    def screen_taskpage_init_msg(self):
        self.screenmsg_taskcont = "欢迎使用！AGV系统初始化，请稍后..."
        action_code = 1
        body = self.screenmsg_taskcont
        # print(body)
        ebody = body.encode("utf-8") + bytes([0])
        self.sendmsg2screen(self.screenmsg_id_od['taskcont'], action_code, ebody)
        time.sleep(0.01)
        
        #"-"为隐藏该内容
        self.screenmsg_taskcont = "-"#"手动搬运完成，请按完成。放弃任务请按取消。"
        action_code = 1
        body = self.screenmsg_taskcont
        # print(body)
        ebody = body.encode("utf-8") + bytes([0])
        self.sendmsg2screen(self.screenmsg_id_od['taskcont_botton_tips'], action_code, ebody)
        time.sleep(0.01)

        self.screenmsg_taskcont = "确认已手动完成任务？"
        action_code = 0x11
        body = self.screenmsg_taskcont
        # print(body)
        ebody = body.encode("utf-8") + bytes([0])
        self.sendmsg2screen(self.screenmsg_id_od['taskcont_win2tips'], action_code, ebody)
        time.sleep(0.01)

        self.screenmsg_taskcont = "确认取消任务？"
        action_code = 0x21
        body = self.screenmsg_taskcont
        # print(body)
        ebody = body.encode("utf-8") + bytes([0])
        self.sendmsg2screen(self.screenmsg_id_od['taskcont_win2tips'], action_code, ebody)
        self.screenmsg_taskcont = ""
        time.sleep(0.01)

    def screenmsg_button_init_msg(self):
        screenmsg_button_body = ""
        screenmsg_button_body_list = list(self.botton_od.values())
        for i in range(len(screenmsg_button_body_list)):
            screenmsg_button_body = screenmsg_button_body + screenmsg_button_body_list[i] + "|"
        if len(screenmsg_button_body):
            action_code = 1
            body = screenmsg_button_body
            # print(body)
            ebody = body.encode("utf-8") + bytes([0])
            self.sendmsg2screen(self.screenmsg_id_od['buttoncont'], action_code, ebody)
        time.sleep(0.01)

    def screenmsg_defaultpage_init_msg(self):
        screenmsg_defaultpage_body = "0"
        action_code = 1
        body = screenmsg_defaultpage_body
        # print(body)
        ebody = body.encode("utf-8") + bytes([0])
        self.sendmsg2screen(self.screenmsg_id_od['config1'], action_code, ebody)
        time.sleep(0.01)

    def getCheckSum(self, pData, len):
        sum = 0
        for i in range(len):
            sum = sum + pData[i]
        #s = sum(pData)
        return (~sum + 1) & 0xFF

    def parseProtocol(self, bytes_data):
        pData = list(bytes_data)
        pData = pData[0:len(pData) - 1]  # 去掉最后的\n
        totallen = len(pData)
        remainLen = len(pData)  # 剩余数据长度
        dataLen = 0  # 数据包长度
        frameLen = 0  # 帧长度
        DATA_PACKAGE_MIN_LEN = 7
        CMD_HEAD1 = 0xFF
        CMD_HEAD2 = 0x55
        # 以下部分需要根据协议格式进行相应的修改，解析出每一帧的数据
        while (remainLen >= DATA_PACKAGE_MIN_LEN):
            # 找到一帧数据的数据头
            while ((remainLen >= 2) and ((pData[0] != CMD_HEAD1) or (pData[1] != CMD_HEAD2))):
                pData = pData[1:len(pData)]
                remainLen = remainLen - 1
                continue

            if (remainLen < DATA_PACKAGE_MIN_LEN):
                break

            dataLen = pData[4] << 8 | pData[5]
            frameLen = dataLen + DATA_PACKAGE_MIN_LEN
            if (frameLen > remainLen):
                # 数据内容不全
                break
            # for i in range(frameLen):
            # print("%x " % (pData[i]))
            if (self.getCheckSum(pData, frameLen - 1) == pData[frameLen - 1]):
                # 解析一帧数据
                # procParse(pData, frameLen)
                # print("ok")
                self.procParse(pData)
            else:
                # print("CheckSum error!!!!!!\n")
                pass

            pData = pData[frameLen:len(pData)]
            remainLen = remainLen - frameLen

        return totallen - remainLen

    def procParse(self, data_list):
        if (data_list[2] << 8) | data_list[3] == 0x0200:
            # print("200:",data_list[6:8])
            self.forkqianqing_houyang_flag=data_list[6]
            self.forkqianqing_houyang_speed = data_list[7]
            self.clear_rcv_count = 1
        if (data_list[2] << 8) | data_list[3] == 0x0300:
            # print("300:",data_list[6:10])
            #if(data_list[6])
            if data_list[6]==self.ACTIONCODE_SHOW:
                self.hand_ctrl_flag = True
            elif data_list[6]==self.ACTIONCODE_HIDE:
                self.hand_ctrl_flag = False
            else:
                BYTE2char =lambda x: x - ((x>> 7) << 8)
                self.rcv_angle=BYTE2char(data_list[7])*10
                self.rcv_dig = data_list[8]
                self.rcv_press_board = data_list[9]
                self.clear_rcv_count = 2
                #self.timeout_clearflag=200
        if (data_list[2] << 8) | data_list[3] == 0x0400:
            # print("400:",data_list[6])
            self.jiaotongguanzhi_flag=data_list[6]
            self.clear_rcv_count = 1
        if (data_list[2] << 8) | data_list[3] == 0x0500 or (data_list[2] << 8) | data_list[3] == 0x30:
            # print("500:",data_list[6])
            self.taskreset_flag=data_list[6]
            self.clear_rcv_count = 1
        if (data_list[2] << 8) | data_list[3] == 0x0600:
            # print("600:",data_list[6])
            self.cacel_task_flag=data_list[6]
            self.clear_rcv_count = 1
        if (data_list[2] << 8) | data_list[3] == 0x0700:
            # print("700:",data_list[6])
            self.finish_task_flag=data_list[6]
            self.clear_rcv_count = 1
        if (data_list[2] << 8) | data_list[3] == 0x10:
            # print("10:",data_list[6])
            self.charge_flag=data_list[6]
            self.clear_rcv_count = 1
        if (data_list[2] << 8) | data_list[3] == 0x20:
            # print("20:",data_list[6])
            self.reset_flag=data_list[6]
            self.clear_rcv_count = 1
        if (data_list[2] << 8) | data_list[3] == 0x40:
            # print("40:",data_list[6:])
            #self.config8_flag=data_list[6]
            self.clear_rcv_count = 1

screen_ctrl = Screen_ctrl()

#使用方法
#screen_ctrl.screenmsg_item_safe_append("safecbjt")
#screen_ctrl.screenmsg_item_safe_append("load_buthw0")
screen_ctrl.screenmsg_taskcont = "AGV工作中..."
def display_thread():
    this_thread_name = "display_thread"
    t_check.add_thread(this_thread_name)
    while True:
        t_check.thread_update(this_thread_name)
        # safe
        screenmsg_safe_body = ""
        for i in range(len(screen_ctrl.screenmsg_item_safe_list)):
            screenmsg_safe_body = screenmsg_safe_body + screen_ctrl.screenmsg_safe_od[screen_ctrl.screenmsg_item_safe_list[i]][1]
        if len(screenmsg_safe_body):
            action_code = 0
            body = screenmsg_safe_body
            # print("screen safe_level_list", screen_ctrl.safe_level_list)
            safe_level = max(screen_ctrl.safe_level_list) if len(screen_ctrl.safe_level_list) else 0
            ebody = bytes([safe_level]) + body.encode("utf-8") + bytes([0])
            screen_ctrl.sendmsg2screen(screen_ctrl.screenmsg_id_od['safecont'], action_code, ebody)
        else:
            action_code = 0
            body = "安全状态:正常"
            ebody = bytes([0]) + body.encode("utf-8") + bytes([0])
            screen_ctrl.sendmsg2screen(screen_ctrl.screenmsg_id_od['safecont'], action_code, ebody)
        time.sleep(0.3)
        # infopage
        # screen_ctrl.screenmsg_infopage_od["电池:"] = str(screen_ctrl.serial_count) + "%放电"
        screenmsg_infodata_body = ""
        screenmsg_infodata_body_list = list(screen_ctrl.screenmsg_infopage_od.values())
        for i in range(len(screenmsg_infodata_body_list)):
            screenmsg_infodata_body = screenmsg_infodata_body + screenmsg_infodata_body_list[i] + "|"
        if len(screenmsg_infodata_body):
            action_code = 0
            body = screenmsg_infodata_body
            ebody = body.encode("utf-8") + bytes([0])
            screen_ctrl.sendmsg2screen(screen_ctrl.screenmsg_id_od['infopagecont'], action_code, ebody)
        time.sleep(0.3)
        # task
        screen_ctrl.screenmsg_taskcont =  screen_ctrl.screenmsg_taskcont_od["开头"] + \
                                          screen_ctrl.screenmsg_taskcont_od["服务器"] + \
                                          screen_ctrl.screenmsg_taskcont_od["导航雷达"] + \
                                          screen_ctrl.screenmsg_taskcont_od["服务器消息"] + \
                                          screen_ctrl.screenmsg_taskcont_od["货叉控制"] + \
                                          screen_ctrl.screenmsg_taskcont_od["调度警告"]


        if len(screen_ctrl.screenmsg_taskcont) and screen_ctrl.is_taskcont_refresh:
            action_code = 0
            body = screen_ctrl.screenmsg_taskcont
            ebody = body.encode("utf-8") + bytes([0])
            screen_ctrl.sendmsg2screen(screen_ctrl.screenmsg_id_od['taskcont'], action_code, ebody)
        time.sleep(0.3)

def display_rcv_thread():
    forkqianqing_houyang_flag_tmp = 0
    this_thread_name = "display_rcv_thread"
    t_check.add_thread(this_thread_name)
    while True:
        t_check.thread_update(this_thread_name)
        time.sleep(0.3)
        res = _serial.inWaiting()
        if res > 0:
            _data = _serial.readline() #todo 返回可能为none
            screen_ctrl.data = _data
            # print("screen_rcv:", _data)
            screen_ctrl.parseProtocol(_data)
        #每次只操作一小会儿
        if screen_ctrl.clear_rcv_count:
            screen_ctrl.clear_rcv_count = screen_ctrl.clear_rcv_count-1
        else:
            screen_ctrl.forkqianqing_houyang_flag = 0
            screen_ctrl.forkqianqing_houyang_speed = 0
            screen_ctrl.jiaotongguanzhi_flag = 0
            screen_ctrl.taskreset_flag = 0
            screen_ctrl.charge_flag = 0
            screen_ctrl.reset_flag = 0
            screen_ctrl.rcv_dig = 0
            screen_ctrl.cacel_task_flag = 0
            screen_ctrl.finish_task_flag = 0
            screen_ctrl.data = 0
            screen_ctrl.rcv_press_board = 0

        # if screen_ctrl.timeout_clearflag:
        #     screen_ctrl.timeout_clearflag= screen_ctrl.timeout_clearflag-1
        # else:
        #     screen_ctrl.hand_ctrl_flag = False

screen_thread_send = threading.Thread(target=display_thread) #树莓派发送给屏幕信息的线程

screen_thread_rcv = threading.Thread(target=display_rcv_thread) #树莓派接受来自屏幕信息的线程

screen_thread_send.start()
screen_thread_rcv.start()


# #修改信息界面数据，一直在发送的
# screen_ctrl.screenmsg_infopage_od["LOCALIP0:"] = "192.168.**"

# #任务界面字典，
# screen_ctrl.screenmsg_taskcont_od["开头"] = ""
# screen_ctrl.screenmsg_taskcont_od["服务器"] = "服务器连接中..."
# screen_ctrl.screenmsg_taskcont_od["导航雷达"] = "导航雷达连接中..."
# screen_ctrl.screenmsg_taskcont_od["服务器消息"] = "##...##"

# #警报界面字典
# screen_ctrl.screenmsg_item_safe_append("charge_butnobms_err")
# screen_ctrl.screenmsg_item_safe_rm("charge_butnobms_err")

# #接收信息
# rcv_angle #舵轮角度
# rcv_press_board #压板上/下
# rcv_dig #0x02-前进，0x04-后退，0x10-货叉上升，0x20-下降
# rcv_press_board #1-压板上升 2-压板下降

