# -*-coding:utf-8 -*-
import os
os.system("cp /home/pi/config.yaml /home/pi/Linkores_AGV_ROS_Executor/")  # 注意文件路径

import time
import threading
import math
import subprocess
import config
import socket
import re
import copy
import AudioPlayer
import global_var
from can_bus import can0 
from log import logger
from locator import locator_protocol
from scheduler_connect import scheduler_protocol
from AD_Client import ad_client
import action_calculation as calc
from io_controller import IO_controller
from uart_screen import screen_ctrl
from fork_controller import fork_control
from thread_check import t_check
import ps2_handle
from tof_camera import tof_camera_data

def get_ip_list():
    ip_str = os.popen("hostname -I").read()
    ip_list = ip_str.split(' ')
    return ip_list


logger.info("*********************************系统启动**********************************************")
ip_list = get_ip_list()
ip_list_len = len(ip_list)
if ip_list_len > 2:
    screen_ctrl.screenmsg_infopage_od["网口地址(ip):"] = ip_list[0]
    screen_ctrl.screenmsg_infopage_od["WiFi地址(ip):"] = ip_list[1]
elif ip_list_len > 1:
    screen_ctrl.screenmsg_infopage_od["网口地址(ip):"] = ip_list[0]
else:
    exit()

screen_ctrl.screenmsg_infopage_od["导航雷达地址(ip):"] = config.locator_ip
screen_ctrl.screenmsg_infopage_od["服务器地址(ip):"] = config.scheduler_ip
screen_ctrl.screenmsg_infopage_od["MAC地址:"] = config.mac
screen_ctrl.screenmsg_infopage_od["地图ID:"] = format(config.cfg_Map_ID)  # 转换成str

controller_status = 0
# [0, None], dist[0]: 0是还未接收到，1是已经接收到，2..., 3..., 9是接收超时。dist[1]是上一次收到报文的时间
curtis_id_dict = {"0x1A6" : {"state": 0, "pre_time": None}, "0x1A2" : {"state": 0, "pre_time": None}}
motec_id_dict = {"0x701" : {"state": 0, "pre_time": None}, "0x181" : {"state": 0, "pre_time": None}, "0x281" : {"state":0, "pre_time": None}}
Rcv_angle = 0
Rcv_speed = 0
mpc_tcp_init_ok = False
mpc_tcp_socket = None
safe_io = 0
warning_io = 0
curtis_1232_err = 0
curtis_1253c_err = 0
final_speed = 0
loading_warning = 0
out_of_control_flag = False
out_of_control_MPC_flag = False
straight_last_point = None
control_mode_switch = IO_controller.get_automode()  # 手动自动开关
loading_state = IO_controller.get_hw()  # 货物开关状态
locator_error = True    # 雷达重复定位过量标志
BMS_SOC = 0
BMS_power_on_flag = 0
BMS_max_monomer_voltage = 0
BMS_battery_status = None
BMS_charger_connection_status = 0
BMS_relay_status = 0

def get_fork_height(ad_value, fork_ad_list=config.fork_ad_list, fork_height_list=config.fork_height_list):
    if len(fork_ad_list) < 2 or len(fork_height_list) < 2:
        logger.warning("货叉高度映射列表出错")
        return 0

    index = 0
    #获取当前ad值在列表中的位置
    for i in range(len(fork_ad_list)):
        if ad_value > fork_ad_list[i]:
            index += 1
        else:
            break

    if index == 0:
        return fork_height_list[0]
    elif index == len(fork_ad_list):
        k = (fork_height_list[index -1] - fork_height_list[index - 2]) / (fork_ad_list[index -1] - fork_ad_list[index - 2])
        b = fork_height_list[index - 1] - fork_ad_list[index - 1] * k
        return k * ad_value + b
    else:
        k = (fork_height_list[index] - fork_height_list[index - 1]) / (fork_ad_list[index] - fork_ad_list[index - 1])
        b = fork_height_list[index] - fork_ad_list[index] * k
        return k * ad_value + b

# 将角度转化成[0 ~ 360]
def straight_or_turn(p1, p2):
    while p1.phi < 0:
        p1.phi += math.pi * 2
    
    while p2.phi < 0:
        p2.phi += math.pi * 2

    return False if abs(p1.phi - p2.phi) > (1 / 180 * math.pi) else True

def operate_vehcile_and_fork(speed, wheel_angle, forkspeed):
    speed = int(speed * 60 / math.pi / 0.23 / (1 / 36.04))
    curtis_d0 = speed & 255
    curtis_d1 = (speed & 65280) >> 8
    sw = -((wheel_angle - config.wheel_angle_drift) / math.pi * 180 * 10) # - cfg_candata_of_wheel_offset
    # print("test_sw", sw)
    # 发送数据精确到0.1度
    motecpost = int((config.cfg_position_zero_R - config.cfg_position_zero_L) / 1800 * (sw + 900) + config.cfg_position_zero_L)
    # print("test_motec", motecpost)
    motec_d2 = motecpost & 0xFF
    motec_d3 = (motecpost & 0xFF00) >> 8
    motec_d4 = (motecpost & 0xFF0000) >> 16
    motec_d5 = (motecpost & 0xFF000000) >> 24
    can0.send(0x201, [0x0F, 0x0, motec_d2, motec_d3, motec_d4, motec_d5], False)
    can0.send(0x201, [0x5F, 0x0, motec_d2, motec_d3, motec_d4, motec_d5], False)

    cfg_forkup_speed_255 = config.cfg_forkup_speed_255  # 0.1#全速上升速度
    cfg_forkdown_speed_255 = config.cfg_forkdown_speed_255  # -0.182 #全速下降速度
    curtis_d5 = 0
    curtis_d4 = 0
    up = 0
    if forkspeed > 0:
        up = int(forkspeed / cfg_forkup_speed_255 * 255)
        up = 150 if up > 150 else up
        curtis_d4 = 2
        # print("up--------",up)
    elif forkspeed < 0:
        curtis_d5 = int(forkspeed / cfg_forkdown_speed_255 * 255)
        curtis_d4 = 0
        # print(curtis_d5)
        curtis_d5 = 60 if curtis_d5 < 60 else curtis_d5
        curtis_d5 = 170 if curtis_d5 > 170 else curtis_d5

    can0.send(0x222, [up, 0x0, 0x0, 0x0, 0x0, 0x0], False)
    can0.send(0x226, [curtis_d0, curtis_d1, 0x0, 0x0, curtis_d4, curtis_d5], False)

def mpc_tcp_thread():
    global mpc_tcp_init_ok
    global mpc_tcp_socket
    dest_ip = "127.0.0.1"
    dest_port = 6667
    dest_addr = (dest_ip, dest_port)

    this_thread_name = "mpc_tcp_thread"
    t_check.add_thread(this_thread_name)

    while True:
        t_check.thread_update(this_thread_name)
        if not mpc_tcp_init_ok:
            try:
                # 1.创建套接字socket
                mpc_tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                mpc_tcp_socket.settimeout(5)
                # 2.连接服务器
                logger.info("连接MPC中")
                mpc_tcp_socket.connect(dest_addr)
            except:
                mpc_tcp_init_ok = False
                if mpc_tcp_socket != None:
                    mpc_tcp_socket.close()
                logger.info("MPC连接失败，一秒后重连")
                time.sleep(1)
                continue

            logger.info("MPC连接成功")
            mpc_tcp_init_ok = True
        time.sleep(0.1)

# 上报点抵达，并且删除第一个点
def arrive_and_del_point(point_one, point_two, pathPart):
    this_time = time.time()
    od = {"timestamp":str(round(this_time*1000)), "mac":config.mac, "sequenceId":point_one.sequenceId, "dotId":point_one.dotId, "nodeNum": point_one.nodeNum, "dotNum": point_one.dotNum, "stamp":point_one.stamp, "pathPart": pathPart}
    logger.info("ARRIVE: {}".format(od))
    scheduler_protocol.senddata_to_scheduler("ARRIVE", od)
    point_one = point_two
    scheduler_protocol.del_point()
    od = {"timestamp":str(round(this_time*1000)), "mac":config.mac, "sequenceId":point_one.sequenceId, "dotId":point_one.dotId, "nodeNum": point_one.nodeNum, "dotNum": point_one.dotNum, "stamp":point_one.stamp, "pathPart": pathPart}
    logger.info("ARRIVE: {}".format(od))
    scheduler_protocol.senddata_to_scheduler("ARRIVE", od)
    return point_one

# 如果从目前开始路径为直线，获取直线的最后一个点
def point_travel_thread():
    global straight_last_point
    while True:
        temp_p = None
        point_list = scheduler_protocol.get_current_point_obj_list()
        if len(point_list) > 1:
            for p in point_list[1:]:
                if straight_or_turn(point_list[0], p):
                    temp_p = p
                else:
                    break
        straight_last_point = temp_p
        time.sleep(0.01)

# scheduler_protocol.targetForkHeight = 0.08
def agv_control_thread():
    global mpc_tcp_init_ok
    global final_speed
    global loading_warning
    global out_of_control_flag
    global out_of_control_MPC_flag
    global locator_error

    state_time_pre = 0      # 上一次上报STATE的时间
    arrive_time_pre = 0     # 上一次上报ARRIVE的时间
    [x, y, phi] = [0, 0, 0] # 当前循环的雷达坐标
    coordinate_pre = 0      # 上次循环时的雷达坐标
    point_one = None        # 路径第一个点
    # wheel_angle = 0         # 发送给控制器的舵轮角度值
    pre_foward = -1         # 上一次发送速度的方向
    change_direction_jiasu_count = 24  # 加减速度状态机值
    locator_error_count = 0  # 雷达重复定位计数
    locator_true_count = 0
    fork_count = 0
    rotate_ready = False
    rotate_finish_pre = False
    last_mpc_time = 0
    #scheduler_protocol.targetForkHeight=0.08

    remain_path_len, total_path_len, lateral_drift, phi_drift, r, forwardOrBackward = 0, 0, 0, 0, 0, False

    this_thread_name = "agv_control_thread"
    t_check.add_thread(this_thread_name)
    os.system("echo '' > lidar.log")
    while True:
        try:
            this_time = time.time()
            # t_check.thread_update(this_thread_name)
            # 获取坐标
            if locator_protocol.state:
                coordinate_current = locator_protocol.getValue()
                [x, y, phi] = [int(coordinate_current[0] * 10000) / 10000, int(coordinate_current[1] * 10000) / 10000, int(coordinate_current[2] * 10000) / 10000]
                # print("dot: {}, {}, {}".format(x, y, phi))
                phi += config.locator_drift
                logger.info("当前坐标: {}".format([x, y, phi]))
                screen_ctrl.screenmsg_infopage_od["X:"] = format(x, '.3f')
                screen_ctrl.screenmsg_infopage_od["Y:"] = format(y, '.3f')
                screen_ctrl.screenmsg_infopage_od["Angle:"] = format(phi/math.pi*180, '.2f')
                if coordinate_current == coordinate_pre:
                    # print(locator_error_count)
                    # 雷达坐标未变化
                    if locator_error_count < 48:
                        locator_error_count += 1
                        # logger.info("-雷达数据可用:{} {},{},{}".format(locator_error_count, locator_error, coordinate_current,coordinate_pre))
                    else:
                        locator_error = True
                        locator_true_count = 0
                        # logger.info("-雷达数据不可用:{} {},{},{}".format(locator_error_count, locator_error, coordinate_current,coordinate_pre))
                else:
                    coordinate_pre = coordinate_current
                    locator_error_count = 0
                    locator_true_count += 1
                    if locator_true_count > 10:
                        locator_error = False

            # 获取任务点位和pathPart需要用锁
            point_one, point_two, point_three, pathPart= scheduler_protocol.get_three_point()

            if point_one != None:
                logger.info("p1: {}".format(point_one))
            if point_two != None:
                screen_ctrl.screenmsg_infopage_od["任务消息:"] = format(point_two.x, '.3f') + "  " + format(point_two.y, '.3f')+ "  " + format(point_two.phi * 180 / math.pi, '.0f') + "  " + format(point_two.speed, '.1f')
                # screen_ctrl.screenmsg_taskcont_od["服务器消息"] = "当前任务："+ scheduler_protocol.typeStr + " | " + scheduler_protocol.startName + "-->" + scheduler_protocol.endName
                logger.info("p2: {}".format(point_two))
            else:
                screen_ctrl.screenmsg_infopage_od["任务消息:"] = "无路径"
                # screen_ctrl.screenmsg_taskcont_od["服务器消息"] = ""
                out_of_control_flag = False
                out_of_control_MPC_flag = False
            if point_three != None:
                logger.info("p3: {}".format(point_three))

            if scheduler_protocol.typeStr is not None:
                screen_ctrl.screenmsg_taskcont_od["服务器消息"] = "当前任务："+ scheduler_protocol.typeStr + " | " + scheduler_protocol.startName + "-->" + scheduler_protocol.endName
            else:
                screen_ctrl.screenmsg_taskcont_od["服务器消息"] = ""

            # 舵轮控制量
            adjustDistance, parsePath = scheduler_protocol.get_parsePath()
            speed = 0
            wheel_angle = 0
            current_point = calc.Ponit(x, y, phi) #当前位置
            if point_two != None:
                if straight_or_turn(point_one, point_two):  # p1, p2 为直线
                    #
                    remain_straight_len = 0
                    if straight_last_point != None:
                        try:
                            # 非线程安全
                            remain_straight_len = calc.calcDriftLine(current_point, point_one, straight_last_point)[0]
                        except:
                            pass
                        logger.info("p_straight_last: {}".format(straight_last_point))
                    #

                    remain_path_len, total_path_len, lateral_drift, phi_drift, forwardOrBackward = calc.calcDriftLine(current_point, point_one, point_two)
                    wheel_angle = calc.calcSteerAngle(current_point, lateral_drift, phi_drift, 0, forwardOrBackward, 0, config.wheelBase)
                    logger.info("直线, 剩余长度：{}，总长度：{}，横向偏差：{}，角度偏差：{}，前后：{}，舵轮角度：{}".format(remain_path_len, total_path_len, lateral_drift, phi_drift, forwardOrBackward, wheel_angle * 180/math.pi))
                    # print("直线, 剩余长度：{}，总长度：{}，横向偏差：{}，角度偏差：{}，前后：{}，舵轮角度：{}".format(remain_path_len, total_path_len, lateral_drift, phi_drift, forwardOrBackward, wheel_angle * 180/math.pi))
                    speed = point_two.speed
                    if point_three != None:
                        if straight_or_turn(point_two, point_three): # p1, p2 为直线, p2, p3 为直线
                            logger.info("p1_p2为直线，p2_p3也为直线")
                            remain_path_len_temp = calc.calcDriftLine(point_two, point_two, point_three)[0]
                            if remain_path_len + remain_path_len_temp <= config.slow_down_straight_len:  # p1_p3剩余长度
                                speed = remain_path_len / config.slow_down_straight_len * point_two.speed
                                speed = speed if speed > 0.1 else 0.1  # 限定最低速度

                            if remain_straight_len > config.slow_down_straight_len: # 直线剩余长度
                                speed = point_two.speed
                            elif remain_straight_len > 0.1 and remain_straight_len < config.slow_down_straight_len: # 直线剩余长度
                                speed = remain_straight_len / config.slow_down_straight_len * point_two.speed
                                speed = speed if speed > 0.1 else 0.1  # 限定最低速度

                            if remain_path_len <= 0.1:
                                point_one = arrive_and_del_point(point_one, point_two, pathPart)

                        else: # p1, p2 为直线, p2, p3 为转弯
                            logger.info("p1_p2为直线，p2_p3为转弯")
                            temp = calc.calcDriftArc(point_two, point_two, point_three) # 5
                            next_forwardOrBackward = temp[5]
                            if remain_path_len <= config.slow_down_straight_to_turn_len:
                                speed = remain_path_len / config.slow_down_straight_len * point_two.speed

                            if forwardOrBackward == next_forwardOrBackward:
                                speed = speed if speed > 0.15 else 0.15 # 限定最低速度
                                if remain_path_len <= 0.25:
                                    point_one = arrive_and_del_point(point_one, point_two, pathPart)
                            else:
                                speed = speed if speed > 0.03 else 0.03  # 限定最速度
                                if remain_path_len <= 0.005:
                                    point_one = arrive_and_del_point(point_one, point_two, pathPart)
                    else: # p1, p2 为直线, p3 为空
                        logger.info("p1_p2为直线，后面没有路径")
                        if remain_path_len <= config.slow_down_straight_len:
                            speed = remain_path_len / config.slow_down_straight_len * point_two.speed
                            speed = speed if speed > 0.03 else 0.03  # 限定最速度
                            if remain_path_len <= 0.005:
                                # point_one = point_two
                                # point_two = point_three
                                # scheduler_protocol.del_point()

                                if float(adjustDistance) == 0:
                                    point_one = arrive_and_del_point(point_one, point_two, pathPart)
                                # point_one = arrive_and_del_point(point_one, point_two, pathPart)
                else: # p1, p2 为转弯
                    remain_path_len, total_path_len, lateral_drift, phi_drift, r, forwardOrBackward = calc.calcDriftArc(current_point, point_one, point_two)
                    wheel_angle = calc.calcSteerAngle(current_point, lateral_drift, phi_drift, r, forwardOrBackward, 0, config.wheelBase)
                    logger.info("转弯, 剩余长度：{}，总长度：{}，横向偏差：{}，角度偏差：{}，半径：{}，前后：{}，舵轮角度：{}".format(remain_path_len, total_path_len, lateral_drift, phi_drift, r, forwardOrBackward, wheel_angle * 180/math.pi))
                    # if abs(r) > 0.8:
                    #     speed = 0.28
                    if abs(r) > 0.6:
                        # speed = 0.25
                        speed = 0.15 + 0.1 * (remain_path_len / total_path_len)
                    else:
                        speed = 0.15 + 0.05 * (remain_path_len / total_path_len)

                    if abs(r) < 0.49: # 弧线转直线
                        angle_temp = point_one.phi * 180 / math.pi
                        while angle_temp < 0:
                            angle_temp += 360

                        if (angle_temp + 0.5 < 1 or angle_temp - 0.5 > 359) or (angle_temp > 179 and angle_temp < 181):
                            temp_point = calc.Ponit(point_two.x, point_one.y, point_one.phi)
                        elif  (angle_temp > 89 and angle_temp < 91) or (angle_temp > 269 and angle_temp < 271):
                            temp_point = calc.Ponit(point_one.x, point_two.y, point_one.phi)

                        # temp_point = calc.Ponit(point_two.x, point_one.y, point_one.phi) #当前位置
                        remain_path_len, total_path_len, lateral_drift, phi_drift, forwardOrBackward = calc.calcDriftLine(current_point, point_one, temp_point)
                        wheel_angle = calc.calcSteerAngle(current_point, lateral_drift, phi_drift, 0, forwardOrBackward, 0, config.wheelBase)
                        logger.info("弧线转直线, 剩余长度：{}，总长度：{}，横向偏差：{}，角度偏差：{}，前后：{}，舵轮角度：{}".format(remain_path_len, total_path_len, lateral_drift, phi_drift, forwardOrBackward, wheel_angle * 180/math.pi))
                        speed = 0.02 + (remain_path_len / total_path_len) * 0.1
                        # print("speed1: ", speed)
                        if remain_path_len <= 0.005:
                            scheduler_protocol.rotate_statue = True
                            scheduler_protocol.target_angle = point_two.phi * 180 / math.pi

                    elif point_three != None:
                        logger.info("p1_p2为转弯，后面还有路径")
                        if straight_or_turn(point_two, point_three): # 判断第二段路径的速度是否相反
                            temp = calc.calcDriftLine(point_two, point_two, point_three)
                            next_forwardOrBackward = temp[4]
                        else:
                            temp = calc.calcDriftArc(point_two, point_two, point_three) # 5
                            next_forwardOrBackward = temp[5]

                        if forwardOrBackward == next_forwardOrBackward: #相同的话就不减速到0
                            # temp_r = 0.12 if abs(r) < 0.5 else 0.05#0.15 # 这里要根据轴距来改参数

                            # FIX, CDD10和熊猫车要统一
                            if remain_path_len < 0.15:
                                speed = 0.03 + 0.15 * (remain_path_len / 0.15)
                            temp_r = 0.03
                            if remain_path_len <= temp_r:
                                change_direction_jiasu_count = 5
                                point_one = arrive_and_del_point(point_one, point_two, pathPart)
                        else:                                           #不相同的话就减速到0
                            if remain_path_len < 0.2:
                                speed = 0.05 + 0.05 * (remain_path_len / 0.2)
                                if remain_path_len <= 0.005:
                                    point_one = arrive_and_del_point(point_one, point_two, pathPart)
                    else: #p3 为空
                        logger.info("p1_p2为转弯，后面没有路径")
                        if remain_path_len < 0.25:
                            speed = 0.05 + 0.05 * (remain_path_len / 0.25)
                            if remain_path_len <= 0.03:
                                point_one = arrive_and_del_point(point_one, point_two, pathPart)

                # 限制最高速度
                speed = 1 if speed > 1 else speed

                # 判断速度方向
                if not forwardOrBackward:
                    speed = -speed
                logger.info("求解控制量：{}，{}".format(speed, wheel_angle))

            #MPC 控制
            if float(adjustDistance) != 0:
                t=time.time()
                if not mpc_tcp_init_ok:

                    speed = 0
                    logger.error("MPC连接失败 speed = 0")
                    screen_ctrl.screenmsg_item_safe_append("mpc_not_connected")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("mpc_not_connected")
                    mpc_speed = 0
                    mpc_wheel_angle = 0
                    send_data = format(x, ".4f") + "," + format(y, ".4f") + "," + format(phi, ".4f") + "," + format(Rcv_speed, ".4f") +  "," + format(Rcv_angle, ".4f")  +"," + parsePath  # + "," + "1.0,0.02,1.0"
                    # 接收服务器发送的数据
                    try:
                        mpc_tcp_socket.send(send_data.encode("utf-8"))
                        recv_data = mpc_tcp_socket.recv(1024).decode("utf-8")
                        logger.info("mpc send_data[]: {}".format(send_data))
                    except:
                        mpc_tcp_init_ok = False
                        recv_data = None

                    if recv_data != None:
                        try:
                            recv_data= recv_data.split(',')
                            for i in range(len(recv_data)):
                                recv_data[i]=float(recv_data[i])

                            mpc_speed = recv_data[0]
                            mpc_wheel_angle = recv_data[1]
                            logger.info("mpc recv_data[]: {}".format(recv_data))
                            if int(recv_data[5]) == 100:
                                pass

                            screen_ctrl.screenmsg_infopage_od["MPC_info:"] = ("{:.0f}#{:.0f}#{}".format(recv_data[5], recv_data[2], mpc_speed))
                            if int(recv_data[5]) == 101:
                                mpc_speed = 0
                                scheduler_protocol.adjustDistance = 0
                                point_one = arrive_and_del_point(point_one, point_two, pathPart)

                            if int(recv_data[5]) == 107:
                                logger.info("mpc, 车辆偏差过大")
                                out_of_control_MPC_flag = True
                            else:
                                out_of_control_MPC_flag = False
                        except:
                            logger.error("mpc Error")

                        # if (abs(mpc_wheel_angle) * 180.0 / math.pi) > 6:  # 调整角度限幅
                        #     mpc_wheel_angle = 0.1 * (abs(mpc_wheel_angle) / mpc_wheel_angle)

                        speed = mpc_speed
                        wheel_angle = mpc_wheel_angle
                        logger.info("mpc, speed:{}, angle:{}".format(speed, wheel_angle))
                        print("mpc, remain_path:{}, speed:{}, angle:{}".format(remain_path_len, speed, wheel_angle))
                        #print("mpc_time",round(time.time()-t,4),time.time())
            else:
                timestamp = time.time()
                if timestamp - last_mpc_time > 1:
                    last_mpc_time = timestamp
                    screen_ctrl.screenmsg_infopage_od["MPC_info:"] = "--"
                    send_data = format(x, ".4f") + "," + format(y, ".4f") + "," + format(phi, ".4f") + "," + format(
                        Rcv_speed, ".4f") + "," + format(Rcv_angle, ".4f") + ","
                    try:
                        mpc_tcp_socket.send(send_data.encode("utf-8"))
                        recv_data = mpc_tcp_socket.recv(1024).decode("utf-8")
                        # logger.info("mpc send_data[]: {}".format(send_data))
                    except:
                        print("mpc send error !!!")

            # 原地旋转
            if scheduler_protocol.rotate_statue and float(adjustDistance) == 0:
                #舵轮打到90度
                temp_angle = Rcv_angle / math.pi * 180
                if temp_angle <= 89.9 and not rotate_ready:
                    logger.info("原地旋转打角中 {}:".format(Rcv_angle / math.pi * 180))
                    print("原地旋转打角中")
                    wheel_angle = math.pi / 2
                    speed = 0
                else:
                    rotate_ready = True # 舵轮已经打到90°了，可以进行原地旋转
                    if rotate_finish_pre: # 已经原地旋转完成
                        if temp_angle > 0.1 or temp_angle < -0.1:
                            wheel_angle = 0
                        else:
                            rotate_ready = False
                            rotate_finish_pre = False
                            scheduler_protocol.rotate_statue = False
                            # point_one = point_two
                            # point_two = point_three
                            # scheduler_protocol.del_point()
                            point_one = arrive_and_del_point(point_one, point_two, pathPart)
                            logger.info("原地旋转完成")
                    else:
                        current_angle = (phi * 180 / math.pi + 360) % 360  #将当前角度区间归为 0° ~ 360°
                        clockwise_angle = (current_angle - scheduler_protocol.target_angle + 360) % 360    #如果顺时针转到目标角度（目标角度也将区间归到了0°~360°），需要转的角度
                        anticlockwise_angle = (scheduler_protocol.target_angle - current_angle + 360) % 360    #如果逆时针转到目标角度（目标角度也将区间归到了0°~360°），需要转的角度

                        #顺时针逆时针角度比较，获取最小的转动角度和方向，-1是顺时针，1的逆时针
                        (rotate_angle, rotate_direction) = (clockwise_angle, -1) if clockwise_angle <= anticlockwise_angle else (anticlockwise_angle, 1)

                        logger.info("SERVER_MSG：ROTATE_ANGLE：{}, {}".format(rotate_angle, current_angle))
                        if rotate_angle <= 0.3: #离目标角度只有0.3度的偏差，停止旋转，任务完成
                            rotate_finish_pre = True

                        # FIX, CDD10和熊猫车要统一
                        rotate_speed = 0.005 * rotate_angle + 0.02 #旋转速度
                        rotate_speed = 0.2 * rotate_direction if rotate_speed > 0.2 else rotate_speed * rotate_direction #最大旋转速度不超过0.5
                        speed = rotate_speed
                        wheel_angle = math.pi / 2
                        logger.info("rotate: {}， {}， {}，{}".format(clockwise_angle, anticlockwise_angle, rotate_angle, rotate_speed))

            # 获取当前货叉高度
            fork_ad_value = ad_client.get_ad_value()
            #fork_ad_value = 222
            screen_ctrl.screenmsg_infopage_od["货叉数据:"] = format(fork_ad_value)
            screen_ctrl.screenmsg_infopage_od["货叉电压:"] = format(fork_ad_value * 0.001, '.3f')
            if fork_ad_value <= 0:
                forkHeight = 0
            else:
                forkHeight = get_fork_height(fork_ad_value)
                logger.info("货叉高度：{:.3f} {}".format(forkHeight, fork_ad_value))
                screen_ctrl.screenmsg_infopage_od["货叉高度:"] = format(forkHeight, '.3f')
                tof_camera_data.tof_height = forkHeight

            # loading_state = IO_controller.get_hw() # 货物开关状态
            # loading_state = 0 # 货物开关状态
            # mode = IO_controller.get_automode()  #  手自动开关状态
            #mode = 1 #  手自动开关状态
            park_state = 0 # 停车状态，调度没有用到
            safety_mode = scheduler_protocol.safemode # 雷达模式
            task_flag = scheduler_protocol.task_flag

            # 与调度系统交互，上报STATE和ARRIVE
            if scheduler_protocol.state:
                # 上报 state， 频率3Hz
                if this_time - state_time_pre >= 0.3 and locator_error !=True:
                    state_time_pre = time.time()
                    realtime_dataod = {"timestamp": str(round(this_time*1000)), "mac": config.mac, "mapId": config.cfg_Map_ID, \
                                        "x": x, "y": y, "angle":phi, "height": forkHeight, \
                                        "power": format(BMS_SOC, '.1f'), "loading": loading_state, "parking": park_state, \
                                        "speed": format(Rcv_speed, '.3f'), "mode": control_mode_switch, \
                                        "safety":safety_mode, "status":200, "workStatus": task_flag \
                                        }
                    scheduler_protocol.senddata_to_scheduler("STATE", realtime_dataod)

                # 上报 arrvie，频率2Hz
                if this_time - arrive_time_pre >= 0.5:
                    arrive_time_pre = time.time()
                    if point_one is not None:
                        od = {"timestamp":str(round(this_time*1000)), "mac":config.mac, "sequenceId":point_one.sequenceId, "dotId":point_one.dotId, "nodeNum": point_one.nodeNum, "dotNum": point_one.dotNum, "stamp":point_one.stamp, "pathPart": pathPart}
                        logger.info(od)
                        scheduler_protocol.senddata_to_scheduler("ARRIVE", od)

            # 货叉控制量
            forkspeed = 0
            if scheduler_protocol.targetForkHeight != None:
                deltal_height, forkspeed = fork_control.get_fork_speed(scheduler_protocol.targetForkHeight, forkHeight)
                # print("deltal_height", deltal_height)
                if math.fabs(deltal_height) < 0.010:
                    AudioPlayer.player.del_audio("货叉下降")
                    AudioPlayer.player.del_audio("货叉抬升")
                    fork_count += 1
                    if fork_count >= 6 and math.fabs(deltal_height) < 0.005:
                        forkspeed = 0
                        scheduler_protocol.targetForkHeight = None
                        if scheduler_protocol.FisBlock == 1:
                            scheduler_protocol.FisBlock = 9
                            scheduler_protocol.senddata_to_scheduler("FINISH", {"timestamp":str(round(time.time()*1000)),"mac": config.mac,"sequenceId":scheduler_protocol.sequenceId,"serialNumber":scheduler_protocol.FserialNumber,"stamp": scheduler_protocol.p1stamp, "status": 4})
                            screen_ctrl.screenmsg_taskcont_od["货叉控制"] = "[货叉控制: - ]"
                else:
                    fork_count = 0
                    if deltal_height > 0.008:
                        AudioPlayer.player.add_audio("货叉抬升")
                        AudioPlayer.player.del_audio("货叉下降")
                    elif deltal_height < -0.008:
                        AudioPlayer.player.add_audio("货叉下降")
                        AudioPlayer.player.del_audio("货叉抬升")
            else:
                AudioPlayer.player.del_audio("货叉下降")
                AudioPlayer.player.del_audio("货叉抬升")


            # 光电开关，雷达触发
            if config.radar_switch and warning_io and abs(speed) > 0.3:
                speed = 0.3 * (speed) / abs(speed)
                logger.warning("雷达警告区触发，减速 speed = 0.3")

            if safe_io:
                if safe_io & 0b10000:
                    speed = 0
                    logger.warning("触边胶条触发 speed = 0")
                elif config.radar_switch and safe_io & 0b111100000:
                    speed = 0
                    logger.warning("雷达危险区触发 speed = 0")
                elif config.laser_switch and safe_io & 0b1111:
                    speed = 0
                    logger.warning("光电开关触发 speed = 0")


            # 控制量过滤
            if speed * pre_foward < 0:
                change_direction_jiasu_count = 12*2  # 状态机计数

            # 状态机 消耗加速计数，原来若是车停下来过，则慢慢加速
            if change_direction_jiasu_count > 1 and float(adjustDistance) == 0:
                change_direction_jiasu_count = change_direction_jiasu_count - 0.2
                if change_direction_jiasu_count != 0 and abs(speed) > 0:
                    #speed = speed / change_direction_jiasu_count
                    speed = ((abs(speed) - 0.05) / change_direction_jiasu_count + 0.05) * speed / abs(speed)
                    #print("speed_", speed, change_direction_jiasu_count)

            if speed > 0:
                pre_foward = 1
            elif speed < 0:
                pre_foward = -1

            if not controller_status:
                logger.warning("控制器不可用 speed = 0")
                speed = 0

            if locator_error:
                logger.warning("定位雷达数据不可用 speed = 0")
                speed = 0

            if slip_detection([x, y, phi], Rcv_speed):
                logger.warning("车辆行走异常空转或打滑")
                speed = 0

            global_var.set_val("rcv_speed", Rcv_speed)
            if global_var.get_val("situ_timeout_stop"):
                logger.warning("车辆停车超时---->停车")
                speed = 0

            # 产生一定偏差时限制速度（根据情况进行修改）
            wheel_drift = abs(wheel_angle - Rcv_angle) * 180 / math.pi # 舵轮角度偏差
            if (math.fabs(lateral_drift) > 0.15 or \
                (math.fabs(math.tan(phi_drift)) > (math.tan(20 / 180 * math.pi)) and \
                math.fabs(math.tan(phi_drift - math.pi)) > (math.tan(20 / 180 * math.pi))) or \
                wheel_drift > 10) and \
                abs(speed) > 0.15:

                speed = 0.15 * (speed) / abs(speed)
                logger.warning("车辆偏差较大，speed = 0.15，横向偏差：{}，车身角度偏差：{}度，舵轮角度偏差：{}".format(lateral_drift, phi_drift * 180 / math.pi, wheel_drift))

            # 偏差过大时停车
            if (math.fabs(lateral_drift) > 0.5 or \
                (math.fabs(math.tan(phi_drift)) > (math.tan(50 / 180 * math.pi)) and \
                math.fabs(math.tan(phi_drift - math.pi)) > (math.tan(50 / 180 * math.pi))) or
                wheel_drift > 66) and \
                scheduler_protocol.rotate_statue is False:
                out_of_control_flag = True
                speed = 0
                logger.warning("车辆偏差过大，speed = 0，横向偏差：{}，车身角度偏差：{}度，舵轮角度偏差：{}，".format(lateral_drift, phi_drift * 180 / math.pi, wheel_drift))
            else:
                out_of_control_flag = False

            # 货物开关状态检测
            if config.fork_switch:
                loading_warning = 0
                screen_ctrl.screenmsg_item_safe_rm("load_err")
                screen_ctrl.screenmsg_item_safe_rm("unload_buthw1_halfway")
                screen_ctrl.screenmsg_item_safe_rm("loadend_buthw0_halfway")
                screen_ctrl.screenmsg_item_safe_rm("load_buthw0_halfway")
                screen_ctrl.screenmsg_item_safe_rm("unloadend_buthw1_halfway")
                if scheduler_protocol.p2context == "LOAD_IN_END":
                    if remain_path_len > 0.05:
                        if loading_state:
                            speed = 0
                            logger.warning("进入取货，货物突出5cm以上")
                            screen_ctrl.screenmsg_item_safe_append("load_err")
                            loading_warning = 1
                    elif remain_path_len < 0.05 and loading_state and point_one is not None and point_two is not None:
                        speed = 0
                        loading_warning = 0
                        scheduler_protocol.p2context = "NO_CHECK" # 这里很重要, 不能一直循环上报arrive和del点
                        logger.info("进入取货，货物突出5cm以内，上报ARRIVE")
                        point_one = arrive_and_del_point(point_one, point_two, pathPart)
                        # scheduler_protocol.p2context = "NO_CHECK"

                elif scheduler_protocol.p2context == "ALWAYS_LOAD":
                    if not loading_state:
                        speed = 0
                        logger.warning("要求有货但无货")
                        screen_ctrl.screenmsg_item_safe_append("load_buthw0_halfway")
                        loading_warning = 1
                elif scheduler_protocol.p2context == "UNLOAD_IN_END":
                    if point_two != None and total_path_len - remain_path_len > 0.08 and loading_state:
                        speed = 0
                        logger.warning("放货退出，要求无货但有货")
                        screen_ctrl.screenmsg_item_safe_append("loadend_buthw0_halfway")
                        loading_warning = 1
                elif scheduler_protocol.p2context == "ALWAYS_UNLOAD":
                    if loading_state:
                        speed = 0
                        logger.warning("要求无货但有货")
                        screen_ctrl.screenmsg_item_safe_append("unload_buthw1_halfway")
                        loading_warning = 1

            # STOP命令
            if scheduler_protocol.stop_mode:
                speed = 0
                logger.warning("STOP命令中 speed = 0")
                screen_ctrl.screenmsg_item_safe_append("stop")
            else:
                screen_ctrl.screenmsg_item_safe_rm("stop")


            # 货叉控制过滤
            if forkHeight == 0:
                forkspeed = 0
                logger.warning("货叉高度数据异常 forkspeed = 0")

            # 充电-旧电池
            if scheduler_protocol.charge_status and abs(Rcv_speed) < 0.01 and BMS_battery_status is None:
                if BMS_SOC < scheduler_protocol.power:
                    IO_controller.set_relay_io(2, 1)
                    speed = 0
                else:
                    scheduler_protocol.charge_status = 0
                    IO_controller.set_relay_io(2, 0)
                    scheduler_protocol.senddata_to_scheduler("FINISH", {"timestamp":str(round(time.time()*1000)),"mac": config.mac,"sequenceId":scheduler_protocol.sequenceId,"serialNumber":scheduler_protocol.FserialNumber,"stamp": scheduler_protocol.p1stamp, "status": 4})
            else:
                IO_controller.set_relay_io(2, 0)

            # 充电-新电池
            if scheduler_protocol.charge_status and abs(Rcv_speed) < 0.01:
                if BMS_battery_status == 0:
                    # 发送允许充电
                    can0.send(0x1800EFF3, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], True)

                if BMS_SOC >= scheduler_protocol.power:
                    scheduler_protocol.charge_status = 0
                    scheduler_protocol.senddata_to_scheduler("FINISH", {"timestamp": str(round(time.time() * 1000)), "mac": config.mac, "sequenceId": scheduler_protocol.sequenceId, "serialNumber": scheduler_protocol.FserialNumber, "stamp": scheduler_protocol.p1stamp, "status": 4})
            else:
                if BMS_battery_status == 2:
                    # 发送禁止充电
                    can0.send(0x1800EFF3, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], True)
                    speed = 0

            # 语音播放器 -- 行走
            if speed > 0:
                AudioPlayer.player.add_audio("前进")
                AudioPlayer.player.del_audio("后退")
            elif speed < 0:
                AudioPlayer.player.add_audio("后退")
                AudioPlayer.player.del_audio("前进")
            else:
                AudioPlayer.player.del_audio("前进")
                AudioPlayer.player.del_audio("后退")

            if abs(wheel_angle) < (30 / 180 * math.pi):
                AudioPlayer.player.del_audio("转弯")
            elif abs(wheel_angle) > (30 / 180 * math.pi):
                AudioPlayer.player.add_audio("转弯")
                AudioPlayer.player.del_audio("前进")
                AudioPlayer.player.del_audio("后退")

            # 手动控制
            # if not IO_controller.get_automode():
            if not control_mode_switch:
                AudioPlayer.player.del_audio("转弯")
                AudioPlayer.player.del_audio("前进")
                AudioPlayer.player.del_audio("后退")
                AudioPlayer.player.del_audio("货叉下降")
                AudioPlayer.player.del_audio("货叉抬升")
                speed = 0
                forkspeed = 0

                if ps2_handle.handle_fd != -1 and ps2_handle.ps2_exists:  # 手柄控制
                    speed, wheel_angle, forkspeed= ps2_handle.ps_control()

                elif screen_ctrl.hand_ctrl_flag:
                    if screen_ctrl.rcv_dig == 0x02:
                        speed = 0.1
                    elif screen_ctrl.rcv_dig == 0x04:
                        speed = -0.1
                    elif screen_ctrl.rcv_dig == 0x10:
                        forkspeed = 0.1
                    elif screen_ctrl.rcv_dig == 0x20:
                        forkspeed = -0.06

                    speed_multiple = screen_ctrl.rcv_press_board - 2  # 屏幕调速
                    if screen_ctrl.rcv_press_board == 0:
                        speed_multiple = 2
                    if speed_multiple > 0 and speed != 0 and config.speed_regulation:
                        speed = speed / abs(speed) * speed_multiple * 0.1
                    # print("speed:", speed, screen_ctrl.rcv_press_board, speed_multiple)

                    # 因为已经习惯反了，所以屏幕控制值与实际发送的值是相反的
                    wheel_angle = -screen_ctrl.rcv_angle / 10 / 180 * math.pi

            # 当速度为0时，重置状态机
            if not speed:
                change_direction_jiasu_count = 12*2
            else:
                out_of_control_flag = False  # 当速度不为0的时候，证明不是偏差过大状态

            # 调用控制接口
            if controller_status:
                final_speed = speed
                logger.info("最终发送控制量S A F：{}，{}，{} ;反馈 {:.4f},{:.4f}".format(speed, wheel_angle, forkspeed, Rcv_speed, Rcv_angle))
                # print("最终发送控制量：{}，{}，{}".format(speed, wheel_angle, forkspeed))
                operate_vehcile_and_fork(speed, wheel_angle, forkspeed)
                # operate_vehcile_and_fork(-0.1, 0, forkspeed)
                # temp = str(int(time.time()*1000)) + " "+ format(x, ".4f") + " " + format(y, ".4f") + " " + format(phi, ".4f") + " " + format(Rcv_speed, ".4f") +  " " + format(Rcv_angle, ".4f")
                # os.system("echo '" + temp + "' >> lidar.log")

            # if Rcv_angle > 1.5707:
            #     operate_vehcile_and_fork(0.05, math.pi/2, forkspeed)
            # else:
            #     operate_vehcile_and_fork(0, math.pi/2, forkspeed)
            screen_ctrl.screenmsg_infopage_od["横向偏差:"] = format(lateral_drift, '.3f')
            screen_ctrl.screenmsg_infopage_od["角度偏差:"] = format(phi_drift/math.pi*180, '.3f')
            screen_ctrl.screenmsg_infopage_od["舵轮偏差:"] = format(0)
            screen_ctrl.screenmsg_infopage_od["路段长度:"] = format(total_path_len, '.3f')
            screen_ctrl.screenmsg_infopage_od["剩余长度:"] = format(remain_path_len, '.3f')

            d_time = 0.04 - round(time.time()-this_time, 4)
            if d_time > 0:
                time.sleep(d_time)
            else:
                print("control_dtime too long !!!!", round(time.time()-this_time, 4))
            #print("control_dtime ", round(time.time()-this_time, 4))
            t_check.thread_update(this_thread_name)
        except Exception as e:
            logger.error("Is_detail:{}".format(e))
            logger.error("Is_file:  {}".format(e.__traceback__.tb_frame.f_globals["__file__"]))
            logger.error("Is_lines: {}".format(e.__traceback__.tb_lineno))


def recv_can_thread():
    global curtis_id_dict
    global motec_id_dict
    global Rcv_angle
    global Rcv_speed
    global curtis_1232_err
    global curtis_1253c_err
    global BMS_SOC
    global BMS_power_on_flag
    global BMS_max_monomer_voltage
    global BMS_battery_status
    global BMS_charger_connection_status
    global BMS_relay_status

    calculation_angle = lambda x: x if x < (1 << 31) else (x & ((1 << 32) - 1)) - (1 << 32)
    calculation_speed = lambda x: x if x < (1 << 15) else (x & ((1 << 16) - 1)) - (1 << 16)
    _revSpeed2Speed = lambda x: x * (1 / 36.04) * 0.23 * math.pi / 60

    this_thread_name = "recv_can_thread"
    t_check.add_thread(this_thread_name)
    while True:
        try:
            # t_check.thread_update(this_thread_name)
            this_time = time.time()
            try:
                can_msg = can0.recv(1)
                can_msg_id = can_msg.arbitration_id
                can_msg_data = can_msg.data
            except:
                logger.error("can error")
                continue

            if can_msg_id == 0x1A6:
                curtis_id_dict["0x1A6"]["pre_time"] = this_time
                curtis_id_dict["0x1A6"]["state"] = 1  # curtis ok
                speed_data = can_msg_data[1] << 8 | can_msg_data[0]
                Rcv_speed = _revSpeed2Speed(calculation_speed(speed_data))
                curtis_1232_err = can_msg_data[5]
                if curtis_1232_err == 0x51 or curtis_1232_err == 0x52:
                    screen_ctrl.screenmsg_infopage_od["行走故障:"] = hex(curtis_1232_err)
                    curtis_1232_err = 0

            elif can_msg_id == 0x1A2:
                curtis_id_dict["0x1A2"]["pre_time"] = this_time
                curtis_id_dict["0x1A2"]["state"] = 1 # curtis ok
                curtis_1253c_err = can_msg_data[2]

            elif can_msg_id == 0x701:
                motec_id_dict["0x701"]["pre_time"] = this_time
                if can_msg_data[0] == 0x7F:
                    motec_id_dict["0x701"]["state"] = 1  # motec 上电心跳
                elif can_msg_data[0] == 0x05:
                    motec_id_dict["0x701"]["state"] = 2  # motec 使能心跳
                else:
                    motec_id_dict["0x701"]["state"] = 9
                    logger.warning('0x701 未知状态')

            elif can_msg_id == 0x181:
                motec_id_dict["0x181"]["pre_time"] = this_time
                if can_msg_data[0] == 0x50:
                    motec_id_dict["0x181"]["state"] = 1
                elif can_msg_data[0] == 0x31:
                    motec_id_dict["0x181"]["state"] = 2
                elif can_msg_data[0] == 0x33:
                    motec_id_dict["0x181"]["state"] = 3
                elif can_msg_data[0] == 0x37:
                    motec_id_dict["0x181"]["state"] = 4
                    angle_data = can_msg_data[7] << 24 | can_msg_data[6] << 16 | can_msg_data[5] << 8 | can_msg_data[4]
                    angle = (calculation_angle(angle_data)-config.cfg_position_zero_L)/((config.cfg_position_zero_R - config.cfg_position_zero_L)/180)-90
                    Rcv_angle = -angle / 180 * math.pi + config.wheel_angle_drift  # 弧度制
                else:
                    motec_id_dict["0x181"]["state"] = 9
                    logger.warning('0x181 未知状态')

            elif can_msg_id == 0x281:
                # print(can_msg_data[2])
                motec_id_dict["0x281"]["pre_time"] = this_time
                if can_msg_data[2] == 1:
                    motec_id_dict["0x281"]["state"] = 3
                elif can_msg_data[4] == 1:
                    motec_id_dict["0x281"]["state"] = 2
                elif can_msg_data[4] == 0:
                    motec_id_dict["0x281"]["state"] = 1
                else:
                    motec_id_dict["0x281"]["state"] = 9  # motec 使能心跳
                    logger.warning('0x281 未知状态')

                motec_err = can_msg_data[3] << 24 | can_msg_data[2] << 16 | can_msg_data[1] << 8 | can_msg_data[0]
                bytearray_to_hex = lambda can_msg: ' '.join(hex(x) for x in can_msg)
                if motec_err:
                    # print("MOTEC故障:{0} [0x281] {1}".format(hex(motec_err), bytearray_to_hex(can_msg_data)))
                    logger.error("MOTEC故障:{0} [0x281] {1}".format(hex(motec_err), bytearray_to_hex(can_msg_data)))


            elif can_msg_id == 0x1801EFF3:
                BMS_power_on_flag = can_msg_data[0]
                BMS_SOC = (can_msg_data[1] + (can_msg_data[2] << 8)) * 0.01
                # print("允许上电：{}，SOC: {}，".format(BMS_power_on_flag, BMS_SOC))

            elif can_msg_id == 0x1802EFF3:
                BMS_max_monomer_voltage = (can_msg_data[0] + (can_msg_data[1] << 8)) * 0.001
                # print("最高单体电压：{}".format(BMS_max_monomer_voltage * 0.001))

            elif can_msg_id == 0x1807EFF3:
                BMS_battery_status = can_msg_data[0]
                BMS_charger_connection_status = can_msg_data[1]
                BMS_relay_status = can_msg_data[3]
                # print("电池状态：{}，充电机连接状态：{}，继电器状态：{}".format(BMS_battery_status, BMS_charger_connection_status, BMS_relay_status))

            elif can_msg_id == 0x1800FFF4:
                BMS_SOC = can_msg_data[4] * 0.4
                BMS_max_monomer_voltage = ((can_msg_data[0] << 8) + can_msg_data[1]) / 1000

            # TOF 报文
            elif can_msg_id == 0x205:
                print("TOF -->0x205: {}".format(can_msg_data))
                tof_camera_data.rcv_tof[0] = can_msg_data[1] << 8 | can_msg_data[0]
                tof_camera_data.rcv_tof[1] = can_msg_data[3] << 8 | can_msg_data[2]
                tof_camera_data.rcv_tof[2] = can_msg_data[5] << 8 | can_msg_data[4]
                tof_camera_data.rcv_tof[3] = can_msg_data[7] << 8 | can_msg_data[6]

            elif can_msg_id == 0x705:
                tof_camera_data.tof_enable = 200

            t_check.thread_update(this_thread_name)
        except Exception as e:
            logger.error("Is_detail:{}".format(e))
            logger.error("Is_file:  {}".format(e.__traceback__.tb_frame.f_globals["__file__"]))
            logger.error("Is_lines: {}".format(e.__traceback__.tb_lineno))


def controller_init_thread():
    global curtis_id_dict
    global motec_id_dict
    global controller_status
    curtis_state = False
    motec_state = False
    motec_zero_command = False
    mode_switch_flag = False
    this_thread_name = "controller_init_thread"
    t_check.add_thread(this_thread_name)

    while True:
        try:
            # t_check.thread_update(this_thread_name)
            if motec_id_dict.get("0x701").get("state") == 0:       # & motec_id_dict.get("0x181").get("state") & motec_id_dict.get("0x281").get("state"):
                can0.send(0x000, [0x1, 0x00], False)

            # 首先判断motec和curtis的CAN节点状态
            if False == (curtis_id_dict.get("0x1A6").get("state") and curtis_id_dict.get("0x1A2").get("state")):
                curtis_state = False
                logger.warning('未读取到科蒂斯控制器报文 (0x1A6) 或 (0x1A2)')
            else:
                curtis_state = True

            if False == (motec_id_dict.get("0x701").get("state") and motec_id_dict.get("0x181").get("state") and motec_id_dict.get("0x281").get("state")):
                motec_init_prepare = False
                logger.warning('未读取到Motec控制器报文 (0x701) 或 (0x181) 或 ("0x281")')
            else:
                motec_init_prepare = True

            # 然后判断motec和curtis的CAN节点是否超时
            this_time = time.time()
            for i in motec_id_dict.keys():
                if motec_id_dict[i].get("pre_time") == None:
                    logger.warning('未读取到Motec控制器报文：' + i)
                    continue

                elif this_time - motec_id_dict[i].get("pre_time") > 3: #超过三秒
                    logger.warning('Motec控制器报文：(' + i + ') 超时' )
                    motec_init_prepare = False

            for i in curtis_id_dict.keys():
                if curtis_id_dict[i].get("pre_time") == None:
                    logger.warning('未读取到科蒂斯控制器报文：' + i)
                    continue

                elif this_time - curtis_id_dict[i].get("pre_time") > 3:
                    logger.warning('科蒂斯控制器报文：(' + i + ') 超时' )
                    curtis_state = False

            if motec_init_prepare: # motec的报文都有
                if motec_id_dict.get("0x701").get("state") == 2:
                    # print("0x181_state", motec_id_dict.get("0x181").get("state"))
                    if motec_id_dict.get("0x181").get("state") == 2:   # 发之前 31 发了之后应该为 33
                        can0.send(0x201, [0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], False)
                        mode_switch_flag = False

                    elif motec_id_dict.get("0x181").get("state") == 1 or mode_switch_flag:     # 发之前 50 发了之后应该为 31
                        can0.send(0x201, [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], False)

                    elif motec_id_dict.get("0x181").get("state") == 3:   # 发之前 33 发了之后应该为 37
                        can0.send(0x201, [0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], False)

                    elif motec_id_dict.get("0x181").get("state") == 4:
                        # print("0x281_state", motec_id_dict.get("0x281").get("state"))
                        if motec_id_dict.get("0x281").get("state") == 1: # FIX 如果是带手柄的，加入手自动判断
                            time.sleep(0.1)
                            if motec_id_dict.get("0x281").get("state") == 1:  # 回中完成判断
                                if motec_zero_command and not motec_state:  # 已经发送回零命令，并且已经完成回零动作
                                    logger.info("回零动作完成")
                                    # 如果零点有偏置，在这做了偏置再将motec_state = True
                                    motec_state = True

                                elif not motec_zero_command: # 还没有发送回零命令
                                    can0.send(0x201, [0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00], False)
                                    logger.info("发送回零命令")

                        elif motec_id_dict.get("0x281").get("state") == 2:
                            motec_zero_command = True # 已经发送命令，并且正在执行回零动作
                            logger.info("正在执行回零动作")

                        elif motec_id_dict.get("0x281").get("state") == 3:
                            mode_switch_flag = True
                            motec_state = False
                            motec_zero_command = False
                            logger.info("从手动切换到自动")

            controller_status = 1 if motec_state and curtis_state else 0  # 控制器总状态
            # logger.debug("控制器状态：" + str(controller_status))
            time.sleep(0.05)
            t_check.thread_update(this_thread_name)
        except Exception as e:
            logger.error("Is_detail:{}".format(e))
            logger.error("Is_file:  {}".format(e.__traceback__.tb_frame.f_globals["__file__"]))
            logger.error("Is_lines: {}".format(e.__traceback__.tb_lineno))


def safe_io_thread():
    global safe_io
    global warning_io
    global loading_state
    global control_mode_switch
    IO_controller.set_relay_io(3, 1) # 将三号继电器打开（电源继电器）
    this_thread_name = "safe_io_thread"
    t_check.add_thread(this_thread_name)

    while True:
        try:
            this_time = time.time()
            # t_check.thread_update(this_thread_name)
            safe_io_temp = 0
            warning_io_temp = 0
            control_mode_switch = IO_controller.get_automode() # 手动自动开关
            loading_state = IO_controller.get_hw() # 货物开关状态

            # 模式切换
            IO_controller.set_radar_mode(scheduler_protocol.safemode)

            if IO_controller.get_ztz():
                safe_io_temp |= 0b1
            if IO_controller.get_zty():
                safe_io_temp |= 0b10
            if IO_controller.get_hcz():
                safe_io_temp |= 0b100
            if IO_controller.get_hcy():
                safe_io_temp |= 0b1000
            if IO_controller.get_cbjt():
                safe_io_temp |= 0b10000

            radar_list = IO_controller.read_radar_data(config.radar_num)
            if radar_list[0]:
                safe_io_temp |= 0b100000
            if radar_list[3]:
                safe_io_temp |= 0b1000000
            if radar_list[6]:
                safe_io_temp |= 0b10000000
            if radar_list[9]:
                safe_io_temp |= 0b100000000

            safe_io = safe_io_temp

            if radar_list[1]:
                warning_io_temp |= 0b1
            if radar_list[2]:
                warning_io_temp |= 0b10
            if radar_list[4]:
                warning_io_temp |= 0b100
            if radar_list[5]:
                warning_io_temp |= 0b1000
            if radar_list[7]:
                warning_io_temp |= 0b10000
            if radar_list[8]:
                warning_io_temp |= 0b100000
            if radar_list[10]:
                warning_io_temp |= 0b1000000
            if radar_list[11]:
                warning_io_temp |= 0b10000000

            warning_io = warning_io_temp
            time.sleep(0.01)
            #print("Io_safe_dtime", round(time.time() - this_time, 4))
            t_check.thread_update(this_thread_name)
        except Exception as e:
            logger.error("Is_detail:{}".format(e))
            logger.error("Is_file:  {}".format(e.__traceback__.tb_frame.f_globals["__file__"]))
            logger.error("Is_lines: {}".format(e.__traceback__.tb_lineno))

def info_thread():
    led_state = 0
    stall_count = 0  # 堵转计数
    this_thread_name = "info_thread"
    t_check.add_thread(this_thread_name)
    while True:
        try:
            this_time1 = time.time()
            logger.info("线程检测: {} 运行中线程数:{} ".format(t_check.thread_dict, threading.active_count()))
            # print(IO_controller.read_radar_data(2))
            logger.info("手自动开关: {}".format(IO_controller.get_automode()))
            # 货物开关状态
            screen_ctrl.screenmsg_infopage_od["货叉状态:"] = format(loading_state)

            # 雷达模式
            screen_ctrl.screenmsg_infopage_od["安全模式:"] = format(scheduler_protocol.safemode)

            # 服务器心跳
            screen_ctrl.screenmsg_infopage_od["心跳:"] = format(scheduler_protocol.heart_beat)

            # 单体最高电压
            screen_ctrl.screenmsg_infopage_od["单体最高电压:"] = format(BMS_max_monomer_voltage, '.3f')

            # 舵轮信息
            screen_ctrl.screenmsg_infopage_od["舵轮角度:"] = format(Rcv_angle * 180 / math.pi, '.2f')
            screen_ctrl.screenmsg_infopage_od["速度(km/h):"] = format(Rcv_speed, '.2f')

            # 光电开关以及雷达危险区
            if config.laser_switch:
                if safe_io & 0b1:
                    logger.warning("左支腿触发")
                    screen_ctrl.screenmsg_item_safe_append("safeztz")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safeztz")

                if safe_io & 0b10:
                    logger.warning("右支腿触发")
                    screen_ctrl.screenmsg_item_safe_append("safezty")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safezty")

                if safe_io & 0b100:
                    logger.warning("左货叉触发")
                    screen_ctrl.screenmsg_item_safe_append("safehcz")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safehcz")

                if safe_io & 0b1000:
                    logger.warning("右货叉触发")
                    screen_ctrl.screenmsg_item_safe_append("safehcy")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safehcy")

            if safe_io & 0b10000:
                logger.warning("触边胶条触发")
                screen_ctrl.screenmsg_item_safe_append("safecbjt")
            else:
                screen_ctrl.screenmsg_item_safe_rm("safecbjt")

            if config.radar_switch:
                if safe_io & 0b100000:
                    screen_ctrl.screenmsg_item_safe_append("safezbz1zone")
                    logger.warning("1号雷达危险区触发")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safezbz1zone")

                if safe_io & 0b1000000:
                    screen_ctrl.screenmsg_item_safe_append("safeybz1zone")
                    logger.warning("2号雷达危险区触发")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safeybz1zone")

                if safe_io & 0b10000000:
                    screen_ctrl.screenmsg_item_safe_append("safesbz1zone")
                    logger.warning("3号雷达危险区触发")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safesbz1zone")

                if safe_io & 0b100000000:
                    screen_ctrl.screenmsg_item_safe_append("safexbz1zone")
                    logger.warning("4号雷达危险区触发")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safexbz1zone")

                # 警告区
                if warning_io & 0b1:
                    logger.info("1号雷达警告2区触发")
                    screen_ctrl.screenmsg_item_safe_append("safezbz2zone")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safezbz2zone")

                if warning_io & 0b10:
                    logger.info("1号雷达警告1区触发")
                    screen_ctrl.screenmsg_item_safe_append("safezbz3zone")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safezbz3zone")

                if warning_io & 0b100:
                    logger.info("2号雷达警告2区触发")
                    screen_ctrl.screenmsg_item_safe_append("safeybz2zone")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safeybz2zone")

                if warning_io & 0b1000:
                    logger.info("2号雷达警告1区触发")
                    screen_ctrl.screenmsg_item_safe_append("safeybz3zone")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safeybz3zone")

                if warning_io & 0b10000:
                    logger.info("3号雷达警告2区触发")
                    screen_ctrl.screenmsg_item_safe_append("safesbz2zone")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safesbz2zone")

                if warning_io & 0b100000:
                    logger.info("3号雷达警告1区触发")
                    screen_ctrl.screenmsg_item_safe_append("safesbz3zone")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safesbz3zone")

                if warning_io & 0b1000000:
                    logger.info("4号雷达警告2区触发")
                    screen_ctrl.screenmsg_item_safe_append("safexbz2zone")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safexbz2zone")

                if warning_io & 0b10000000:
                    logger.info("4号雷达警告1区触发")
                    screen_ctrl.screenmsg_item_safe_append("safexbz3zone")
                else:
                    screen_ctrl.screenmsg_item_safe_rm("safexbz3zone")

            # 控制器报错
            if curtis_1232_err:
                logger.error("行走控制器1232错误：{}".format(curtis_1232_err))
                screen_ctrl.screenmsg_infopage_od["行走故障:"] = hex(curtis_1232_err)
                screen_ctrl.screenmsg_item_safe_append("curtis_1232_err")
            else:
                screen_ctrl.screenmsg_infopage_od["行走故障:"] = "0"
                screen_ctrl.screenmsg_item_safe_rm("curtis_1232_err")

            if curtis_1253c_err:
                logger.error("泵控1253c错误：{}".format(curtis_1253c_err))
                screen_ctrl.screenmsg_item_safe_append("curtis_1253c_err")
                screen_ctrl.screenmsg_infopage_od["泵控故障:"] = hex(curtis_1253c_err)
            else:
                screen_ctrl.screenmsg_item_safe_rm("curtis_1253c_err")
                screen_ctrl.screenmsg_infopage_od["泵控故障:"] = "0"

            # 电池
            if BMS_SOC < 30:
                logger.warning("电量低：{}".format(BMS_SOC))
                screen_ctrl.screenmsg_item_safe_append("battery_low")
            else:
                screen_ctrl.screenmsg_item_safe_rm("battery_low")
                logger.info("当前电量: {}".format(BMS_SOC))

            if scheduler_protocol.charge_status:
                logger.info("充电中 speed = 0, 目标电量：{}".format(scheduler_protocol.power))
                screen_ctrl.screenmsg_infopage_od["电池:"] = format(BMS_SOC, ".1f") + "%充电中"
            else:
                screen_ctrl.screenmsg_infopage_od["电池:"] = format(BMS_SOC, ".1f") + "%"

            # 雷达状态
            if locator_error:
                screen_ctrl.screenmsg_item_safe_append("locator_err")
            else:
                screen_ctrl.screenmsg_item_safe_rm("locator_err")

            # 控制器状态
            if not controller_status:  # and control_mode_switch:
                screen_ctrl.screenmsg_item_safe_append("controller_err")
            else:
                screen_ctrl.screenmsg_item_safe_rm("controller_err")
                #IO_controller.set_buzzer(1)

              # 堵转监控
            if controller_status and final_speed != 0 and abs(Rcv_speed) < 0.01 and stall_count < 100:
                stall_count += 1
            else:
                stall_count = 0

            if stall_count >= 100:
                screen_ctrl.screenmsg_item_safe_append("stall_err")
                logger.error("舵轮堵转")  # 查询堵转问题的时候得查看控制线程中final_speed是否还再更新，如果控制线程奔溃也会出现堵转信息
            else:
                screen_ctrl.screenmsg_item_safe_rm("stall_err")

            if locator_protocol.state:
                screen_ctrl.screenmsg_taskcont_od["导航雷达"] = "导航雷达连接成功 "
            else:
                screen_ctrl.screenmsg_taskcont_od["导航雷达"] = "导航雷达正在连接~ "

            if scheduler_protocol.state:
                screen_ctrl.screenmsg_taskcont_od["服务器"] = "[" + config.cfg_car_Num + "] | "+"服务器连接成功#"
            else:
                screen_ctrl.screenmsg_taskcont_od["服务器"] = "服务器正在连接~ "

            if out_of_control_flag or out_of_control_MPC_flag:
                screen_ctrl.screenmsg_item_safe_append("outofctrl")
            else:
                screen_ctrl.screenmsg_item_safe_rm("outofctrl")

            led_color = ""
            if config.radar_switch and warning_io:
                led_color = "yellow"

            # print("mpc", mpc_tcp_init_ok)
            if (config.radar_switch and safe_io & 0b111100000) or \
               (config.laser_switch and safe_io & 0b1111) or \
               (config.fork_switch and loading_warning) or \
               (not controller_status) or \
               (BMS_SOC < 30) or \
               (curtis_1232_err) or \
               (curtis_1253c_err) or \
               (stall_count >= 100) or \
               (not locator_protocol.state) or \
               (not scheduler_protocol.state) or \
               (out_of_control_flag or out_of_control_MPC_flag) or\
               (not mpc_tcp_init_ok) or \
               slip_stop or \
               locator_error:
                led_color = "red"

            # led三色灯
            led_state ^= 1
            if led_color == "red":  # FIX 这里还要加上故障列表
                IO_controller.set_led_r(led_state)
                IO_controller.set_led_y(0)
                IO_controller.set_led_g(0)
            elif led_color == "yellow":
                IO_controller.set_led_r(0)
                IO_controller.set_led_y(led_state)
                IO_controller.set_led_g(0)
            else:
                IO_controller.set_led_r(0)
                IO_controller.set_led_y(0)
                IO_controller.set_led_g(led_state)

            cpu_temp = subprocess.getstatusoutput('cat /sys/class/thermal/thermal_zone0/temp')[1]
            screen_ctrl.screenmsg_infopage_od["CPU温度:"] = format(float(cpu_temp) / 1000, '.1f')
            this_time = time.strftime('%m-%d %H:%M:%S')
            screen_ctrl.screenmsg_infopage_od["时间:"] = this_time

            # ping 服务器，同时也将本线程延迟了1秒
            res = subprocess.getstatusoutput('ping -w 1 ' + config.scheduler_ip)
            if not res[0]:
                ping_delay = re.search(r'time=(.*)ms', res[1], re.I).group(1)
                screen_ctrl.screenmsg_infopage_od["ping:"] = format(ping_delay)
                logger.info("ping: {}".format(ping_delay))
            else:
                screen_ctrl.screenmsg_infopage_od["ping:"] = "失败"
                logger.info("ping: 失败")
            # print("info_dtime", round(time.time() - this_time1, 4))
            t_check.thread_update(this_thread_name)
        except Exception as e:
            logger.error("Is_detail:{}".format(e))
            logger.error("Is_file:  {}".format(e.__traceback__.tb_frame.f_globals["__file__"]))
            logger.error("Is_lines: {}".format(e.__traceback__.tb_lineno))


def audio_thread(player):
    this_thread_name = "audio_thread"
    t_check.add_thread(this_thread_name)

    while True:
        t_check.thread_update(this_thread_name)
        if not IO_controller.get_automode():
            time.sleep(1)
            continue
        audio_list = copy.copy(player.audio_list)
        for audio_key in audio_list:
            audio_num = player.audio_infopage_od[audio_key][0]
            audio_time = player.audio_infopage_od[audio_key][1]
            # if 100 == audio_num:
            #     player.del_audio("服务器连接成功")
            #     continue
            # print("audio_num", audio_num)
            # print("audio_list", audio_list)
            player.audio_play(audio_num)
            time.sleep(audio_time)
        player.audio_stop()
        time.sleep(0.1)
        player.audio_stop()
        time.sleep(0.1)


walk_count = 0
pre_loca = [0, 0, 0]
slip_stop = False
def slip_detection(locator, receive_speed):
    global walk_count
    global pre_loca
    global slip_stop
    try:
        [x, y, phi] = copy.copy(locator)
        if abs(receive_speed) > 0.05:
            walk_count += 1
        else:
            walk_count = 0

        if locator_protocol.state and x is not None and pre_loca[0] is not None:
            if (abs(pre_loca[0] - x) > 0.1 or abs(pre_loca[1] - y) > 0.1) and True:
                pre_loca = [x, y, phi]
                walk_count = 0

            else:
                # print("walk_count", walk_count)
                if walk_count > 300:
                    screen_ctrl.screenmsg_item_safe_append("wheel_slip")
                    slip_stop = True
                    print("车辆行走异常空转或打滑")
                elif not control_mode_switch:
                    screen_ctrl.screenmsg_item_safe_rm("wheel_slip")
                    slip_stop = False
        return slip_stop

    except Exception as e:
        print("Is_error :loca", e)



t1 = threading.Thread(target=recv_can_thread)
t2 = threading.Thread(target=controller_init_thread)
t3 = threading.Thread(target=agv_control_thread)
t4 = threading.Thread(target=safe_io_thread)
t5 = threading.Thread(target=mpc_tcp_thread)
t6 = threading.Thread(target=info_thread)
t7 = threading.Thread(target=audio_thread, args=(AudioPlayer.player,))
t8 = threading.Thread(target=point_travel_thread)
t1.start()
t2.start()
t3.start()
t4.start()
t5.start()
t6.start()
t7.start()
t8.start()


from radar_log import radar_client  # 导入雷达日志模块
# from udp_radar import agv_logs  # 导入雷达日志模块 new
while True:
    if IO_controller.get_shutdown():
        logger.info("-----执行关机-----")
        # 屏幕显示关机3秒
        screen_ctrl.is_taskcont_refresh = False
        body = "**********系统关机中!!!**********"
        ebody = body.encode("utf-8") + bytes([0])
        action_code = 0
        screen_ctrl.sendmsg2screen(screen_ctrl.screenmsg_id_od['taskcont'], action_code, ebody)
        time.sleep(3)

        os.system("sudo shutdown now")
    time.sleep(1)

'''
init线程只做初始化
监控线程，监控手自动切换，监控节点接收超时
'''
