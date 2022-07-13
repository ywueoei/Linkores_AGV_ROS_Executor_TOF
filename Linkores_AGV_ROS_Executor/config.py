# -*-coding:utf-8 -*-
import os
import math
import yaml

# from pprint import pprint
# yaml_file = "db.yaml"
fs = open(os.path.join("./", "config.yaml"), encoding="UTF-8")
datas = yaml.safe_load(fs)

def get_mac():
    mac_str = os.popen("ip a | grep 'link/ether' | awk -F ' ' '{print $2}'  | head -1 | awk -F ':' '{print $1$2$3$4$5$6}'").read()
    mac = mac_str[0:-1]
    return mac


mac = get_mac()
cfg_car_Num = datas["cfg_car_Num"]
cfg_nav350_edgeoffset = datas["cfg_nav350_edgeoffset"]
cfg_nav350_goodoffset = datas["cfg_nav350_goodoffset"]   # 雷达与从动轮的距离
cfg_nav350_TOF_offset = 0.645  # TOF与从动轮的距离
locator_offset = [cfg_nav350_goodoffset, 0, -math.pi]
tof_camera_offset = [cfg_nav350_goodoffset - cfg_nav350_TOF_offset, 0, -math.pi]
wheelBase = datas["wheelBase"]

# 地图ID
cfg_Map_ID = datas["cfg_Map_ID"]
# 光电开关启用
laser_switch = datas["laser_switch"]
# 避障雷达启用
radar_switch = datas["radar_switch"]
# 雷达数量
radar_num = datas["radar_num"]
# 货物校验开关
fork_switch = datas["fork_switch"]

wheel_angle_drift = datas["wheel_angle_drift"] #datas["wheel_angle_drift"]  # 舵轮偏移，弧度制
locator_drift = datas["locator_drift"]    # 雷达偏移
locator_ip = datas["locator_ip"]  # 雷达ip
scheduler_ip = datas["scheduler_ip"]  # 服务器ip
ad_modular_ip = datas["ad_modular_ip"]   # AD模块ip
fork_height_list = datas["fork_height_list"]  # 货叉高度和AD模块的对应值
fork_ad_list = datas["fork_ad_list"]
cfg_forkup_speed_255 = datas["cfg_forkup_speed_255"]  # 全速上升速度
cfg_forkdown_speed_255 = datas["cfg_forkdown_speed_255"]  # 全速下降速度

#舵轮90（左）（MOTEC-绝对位置）
cfg_position_zero_L = datas["cfg_position_zero_L"]  # YHL or YUFENGPANDA   对应编码器的值
#舵轮90（右）（MOTEC-绝对位置）
cfg_position_zero_R = datas["cfg_position_zero_R"] # YHL or YUFENGPANDA

# 直线长度
slow_down_straight_len = datas["slow_down_straight_len"]
slow_down_straight_to_turn_len = datas["slow_down_straight_to_turn_len"]

# 手动速度调速使能（True 默认速度0.2 False默认0.1 ）
speed_regulation = False
adjust_distance = 0.7
# 播报故障语音
malfunction_broadcast = True
# 任务异常超时检测
task_timeout_check = False

