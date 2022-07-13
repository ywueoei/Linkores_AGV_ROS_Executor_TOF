# -*-coding:utf-8 -*-
import os
import math

def get_mac():
    mac_str = os.popen("ip a | grep 'link/ether' | awk -F ' ' '{print $2}'  | head -1 | awk -F ':' '{print $1$2$3$4$5$6}'").read()
    mac = mac_str[0:-1]
    #mac = "BCM2F1B31184"
    return mac

cfg_car_Num = "CAINIAO-2"
cfg_nav350_edgeoffset = {"front":1.80, "behind":0.95, "left":0.96, "right":0.96}
cfg_nav350_goodoffset = 0.865   #雷达与从动轮的距离
locator_offset = [cfg_nav350_goodoffset, 0, -math.pi]
wheelBase = 1.53

# 地图ID = 31
# cfg_Map_ID = 191#20 #181 #182
cfg_Map_ID = 31
# 光电开关启用
laser_switch = True
# 避障雷达启用
radar_switch = False
# 雷达数量
radar_num = 2
# 货叉开关
fork_switch = False

wheel_angle_drift = 0.0107  # 舵轮偏移，弧度制
locator_drift = -0.0489    # 雷达偏移
mac = get_mac()
locator_ip = "192.168.137.51"  #雷达ip
scheduler_ip = "192.168.1.185"  #服务器ip
ad_modular_ip = "192.168.137.62"   #AD模块ip
fork_height_list= [0.08, 0.111, 0.234, 0.464, 0.860,1.290]  #货叉高度和AD模块的对应值
fork_ad_list= [680, 767, 1072, 1635, 2606,3670]

#舵轮90（左）（MOTEC-绝对位置）
cfg_position_zero_L = -10485760  #YHL or YUFENGPANDA   对应编码器的值
#舵轮90（右）（MOTEC-绝对位置）
cfg_position_zero_R = 10485760 #YHL or YUFENGPANDA


cfg_forkup_speed_255 = 0.1  # 全速上升速度
cfg_forkdown_speed_255 = -0.162  # 全速下降速度
# 直线长度
slow_down_straight_len = 1
slow_down_straight_to_turn_len = 1
# 手动速度调速使能（True 默认速度0.2 False默认0.1 ）
speed_regulation = False
adjust_distance = 0.7
