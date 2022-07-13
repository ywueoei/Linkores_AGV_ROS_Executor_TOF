# -*-coding:UTF-8 -*-
import time
import threading
import copy
from can_bus import can0 

class PlayerCtrl():
    def __init__(self):
        self.audio_list = []
        
        self.audio_infopage_od = {}
        self.audio_infopage_od["服务器连接成功"] = [100, 3] #文件编号，播放时间
        self.audio_infopage_od["服务器连接失败"] = [101, 3]
        self.audio_infopage_od["服务器连接断开"] = [102, 3]
        self.audio_infopage_od["前进"] = [200, 3]
        self.audio_infopage_od["后退"] = [201, 3]
        self.audio_infopage_od["左转"] = [202, 3]
        self.audio_infopage_od["右转"] = [203, 3]
        self.audio_infopage_od["货叉抬升"] = [204, 3]
        self.audio_infopage_od["货叉下降"] = [205, 3]
        self.audio_infopage_od["转弯"] = [206, 3]
        self.audio_infopage_od["触发报警减速"] = [300, 3]
        self.audio_infopage_od["触发报警停车"] = [301, 3]
        self.audio_infopage_od["车辆故障"] = [302, 3]

    def add_audio(self, key):
        audio_data = self.audio_infopage_od.get(key, None)
        if audio_data != None:
            if key not in self.audio_list:
                self.audio_list.append(key)
        else:
            print("add error!!!, {} not exist".format(key))

    def del_audio(self, key):
        audio_data = self.audio_infopage_od.get(key, None)
        if audio_data != None:
            if key in self.audio_list:
                self.audio_list.remove(key)
        else:
            print("remove error!!!, {} not exist".format(key))

    def audio_play(self, audio_num):
        d4 = (audio_num & 0xFF00) >> 8
        d5 = audio_num & 0xFF
        can_data = [0x23, 0x00, 0x01, 0x00, d4, d5, 0x1F, 0x00]
        can0.send(0x60A, can_data, False)
    
    def audio_stop(self):
        can_data = [0x23, 0x00, 0x01, 0x00, 0x00, 0x00, 0x1F, 0x00]
        # d4 = (209 & 0xFF00) >> 8
        # d5 = 209 & 0xFF
        # can_data = [0x23, 0x00, 0x01, 0x00, d4, d5, 0x1F, 0x00]
        can0.send(0x60A, can_data, False)


player = PlayerCtrl()


# 模块测试
def audio_thread(player):
    while True:
        audio_list = copy.copy(player.audio_list)
        for audio_key in audio_list:
            audio_num = player.audio_infopage_od[audio_key][0]
            print("audio", audio_num)
            audio_time = player.audio_infopage_od[audio_key][1]
            if 100 == audio_num:
                player.del_audio("服务器连接成功")
                continue
            player.audio_play(audio_num)
            time.sleep(audio_time)
        player.audio_stop()
        print("stop")
        time.sleep(0.2)
    
if __name__ == '__main__':
    player = PlayerCtrl()
    player.add_audio("服务器连接成功")
    player.add_audio("前进")
    player.add_audio("后退")
    t1 = threading.Thread(target = audio_thread, args = (player,))
    t1.start()
    t1.join()
