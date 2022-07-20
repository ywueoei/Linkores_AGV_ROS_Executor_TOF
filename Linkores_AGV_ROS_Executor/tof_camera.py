import copy
import math
from can_bus import can0


class TofCamera():
    def __init__(self):
        self.rcv_tof = [0, 0, 0, 0]  # [x, y, phi, height]
        self.send_tof = [0, 0, 0, 0]
        self.tof_enable = 10
        self.tof_height = 0.315
        self.h_offset = 0.23  # 高度补偿

    def send_to_tof(self, ):
        self.tof_enable -= 1
        if self.tof_enable < 0:
            # print('TOF Camera --> not connected')
            return

        data = copy.copy(self.send_tof)
        d_tof = [0, 0, 0, 0, 0, 0, 0, 0]
        x = int(data[0]*1000)
        y = int(data[1]*1000)
        phi = int(data[2]/math.pi*18000)
        h = int((data[3] + self.h_offset) * 1000)
        d_tof[0] = x & 255
        d_tof[1] = (x & 65280) >> 8
        d_tof[2] = y & 255
        d_tof[3] = (y & 65280) >> 8
        d_tof[4] = phi & 255
        d_tof[5] = (phi & 65280) >> 8
        d_tof[6] = h & 255
        d_tof[7] = (h & 65280) >> 8
        can0.send(0x205, d_tof, False)
        print('send tof: {}, X:{}, Y:{}, PHI:{}, H:{}'.format(d_tof, x, y, phi, h))
        print('rcv tof: {}, ---> x:{}, y:{}, phi:{}, h:{}'.format(self.rcv_tof, self.rcv_tof[0], self.rcv_tof[1], self.rcv_tof[2], self.rcv_tof[3]))
        print('差值: ---> dx:{}, dy:{}, dphi:{}, dh:{}'.format(x - self.rcv_tof[0], y - self.rcv_tof[1], phi - self.rcv_tof[2], h - self.rcv_tof[3]))


tof_camera_data = TofCamera()
