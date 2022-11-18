import copy
import math
import config
import time
from tof_camera import tof_camera_data
from scheduler_connect import scheduler_protocol
first_flag = 180
mpc_path = None


# Tray identification
class TrayIdentification:
    def __init__(self):
        self.tray_spot_switch = True
        self.tray_path_r = None

    def tray_identification(self, x, y, phi, rcv_tof_dat) -> str:
        global first_flag, mpc_path
        beeline_set = 1.2 #+ 1.5
        config.adjust_distance = 0.7
        print("endPosition----->", x,",",y,",",phi/math.pi*180,first_flag)
        first_flag -= 1
        if first_flag == 0:
            x0 = x
            y0 = y
            # phi0 = phi/math.pi*180 +5
            tray_position_o = copy.copy(rcv_tof_dat)
            tray_position = [tray_position_o[0]/1000, tray_position_o[1]/1000, tray_position_o[2]/100, 0]
            # tray_position = [11.4587 , 2.936 , -9.087, 0]
            # tray_position = [x0, y0, round(phi/math.pi*180, 4), 0]
            dx = tray_position[0] - x0
            dy = tray_position[1] - y0
            phi0 = tray_position[2]
            beeline = - (math.sqrt(dx**2 + dy**2))

            x_initial = tray_position[0] + beeline_set * math.cos(phi0/180*math.pi)
            y_initial = tray_position[1] + beeline_set * math.sin(phi0/180*math.pi)
            print("startPosition----->", x_initial, ",", y_initial, ",", phi0, tray_position)

            if abs(beeline_set) > 0.001 and tof_camera_data.tof_state:
                mpc_path = "100," + format(x_initial, ".4f") + "," + format(y_initial, ".4f") + "," + format(phi0, ".4f") + "," + format(-(beeline_set-0.7), ".4f") +  ",0.0, 0.0, 0.2, 666, 6, 6, -1.27,0.0, 0.0, 0.2, 777, 7, 7," + format(config.adjust_distance, ".4f")
                # scheduler_protocol.targetForkHeight = tof_camera_data.rcv_identify_h/1000
            else:
                # logger.info("Tray position is error")
                # print("Tray position is error")
                print("TOF Control --> path plan is None")
                return None
        else:
            return mpc_path


tray_recognition = TrayIdentification()
