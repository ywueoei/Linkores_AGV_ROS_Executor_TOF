import copy
import math
import config

first_flag = True
mpc_path = None


# Tray identification
class TrayIdentification:
    def __init__(self):
        pass

    def tray_identification(self, x, y, phi, rcv_tof_dat):
        global first_flag, mpc_path
        beeline_set = 1 + 1.5
        config.adjust_distance = 0.7
        print("endPosition----->", x,",",y,",",phi/math.pi*180)
        if first_flag:
            x0 = x
            y0 = y
            # phi0 = phi/math.pi*180 +5
            first_flag = False
            tray_position = copy.copy(rcv_tof_dat)
            # tray_position = [11.4587 , 2.936 , -9.087, 0]
            tray_position = [x0, y0, round(phi/math.pi*180, 4), 0]
            dx = tray_position[0] - x0
            dy = tray_position[1] - y0
            phi0 = tray_position[2]
            beeline = - (math.sqrt(dx**2 + dy**2))

            x_initial = tray_position[0] + beeline_set * math.cos(phi0/180*math.pi)
            y_initial = tray_position[1] + beeline_set * math.sin(phi0/180*math.pi)
            print("startPosition----->", x_initial, ",", y_initial, ",", phi0, tray_position)

            if abs(beeline_set) > 0.001:
                mpc_path = "100," + format(x_initial, ".4f") + "," + format(y_initial, ".4f") + "," + format(phi0, ".4f") + "," + format(-(beeline_set-1.5), ".4f") +  ",0.0, 0.0, 0.2, 6666, 6, 6, -1.5,0.0, 0.0, 0.2, 7777, 7, 7," + format(config.adjust_distance, ".4f")
            else:
                # logger.info("Tray position is error")
                print("Tray position is error")
                return None
        else:
            return mpc_path


tray_recognition = TrayIdentification()
