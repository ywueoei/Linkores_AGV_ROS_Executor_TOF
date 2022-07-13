import time
import pid_source
from log import logger

fork_up_P = 0.8  #上升P 0.8
fork_up_I = 0.03 #上升I 3
fork_up_D = 1.5  #上升D
#下降最小值23
fork_down_P = 0.85 #下降P 0.55
fork_down_I = 0.02 #下降I
fork_down_D = 1.2 #下降D

class Fork_Controller():
    def __init__(self):
        self.fork_pid = pid_source.Fork_PID()
   
    def get_fork_speed(self, target_height, current_height):
        deltal_height = target_height - current_height
        up_or_down = True if deltal_height > 0 else False
        if up_or_down:
            self.fork_pid.setKp(fork_up_P)
            self.fork_pid.setKi(fork_up_I)
            self.fork_pid.setKd(fork_up_D)
            forkSpeed = self.fork_pid.update_up(target_height, current_height)
        else:
            self.fork_pid.setKp(fork_down_P)
            self.fork_pid.setKi(fork_down_I)
            self.fork_pid.setKd(fork_down_D)
            forkSpeed = self.fork_pid.update_down2(target_height, current_height)
            
        logger.info("货叉控制: 目标高度：{}，当前高度：{}，速度控制量：{}".format('%.2f' % target_height, '%.4f' % current_height, '%.4f' % forkSpeed))
        return deltal_height, forkSpeed


fork_control = Fork_Controller()
# while True:
#     print(fork.get_fork_speed(100, 0.08))
#     time.sleep(0.06)