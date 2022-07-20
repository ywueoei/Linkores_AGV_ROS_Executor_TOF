import ctypes
import math
import os
import threading
import time

ps2_lib = ctypes.cdll.LoadLibrary('./ps2.so')
ps2_lib.ps2_open.restype = ctypes.c_int32
handle_fd = ps2_lib.ps2_open()
ps2_key = [0] * 9
handle = 0
ps2_exists = 0
handle_mode = 0
motion_mode_count = 0

if handle_fd == -1:
    print("error")
    # exit()
else:
    handle = 1
    ps2_lib.get_ps2_map_pointer.restype = ctypes.c_int64  # 64bit pointer
    map_pointer_base_addr = ps2_lib.get_ps2_map_pointer()
    a_offset = 1 * 4
    x_offset = 3 * 4
    lb_offset = 5 * 4
    rb_offset = 6 * 4
    lt_offset = 7 * 4
    rt_offset = 8 * 4
    select_offset = 9 * 4
    ly_offset = 15 * 4
    rx_offset = 16 * 4
    a  = ctypes.cast(map_pointer_base_addr + a_offset,  ctypes.POINTER(ctypes.c_int32))
    x  = ctypes.cast(map_pointer_base_addr + x_offset,  ctypes.POINTER(ctypes.c_int32))
    lb = ctypes.cast(map_pointer_base_addr + lb_offset, ctypes.POINTER(ctypes.c_int32))
    rb = ctypes.cast(map_pointer_base_addr + rb_offset, ctypes.POINTER(ctypes.c_int32))
    lt = ctypes.cast(map_pointer_base_addr + lt_offset, ctypes.POINTER(ctypes.c_int32))
    rt = ctypes.cast(map_pointer_base_addr + rt_offset, ctypes.POINTER(ctypes.c_int32))
    select = ctypes.cast(map_pointer_base_addr + select_offset, ctypes.POINTER(ctypes.c_int32))
    ly = ctypes.cast(map_pointer_base_addr + ly_offset, ctypes.POINTER(ctypes.c_int32))
    rx = ctypes.cast(map_pointer_base_addr + rx_offset, ctypes.POINTER(ctypes.c_int32))

    speed_mode = lt
    angle_mode = rt
    fork_up = lb
    fork_down = rb
    speed = ly
    angle = rx
    fork_stretch = a
    fork_shrink = x
    motion_mode = select
    ps2_lib.get_value.restype = ctypes.c_int32

    def handle_thread():
        global ps2_key
        global handle_fd
        global ps2_exists
        last_time = 0
        while True:
            len = ps2_lib.get_value()
            ps2_key = [speed_mode.contents.value, angle_mode.contents.value, motion_mode.contents.value, fork_up.contents.value, fork_down.contents.value, fork_stretch.contents.value, fork_shrink.contents.value, speed.contents.value, angle.contents.value]
            print(ps2_key)
            time_t = time.time()
            if time_t - last_time > 2:
                ps2_exists = os.path.exists("/dev/input/js0")
                last_time = time_t

    t = threading.Thread(target=handle_thread)
    t.start()


def ps_control(forkspeed=0):
    global handle_mode, motion_mode_count
    speed_mode = ps2_key[0]
    angle_mode = ps2_key[1]
    motion_mode = ps2_key[2]
    fork_up = ps2_key[3]
    fork_down = ps2_key[4]
    fork_reach = ps2_key[5]
    fork_indent = ps2_key[6]
    speed = -ps2_key[7]
    angle = -ps2_key[8]
    if angle_mode:
        wheel_angle = angle * 90 / 32767 / 180 * math.pi
    else:
        wheel_angle = angle * 60 / 32767 / 180 * math.pi
    if fork_up:
        forkspeed = 0.1
    elif fork_down:
        forkspeed = -0.06
    if speed_mode:
        speed = speed * 0.3 / 32767
    else:
        speed = speed * 0.2 / 32767
    if motion_mode:
        motion_mode_count += 1
        if motion_mode_count == 20:
            handle_mode = not handle_mode
            print("手自动切换",handle_mode)
    else:
        motion_mode_count = 0
    # if not handle_mode:
        # wheel_angle = -wheel_angle
        # speed = -speed
    return speed, wheel_angle, forkspeed


if __name__ == "__main__":
    while 1:
        print(ps2_key)
        time.sleep(0.2)
    
