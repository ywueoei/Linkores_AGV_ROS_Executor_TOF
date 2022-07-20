import wiringpi
import ps2_handle
class IO_Controller():
    # 初始化I\O扩展模块
    def __init__(self):
        wiringpi.wiringPiSetup()

        # 5 CAN2WIFI到位继电器（大部分车没有用到），29 目前没有用，26 刷板继电器，27 电源开关继电器
        self.relay_io = [5, 29, 26, 27]
        for i in self.relay_io:
            wiringpi.pinMode(i, 1)
            wiringpi.digitalWrite(i, 0)

        # （I2C扩展I\O）
        wiringpi.pcf8574Setup(64, 0x21)
        wiringpi.pcf8574Setup(72, 0x22)
        wiringpi.pcf8574Setup(80, 0x24)
        wiringpi.pcf8574Setup(88, 0x25)

        # 输出I\O初始化
        # 64 ~ 67 对应雷达模式引脚
        # 68 ~ 71 对应红、黄、绿、蜂鸣器
        for i in range(64, 72):
            wiringpi.pinMode(i, 1)
            wiringpi.digitalWrite(i, 1)

        # 输入I\O初始化
        # 72 ~ 74 雷达， 76 ~ 78 雷达， 88 ~ 90 雷达， 92 ~ 94 雷达
        # 79 手动、自动开关
        # 80 电源开关
        # 81 回中接近开关（看车型，有的未接）
        # 82 ~ 87 触边胶条、货物开关、货叉左光电、货叉右光电、支腿左光电、支腿右光电
        # 75 91 95 暂时没有用上
        for i in range(72, 96):
            wiringpi.pinMode(i, 0)

    # 继电器开关控制
    def set_relay_io(self, relay_num, on_or_off):
        wiringpi.digitalWrite(self.relay_io[relay_num], on_or_off)

    def set_led_r(self, on_or_off):
        wiringpi.digitalWrite(68, not on_or_off)

    def set_led_y(self, on_or_off):
        wiringpi.digitalWrite(69, not on_or_off)

    def set_led_g(self, on_or_off):
        wiringpi.digitalWrite(70, not on_or_off)

    # 蜂鸣器
    def set_buzzer(self, on_or_off):
        wiringpi.digitalWrite(71, not on_or_off)

    # 支腿左
    def get_ztz(self):
        return wiringpi.digitalRead(87)

    def get_zty(self):
        return wiringpi.digitalRead(86)

    # 货叉左
    def get_hcz(self):
        return wiringpi.digitalRead(84)

    def get_hcy(self):
        return wiringpi.digitalRead(85)

    # 货物开关
    def get_hw(self):
        return wiringpi.digitalRead(83)

    # 舵轮零点电压
    def get_wheelvoltage(self):
        return wiringpi.digitalRead(81)

    # 触边胶条
    def get_cbjt(self):
        return wiringpi.digitalRead(82)

    # 延时关机io，电源开关状态
    def get_shutdown(self):
        return wiringpi.digitalRead(80)

    # 手自动模式
    def get_automode(self):
        # return wiringpi.digitalRead(79)
        # print("手自动切换#########",ps2_handle.handle_mode)
        return ps2_handle.handle_mode

    # 所有安全雷达的控制口都接到了这四个口
    def set_radar_mode(self, radar_mode):
        if radar_mode > 15 or radar_mode < 0:
            return False
        else:
            io_num = 64
            for i in range(4):
                wiringpi.digitalWrite(io_num + i, not (radar_mode >> i & 0b1))
            return True

            # bit_0 = radar_mode >> 0 & 0b1
            # bit_1 = radar_mode >> 1 & 0b1
            # bit_2 = radar_mode >> 2 & 0b1
            # bit_3 = radar_mode >> 3 & 0b1
            # wiringpi.digitalWrite(64, not bit_0)
            # wiringpi.digitalWrite(65, not bit_1)
            # wiringpi.digitalWrite(66, not bit_2)
            # wiringpi.digitalWrite(67, not bit_3)

    def read_radar_data(self, radar_num):
        radar_one   = [wiringpi.digitalRead(88), wiringpi.digitalRead(89), wiringpi.digitalRead(90)]
        radar_two   = [wiringpi.digitalRead(92), wiringpi.digitalRead(93), wiringpi.digitalRead(94)]
        radar_three = [wiringpi.digitalRead(72), wiringpi.digitalRead(73), wiringpi.digitalRead(74)]
        radar_four  = [wiringpi.digitalRead(76), wiringpi.digitalRead(77), wiringpi.digitalRead(78)]

        if radar_num == 0:
            return [0] * 12
        elif radar_num == 1:
            return radar_one + [0] * 9
        elif radar_num == 2:
            return radar_one + radar_two + [0] * 6
        elif radar_num == 3:
            return radar_one + radar_two + radar_three + [0] * 3
        elif radar_num == 4:
            return radar_one + radar_two + radar_three + radar_four

# relayCtrl = relay_ctrl()
IO_controller = IO_Controller()

# import time
# while True:
#     time.sleep(1)