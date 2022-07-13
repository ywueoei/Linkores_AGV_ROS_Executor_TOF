import threading
import time
from AudioPlayer import player
import global_var


# 报警等消息处理
class msglist_pro():
    def __init__(self):
        self.exception_list = []
        self.exception_time_list = []
        self.alarm_list = []
        self.resendposture_list = []
        self.rcv_speed = 0
        # 安全区域内容字典
        self.alarmcont_od = {
            # key :(safe_level ,content)
            "safecbjt": 4001,  # (3, "触边胶条触发 "),
            "safehcz": 4002,  # (3, "左货叉遇障碍 "),
            "safehcy": 4003,  # (3, "右货叉遇障碍 "),
            "safeztz": 4004,  # (3, "左支腿遇障碍 "),
            "safezty": 4005,  # (3, "右支腿遇障碍 "),
            "safe1zone": 4006,  # (3, "安全雷达高危险1区触发 "),
            "safe2zone": 4007,  # (2, "安全雷达次危险2区触发 "),
            "safe3zone": 4008,  # (2, "安全雷达警告3区触发 "),

            "load_buthw0_nearend": 4009,  # (2, "要求载货中但检测无货NearEnd "),
            "unload_buthw1_nearend": 4010,  # (2, "要求无货但检测有货NearEnd "),
            "unloadend_buthw1_nearend": 4011,  # (2, "卸货完成但检测有货 "),
            "loadend_buthw0_nearend": 4012,  # (2, "装货完成但检测无货 "),

            "load_buthw0_halfway": 4013,  # (2, "要求载货中但检测无货Halfway "),
            "unload_buthw1_halfway": 4014,  # (2, "要求无货但检测有货Halfway "),
            "unloadend_buthw1_halfway": 4015,  # (2, "卸货,货物被拖出 "),
            "loadend_buthw0_halfway": 4016,  # (2, "插货,货物突出 "),
            "xiaojiting_226can": 4017,  # [2, "##电机控制器连接断开，请检查急停按钮是否被按下"],

            "outofctrl": 5001,  # (3, "车辆偏差过大 "),
            "battery_low": 5002,  # (3, "#低电量警告，请及时充电！# "),
            "ctrler_fault": 5003,  # (3, "#行走控制器故障（故障号详见系统信息页）#"),
        }

        self.exception_cont_od = {
            "outofctrl": 5001,  # (3, "车辆偏差过大 "),
            "battery_low": 5002,  # (3, "#低电量警告，请及时充电！# "),
            "ctrler_fault": 5003,  # (3, "#行走控制器故障（故障号详见系统信息页）#")
            "nav_err": 5004,  #
            "scheduler_err": 5005,
            "huochalowestpoint_err": 5006,
        }

    def exception_list_append(self, id, detail=None):
        od = {"id": id, "detail": detail}
        for odold in self.exception_list:
            if odold["id"] == id and odold["detail"] == detail:
                return
        od.update(timestamp=int(time.time()))
        self.exception_list.append(od)

    def exception_list_rm(self, id, detail=None):
        od = {"id": id, "detail": detail}
        for i in range(len(self.exception_list)):
            if self.exception_list[i].get("id") == od.get("id") and self.exception_list[i].get("detail") == od.get(
                    "detail"):
                od.update(timestamp=self.exception_list[i].get("timestamp"))
                self.exception_list.remove(od)
                break

    # 默认使用
    def alarm_list_append(self, id):
        od = {"id": id}
        for odold in self.alarm_list:
            if odold["id"] == id:
                return
        od.update(timestamp=int(time.time()))
        self.alarm_list.append(od)
        # print("ALARM 1", self.alarm_list)

    def alarm_list_rm(self, id):
        od = {"id": id}
        for i in range(len(self.alarm_list)):
            if self.alarm_list[i].get("id") == od.get("id"):
                od.update(timestamp=self.alarm_list[i].get("timestamp"))
                self.alarm_list.remove(od)
                break


    def resendposture_list_append(self, sequenceId, dotId, nodeNum, dotNum, stamp, status=1):
        od = {"mac": MAC, "sequenceId": sequenceId, "dotId": dotId, "nodeNum": nodeNum,
              "dotNum": dotNum, "stamp": stamp, "status": status}
        if od not in self.resendposture_list:
            self.resendposture_list.append(od)

    def resendposture_list_rm(self, dotId, stamp, status=1):
        if len(self.resendposture_list):
            self.resendposture_list.clear()



def sand_alarm_exception_to_scheduler_thread():
    count = 1
    speed_flag = 0
    time.sleep(3)
    from uart_screen import screen_ctrl
    while True:
        # if commander.R_status == 1:
        #     if commander.connected == True:
            if schedule.scheduler_protocol.state:
                if count % 20 == 0:
                    # print("ALARM len :{} {}\r\n".format(len(alarm_msglist.alarm_list),alarm_msglist.alarm_list))
                    if len(alarm_msglist.alarm_list):
                        for alarmget in alarm_msglist.alarm_list:
                            if alarmget.get("id"):
                                alarm_dataod = {"timestamp_now": str(round(time.time() * 1000)), "mac": MAC,
                                                "timestamp": alarmget.get("timestamp"), "alarm": alarmget.get("id")}
                                schedule.scheduler_protocol.senddata_to_scheduler("ALARM", alarm_dataod)
                                if count % 40 == 0 and config.malfunction_broadcast:
                                    player.add_audio("车辆故障")
                                # print("ALARM2 :", alarm_dataod)
                            elif len(alarm_msglist.alarm_list) == 1:
                                alarm_dataod = {"timestamp_now": str(round(time.time() * 1000)), "mac": MAC,
                                                "timestamp": str(round(time.time() * 1000)), "alarm": 200}
                                schedule.scheduler_protocol.senddata_to_scheduler("ALARM", alarm_dataod)
                                player.del_audio("车辆故障")
                                # print("ALARM3 :", alarm_dataod)
                    else:
                        alarm_dataod = {"timestamp_now": str(round(time.time() * 1000)), "mac": MAC,
                                        "timestamp": str(round(time.time() * 1000)), "alarm": 200}
                        schedule.scheduler_protocol.senddata_to_scheduler("ALARM", alarm_dataod)
                        player.del_audio("车辆故障")
                        # print("ALARM3 :", alarm_dataod)

                # 运行异常检测
                if count % 1800 == 0 and (schedule.scheduler_protocol.typeStr == "搬运任务" or schedule.scheduler_protocol.typeStr == "路径任务" or schedule.scheduler_protocol.typeStr == "负载任务" or schedule.scheduler_protocol.typeStr == "卸载任务"):  # stop超时检测
                    from io_controller import IO_controller
                    if IO_controller.get_automode() and speed_flag > 100 and config.task_timeout_check:
                        screen_ctrl.screenmsg_item_safe_append("situ_timeout")
                        global_var.set_val("situ_timeout_stop", True)
                        # print("车辆停止超时--------> STOP")
                else:
                    if abs(global_var.get_val("rcv_speed")) < 0.03:
                        speed_flag += 1
                    else:
                        speed_flag = 0
                        screen_ctrl.screenmsg_item_safe_rm("situ_timeout")
                        global_var.set_val("situ_timeout_stop", False)
                # print("speed_flag:", speed_flag, count)


                if count % 10 == 0:
                    # print("EXCEPTION", len(msglist.exception_list))
                    if len(alarm_msglist.exception_list):
                        for exceptionget in alarm_msglist.exception_list:
                            exception_dataod = {"timestamp_now": str(round(time.time() * 1000)), "mac": MAC,
                                                "timestamp": exceptionget.get("timestamp"),
                                                "alarm": exceptionget.get("id"), "detail": exceptionget.get("detail")}
                            schedule.scheduler_protocol.senddata_to_scheduler("EXCEPTION", exception_dataod)
                            # print("EXCEPTION", exception_dataod)
                            # time.sleep(0.5)
                time.sleep(0.1)
                count += 1
                # print("count", count)
            else:
                time.sleep(0.3)



def get_timestamp():
    s = time.time()
    return time.strftime("%y.%m.%d %H:%M:", time.localtime(s)) + str(float("{0:.4f}".format(s % 60)))


import config
import scheduler_connect as schedule
MAC = config.mac
alarm_msglist = msglist_pro()
t = threading.Thread(target=sand_alarm_exception_to_scheduler_thread)
t.start()


if __name__ == '__main__':
    # gol._init()
    # gol._init_define()
    t = threading.Thread(target=sand_alarm_exception_to_scheduler_thread)
    t.start()
    alarm_msglist.alarm_list_append(alarm_msglist.alarmcont_od.get("safehcy"))
    alarm_msglist.alarm_list_append(alarm_msglist.alarmcont_od.get("load_buthw0_nearend"))
    alarm_msglist.alarm_list_append(alarm_msglist.alarmcont_od.get("outofctrl"))
    alarm_msglist.exception_list_append(alarm_msglist.exception_cont_od.get("scheduler_err"))
    # print(get_timestamp())
