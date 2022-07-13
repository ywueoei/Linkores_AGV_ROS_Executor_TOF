# -*- coding: UTF-8 -*-
import os
import can
import time

while os.system('sudo ip link set can0 down') != 0:
    time.sleep(1)

while os.system('sudo ip link set can0 up type can bitrate 250000') != 0:
    time.sleep(1)

# os.system('sudo ip link set can0 type can restart-ms 100')

# os.system('sudo ifconfig can0 down')
# os.system('sudo ifconfig can0 up')

class Can():
    def __init__(self) -> None:
        self.can_interface = can.interface.Bus(channel='can0', bustype='socketcan')
        self.can_interface.set_filters(filters)

    def send(self, can_id, can_data, is_extended):
        try:
            msg = can.Message(arbitration_id=can_id, data=can_data, extended_id=is_extended)
            self.can_interface.send(msg, 1.0)
        except:
            pass

    def recv(self, time_out):
        return self.can_interface.recv(time_out)


filters = [{"can_id": 0x226, "can_mask": 0x226, "extended": False},
           {"can_id": 0x1800FFF4, "can_mask": 0x1800FFF4, "extended": True},
           {"can_id": 0x18FF50E5, "can_mask": 0x18FF50E5, "extended": True},
           {"can_id": 0x222, "can_mask": 0x222, "extended": False},
           {"can_id": 0x1A2, "can_mask": 0x1A2, "extended": False},
           {"can_id": 0x701, "can_mask": 0x701, "extended": False},
           {"can_id": 0x000, "can_mask": 0x000, "extended": False},
           {"can_id": 0x181, "can_mask": 0x181, "extended": False},
           {"can_id": 0x281, "can_mask": 0x281, "extended": False},
           {"can_id": 0x201, "can_mask": 0x201, "extended": False},
           {"can_id": 0x1A6, "can_mask": 0x1A6, "extended": False},
           {"can_id": 0x1800EFF3, "can_mask": 0x1800EFF3, "extended": True},
           {"can_id": 0x1801EFF3, "can_mask": 0x1801EFF3, "extended": True},
           {"can_id": 0x1802EFF3, "can_mask": 0x1802EFF3, "extended": True},
           {"can_id": 0x1807EFF3, "can_mask": 0x1807EFF3, "extended": True},
           {"can_id": 0x205, "can_mask": 0x205, "extended": False},
           {"can_id": 0x185, "can_mask": 0x185, "extended": False},
           {"can_id": 0x705, "can_mask": 0x705, "extended": False},
           ]
# # A filter matches, when <received_can_id> & can_mask == can_id & can_mask.
# # If extended is set as well, it only matches messages where <received_is_extended> == extended. Else it
# # matches every messages based only on the arbitration ID and mask.
can0 = Can()
