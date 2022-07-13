# -*-coding:UTF-8 -*-
def _init():
    global _global_dict
    _global_dict = dict(
        rcv_speed=0,
        situ_timeout_stop=False,

    )


def set_val(key, value):
    global _global_dict
    _global_dict[key] = value


def get_val(key, defValue=None):
    global _global_dict
    try:
        return _global_dict[key]
    except KeyError:
        return defValue


_init()
