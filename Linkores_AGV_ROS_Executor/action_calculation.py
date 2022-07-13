import numpy as np
import math

class Ponit():
    def __init__(current, _x, _y, _phi) -> None:
        current.x = _x
        current.y = _y
        current.phi = _phi


def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0] * p2[1] - p2[0] * p1[1])
    return A, B, -C

def intersection(L1, L2):
    D = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    
    if D != 0:
        x = Dx / D
        y = Dy / D
        return True, x, y
    else:
        return False, False, False

distance = lambda x, y: math.sqrt(x * x + y * y)

formatAngle = lambda x: x + 2 * math.pi if x < -math.pi else x - 2 * math.pi if x > math.pi else x

def calcDriftArc(current, p1, p2):
    x_remain = 0
    x_total = 1
    y_drift = 0
    phi_drift = 0

    if (p1.phi == 0 or p1.phi == math.pi):
        dx = 0
        dy = 1
    else:
        dx = 1
        dy = math.tan(p1.phi + math.pi / 2)
    # l1 = Line2D(Point2D(p1.x, p1.y), Point2D(p1.x+dx, p1.y+dy))
    l1 = line([p1.x, p1.y], [p1.x + dx, p1.y + dy])

    if (p2.phi == 0 or p2.phi == math.pi):
        dx = 0
        dy = 1
    else:
        dx = 1
        dy = math.tan(p2.phi + math.pi / 2)
    # l2 = Line2D(Point2D(p2.x, p2.y), Point2D(p2.x+dx, p2.y+dy))
    l2 = line([p2.x, p2.y], [p2.x + dx, p2.y + dy])

    # center = Line2D.intersection(l1, l2)  #performence too low
    flag, cx, cy = intersection(l1, l2)
    if flag == False:
        return [False,False,False,False,False,False]
    # print("A",time.time()-start)
    # center = Point2D(center[0],center[1])
    # center = Point2D(cx,cy)
    # print("B",time.time()-start)
    # center = center[0]

    # print(time.time()-start)

    # r = center.distance(Point2D(p2.x, p2.y))   #this performence is too low
    # rv = center.distance(Point2D(current.x, current.y))  #this performence is too low
    r = distance(cx - p2.x, cy - p2.y)
    rv = distance(cx - current.x, cy - current.y)

    a1 = np.angle(float(p1.x - cx) + float(p1.y - cy) * 1j)
    a2 = np.angle(float(p2.x - cx) + float(p2.y - cy) * 1j)
    av = np.angle(float(current.x - cx) + float(current.y - cy) * 1j)

    a1_2 = formatAngle(a2 - a1)
    a1_v = formatAngle(av - a1)

    if a1_2 > 0:
        y_drift = float(r - rv)
    else:
        y_drift = float(rv - r)

    # print(time.time()-start)
    a_phi2_a2 = formatAngle(p2.phi - a2)
    forwardOrBackward = True if a_phi2_a2 > 0 else False
    forwardOrBackward = not forwardOrBackward if a1_2 < 0 else forwardOrBackward

    phi_correct = formatAngle(av + math.pi / 2)  # if a_phi2_a2 > 0 else formatAngle(av - math.pi / 2)
    phi_drift = formatAngle(current.phi - phi_correct)
    #print("ARC ",current.phi/math.pi*180,phi_correct/math.pi*180,phi_drift/math.pi*180)

    r = r if a1_2 > 0 else -r
    x_remain = r * formatAngle(a2 - av)
    # print(time.time()-start)
    # x_total= math.fabs(r * a1_2)
    x_total = r * a1_2
    if x_total < 0:
        x_total = -x_total

    # print(time.time()-start)
    return x_remain, x_total, y_drift, phi_drift, r, forwardOrBackward


def calcDriftLine(current, p1, p2):
    phi_correct = p2.phi  # confirm at xianghu, same as above
    phi_drift = formatAngle(current.phi - phi_correct)

    v = np.array([p2.x - p1.x, p2.y - p1.y])
    mod_v = math.sqrt(np.dot(v, v))
    vloc = np.array([current.x - p1.x, current.y - p1.y])
    try:
        x_done = np.dot(vloc, v) / mod_v
        y_drift = float(np.cross(v, vloc) / mod_v)
        x_remain = mod_v - x_done
    except:
        print("error in calcDriftLine: mod_v= 0")
    # x_remain = mod_v - x_done

    a_correct = np.angle(v[0] + v[1] * 1j)
    # y_drift = float(np.cross(v, vloc) / mod_v)
    if mod_v == 0:
        phi_drift = formatAngle(current.phi - p2.phi)
        y_drift = 0
        x_remain = 0

    forwardOrBackward = True if -math.pi / 2 < formatAngle(p2.phi - a_correct) < math.pi / 2 else False

    return x_remain, mod_v, y_drift, phi_drift, forwardOrBackward

def calcSteerAngle(current, y_drift, phi_drift, r, forwardOrBackward, speed, wheelBase):
    yaw = np.tan(phi_drift)
    ctrl = -0.6 * yaw - 1.2* y_drift - 1 * wheelBase * yaw
    # ctrl = -0.6 * yaw - 0.6 * y_drift - 1 * wheelBase * yaw
    if r != 0:
        ctrl = np.arctan(wheelBase / r)  +  np.arctan(ctrl) 
    else:
        ctrl = np.arctan(ctrl)
    ctrl = math.pi / 2 if ctrl > math.pi / 2 else -math.pi / 2 if ctrl < -math.pi / 2 else ctrl
    ctrl = ctrl if forwardOrBackward else -ctrl

    return ctrl


# a = Ponit(0, 0, 0)
# b = Ponit(10, 0, 0)
# current = Ponit(1, 0, 0)
# remain_path_len, total_path_len, lateral_drift, phi_drift, forwardOrBackward = calcDriftLine(current, a, b)
# print("总长度：{}，剩余路径：{}，横向偏差：{}，角度偏差：{}，方向：{}"\
#     .format(total_path_len, remain_path_len, lateral_drift, phi_drift, forwardOrBackward))

# a = Ponit(0, 0, 0)
# b = Ponit(1, 1,  math.pi/2)
# current = Ponit(-0.5, 0.5, 0)

# remain_path_len, total_path_len, lateral_drift, phi_drift, r, forwardOrBackward = calcDriftArc(current, a, b)
# print("总长度：{}，剩余路径：{}，横向偏差：{}，角度偏差：{}，半径：{}，方向：{}"\
#     .format(total_path_len, remain_path_len, lateral_drift, phi_drift, r, forwardOrBackward))

# print(calcSteerAngle(current, lateral_drift, phi_drift, r, forwardOrBackward, 0, 2))
