from message import Message
from lane import Lane
from obstacle import Obstacle

INIT = 0x700
END = 0x728
LEFT_LANE_A = 0x766
LEFT_LANE_B = 0x767
RIGHT_LANE_A = 0x768
RIGHT_LANE_B = 0x769
NEXT_LANE_INFO = 0x76b
NEXT_LANE_A = 0x76c
NEXT_LANE_B = 0x76d
OBSTACLE_INFO = 0x738
OBSTACLE_A = 0x739
OBSTACLE_B = 0x73a
OBSTACLE_C = 0x73b

def sign_extend(value, bits):
    sign_bit = 1 << (bits - 1)
    return (value & (sign_bit - 1)) - (value & sign_bit)

def get_signal(msg, offset, size):
    signal = 0
    for pos in range(size - 1, -1, -1):
        row = (offset + pos) / 8
        col = (offset + pos) % 8
        signal = signal | msg.data[row][col]

        if pos > 0:
            signal = signal << 1
    
    return signal

def print_error_message(seq, target):
    print("%d-th data does not have message about %s" %(seq, target))
    return False

def decode_lane(LaneA, LaneB):
    rt = Lane()

    signal = get_signal(msg=LaneA, offset=4, size=2)
    rt.quality = signal

    signal = get_signal(msg=LaneA, offset=0, size=4)
    rt.lane_type = signal

    signal = get_signal(msg=LaneA, offset=56, size=8)
    rt.marking_width = float(signal) * 0.01

    signal = get_signal(msg=LaneA, offset=8, size=16)
    rt.c0 = float(sign_extend(signal, 16)) * 0.00390625

    signal = get_signal(msg=LaneA, offset=24, size=16)
    rt.c2 = float(signal) * 9.7656e-7 - 0.0319989415

    signal = get_signal(msg=LaneA, offset=40, size=16)
    rt.c3 = float(signal) * 3.7252903e-9 - 0.00012206659

    signal = get_signal(msg=LaneB, offset=0, size=16)
    rt.c1 = float(signal) * 0.00097656 - 31.9990234

    signal = get_signal(msg=LaneB, offset=16, size=15)
    rt.view_range = float(signal) * 0.00390625

    signal = get_signal(msg=LaneB, offset=31, size=1)
    rt.view_range_availability = True if signal == 1 else False

    return rt

def decode_next_lane_info(msg):
    signal = get_signal(msg=msg, offset=0, size=8)
    return signal

def decode_obstacle_info(msg):
    signal =  get_signal(msg=msg, offset=0, size=8)
    return signal

def decode_obstacle(ObstacleA, ObstacleB, ObstacleC):
    rt = Obstacle()

    signal = get_signal(msg=ObstacleA, offset=0, size=8)
    rt.id = signal

    signal = get_signal(msg=ObstacleA, offset=52, size=3)
    rt.type = signal
    
    signal = get_signal(msg=ObstacleA, offset=56, size=3)
    rt.status = signal

    signal = get_signal(msg=ObstacleB, offset=16, size=8)
    rt.age = signal

    signal = get_signal(msg=ObstacleB, offset=8, size=8)
    rt.width = float(signal) * 0.05

    signal = get_signal(msg=ObstacleB, offset=0, size=8)
    rt.length = float(signal) * 0.5

    signal = get_signal(msg=ObstacleA, offset=8, size=12)
    rt.x = float(signal) * 0.0625

    signal = get_signal(msg=ObstacleA, offset=24, size=10)
    rt.y = float(sign_extend(signal, 10)) * 0.0625

    signal = get_signal(msg=ObstacleA, offset=40, size=12)
    rt.vel = float(sign_extend(signal, 12)) * 0.0625

    signal = get_signal(msg=ObstacleC, offset=32, size=10)
    rt.accel = float(sign_extend(signal, 10)) * 0.03

    signal = get_signal(msg=ObstacleC, offset=48, size=16)
    rt.theta = float(sign_extend(signal, 16)) * 0.01

    signal = get_signal(msg=ObstacleC, offset=0, size=16)
    rt.omega = float(sign_extend(signal, 16)) * 0.01

    signal = get_signal(msg=ObstacleC, offset=16, size=16)
    rt.scale_rate = float(sign_extend(signal, 16)) * 0.0002

    return rt