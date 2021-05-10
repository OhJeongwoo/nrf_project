from message import Message
from lane import Lane
from obstacle import Obstacle
import utils

class Data:
    def __init__(self, seq = None, time = None, messages = None):
        if messages:
            self.seq = seq
            self.time = time
            self.size = len(messages)
            self.messages = messages
        self.left_lane = Lane()
        self.right_lane = Lane()
        self.n_next_lanes = 0
        self.next_lanes = []
        self.n_obstacles = 0
        self.obstacles = []
        self.valid = True
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.theta = 0.0
        self.omega = 0.0


    def add_vehicle_state(self, x, y, v, ax, ay, theta, omega):
        self.x = x
        self.y = y
        self.v = v
        self.ax = ax
        self.ay = ay
        self.theta = theta
        self.omega = omega

    def find_message(self, target):
        for msg in self.messages:
            if msg.type == target:
                return msg
        self.valid = False
        return None

    def decode_data(self):
        # decode left lane
        LaneA = self.find_message(utils.LEFT_LANE_A)
        LaneB = self.find_message(utils.LEFT_LANE_B)
        if LaneA is None or LaneB is None:
            self.valid = utils.print_error_message(self.seq, "left_lane")
        else:
            self.left_lane = utils.decode_lane(LaneA, LaneB)
           
        # decode right lane
        LaneA = self.find_message(utils.RIGHT_LANE_A)
        LaneB = self.find_message(utils.RIGHT_LANE_B)
        if LaneA is None or LaneB is None:
            self.valid = utils.print_error_message(self.seq, "right_lane")
        else:
            self.right_lane = utils.decode_lane(LaneA, LaneB)

        # decode next lane
        NextLaneInfo = self.find_message(utils.NEXT_LANE_INFO)
        if NextLaneInfo is None:
            self.valid = utils.print_error_message(self.seq, "next lane info")
        else:
            self.n_next_lanes = utils.decode_next_lane_info(NextLaneInfo)
        
        for i in range(self.n_next_lanes):
            LaneA = self.find_message(utils.NEXT_LANE_A + 2 * i)
            LaneB = self.find_message(utils.NEXT_LANE_B + 2 * i)
            if LaneA is None or LaneB is None:
                self.valid = utils.print_error_message(self.seq, str(i) + "-th next lane")
            else:
                self.next_lanes.append(utils.decode_lane(LaneA, LaneB))
        
        # decode obstacle
        ObstacleInfo = self.find_message(utils.OBSTACLE_INFO)
        if ObstacleInfo is None:
            self.valid = utils.print_error_message(self.seq, "obstacle info")
        else:
            self.n_obstacles = utils.decode_obstacle_info(ObstacleInfo)
        
        for i in range(self.n_obstacles):
            ObstacleA = self.find_message(utils.OBSTACLE_A + 3 * i)
            ObstacleB = self.find_message(utils.OBSTACLE_B + 3 * i)
            ObstacleC = self.find_message(utils.OBSTACLE_C + 3 * i)
            if ObstacleA is None or ObstacleB is None or ObstacleC is None:
                self.valid = utils.print_error_message(self.seq, str(i) + "-th obstacle")
            else :
                self.obstacles.append(utils.decode_obstacle(ObstacleA, ObstacleB, ObstacleC))

        print("Success to decode %d-th data" %(self.seq))
        return self.valid

    def get_data(self):
        rt = {}

        rt['seq'] = self.seq
        rt['time'] = self.time
        rt['lane'] = {'left_lane': self.left_lane.get_lane(),
                      'right_lane': self.right_lane.get_lane(),
                      'next_lane': {'size': self.n_next_lanes,
                                    'next_lanes': [lane.get_lane() for lane in self.next_lanes]}}
        rt['obstacle'] = {'size': self.n_obstacles,
                          'obstacles': [obstacle.get_obstacle() for obstacle in self.obstacles]}
        rt['vehicle_state'] = {'x': self.x,
                               'y': self.y,
                               'v': self.v,
                               'ax': self.ax,
                               'ay': self.ay,
                               'theta': self.theta,
                               'omega': self.omega}
        
        return rt