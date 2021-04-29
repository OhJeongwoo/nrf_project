from nrf_project.msg import LaneMsg

class Lane:
    def __init__(self):
        self.quality = 0
        self.lane_type = 0
        self.c0 = 0.0
        self.c1 = 0.0
        self.c2 = 0.0
        self.c3 = 0.0
        self.marking_width = 0.0
        self.view_range = 0.0
        self.view_range_availability = False

    def get_lane(self):
        rt = {}
        
        rt['lane_type'] = self.lane_type
        rt['quality'] = self.quality
        rt['c0'] = self.c0
        rt['c1'] = self.c1
        rt['c2'] = self.c2
        rt['c3'] = self.c3
        rt['marking_width'] = self.marking_width
        rt['view_range'] = self.view_range
        rt['view_range_availability'] = self.view_range_availability

        return rt

    def generate_message(self):
        rt = LaneMsg()

        rt.quality = self.quality
        rt.lane_type = self.lane_type
        rt.c = [self.c0, self.c1, self.c2, self.c3]
        rt.marking_width = self.marking_width
        rt.view_range = self.view_range
        rt.view_range_availability = self.view_range_availability

        return rt