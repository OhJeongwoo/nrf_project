from sensor_decoder.msg import ObstacleMsg

class Obstacle:
    def __init__(self):
        self.id = 0
        self.type = 0
        self.status = 0
        self.age = 0
        self.width = 0.0
        self.length = 0.0
        self.x = 0.0
        self.y = 0.0
        self.vel = 0.0
        self.accel = 0.0
        self.theta = 0.0
        self.omega = 0.0
        self.scale_rate = 0.0
    
    def get_obstacle(self):
        rt = {}

        rt['id'] = self.id
        rt['type'] = self.type
        rt['status'] = self.status
        rt['age'] = self.age
        rt['width'] = self.width
        rt['length'] = self.length
        rt['x'] = self.x
        rt['y'] = self.y
        rt['vel'] = self.vel
        rt['accel'] = self.accel
        rt['theta'] = self.theta
        rt['omega'] = self.omega
        rt['scale_rate'] = self.scale_rate

        return rt

    def generate_message(self):
        rt = ObstacleMsg()

        rt.id = self.id
        rt.type = self.type
        rt.status = self.status
        rt.age = self.age
        rt.width = self.width
        rt.length = self.length
        rt.x = self.x
        rt.y = self.y
        rt.vel = self.vel
        rt.accel = self.accel
        rt.theta = self.theta
        rt.omega = self.omega
        rt.scale_rate =self.scale_rate
        
        return rt