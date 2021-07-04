EPS = 1e-6

class Point:
    def __init__(self, x,y,p=1.0,q=0.0):
        self.x = x
        self.y = y
        self.p = p
        self.q = q
    
    def __gt__(self, other):
        if abs(self.q * other.p - self.p * other.q) > EPS:
            return self.q * other.p > self.p * other.q
        if abs(self.y - other.y) > EPS:
            return self.y > other.y
        return self.x > other.x

def ccw(a, b, c):
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)

class ConvexHull:
    def __init__(self, points):
        n = len(points)
        sorted(points)
        for i in range(1,n):
            points[i].p = points[i].x - points[0].x
            points[i].q = points[i].y - points[0].y
        points[1:n] = sorted(points[1:n])
        index = []
        index.append(0)
        index.append(1)
        next = 2
        while next < n:
            while len(index) >= 2:
                first = index[len(index)-1]
                index.pop()
                second = index[len(index)-1]
                if ccw(points[second], points[first], points[next]) > EPS:
                    index.append(first)
                    break
            index.append(next)
            next += 1

        self.points = []
        for i in range(len(index)):
            self.points.append(points[index[i]])
        self.n_points = len(self.points)
        

    def get_center(self):
        sum_x = 0.0
        sum_y = 0.0
        for point in self.points:
            sum_x += point.x
            sum_y += point.y
        return sum_x / self.n_points, sum_y / self.n_points

# demo = []
# demo.append(Point(1,0))
# demo.append(Point(2,0))
# demo.append(Point(1,1))
# demo.append(Point(2,1))
# demo.append(Point(2,2))

# cvh = ConvexHull(demo)
# print(cvh.get_center())