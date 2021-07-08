import math

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
        print("# of convexpoints :",n)
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
    
    def is_include(self, point):
        value = ccw(point, self.points[self.n_points-1], self.points[0])
        for i in range(1, self.n_points):
            if value * ccw(point, self.points[i-1], self.points[i]) < 0:
                return False
        return True


def rotation(x, y, theta):
    ct = math.cos(theta)
    st = math.sin(theta)
    return x * ct - y * st, x * st + y * ct


def box_convex_hull(x,y,theta,l,w):
    pts = []
    px, py = rotation(l/2, w/2, theta)
    pts.append(Point(x+px, y+py))
    t = math.atan((y+py)/(x+px))
    px, py = rotation(l/2, -w/2, theta)
    pts.append(Point(x+px, y+py))
    px, py = rotation(-l/2, w/2, theta)
    pts.append(Point(x+px, y+py))
    px, py = rotation(-l/2, -w/2, theta)
    pts.append(Point(x+px, y+py))
    
    return ConvexHull(pts)

grid_size = 500
def calculate_IOU(A, B):
    """
    calculate IOU between two Convex Hull
    """
    minX = 1e6
    maxX = -1e6
    minY = 1e6
    maxY = -1e6
    for point in A.points:
        if point.x < minX:
            minX = point.x
        if point.x > maxX:
            maxX = point.x
        if point.y < minY:
            minY = point.y
        if point.y > maxY:
            maxY = point.y
    
    n_A = 0
    n_B = 0
    n_AB = 0
    for i in range(grid_size):
        for j in range(grid_size):
            x = minX + i / grid_size * (maxX - minX)
            y = minY + j / grid_size * (maxY - minY)
            p = Point(x,y)
            a = A.is_include(p)
            b = B.is_include(p)
            if a:
                n_A += 1
            if b:
                n_B += 1
            if a and b:
                n_AB += 1
    return 1.0 * n_AB / (n_A + n_B - n_AB)


# demo = []
# demo.append(Point(1,0))
# demo.append(Point(2,0))
# demo.append(Point(1,1))
# demo.append(Point(2,1))
# demo.append(Point(2,2))

# cvh = ConvexHull(demo)
# print(cvh.get_center())