import math

EPS = 1e-6
INF = 1e9

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

class Line:
    def __init__(self, s, e):
        self.s = s
        self.e = e

    def length(self):
        return math.sqrt(dist2(self.s,self.e))

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

def area(a, b):
    return 0.5 * abs(a.y * b.x - a.x * b.y)

def vec(a,b):
    return Point(b.x - a.x, b.y - a.y)


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

def dist2(A, B):
    return (A.x- B.x) ** 2 + (A.y - B.y) ** 2

def valid(p, s, e):
    d = math.sqrt(dist2(s, e))
    x = math.sqrt(dist2(p, s))
    y = math.sqrt(dist2(p, e))
    if d + x < y or d + y < x :
        return False
    return True

def line_distance(l1, l2):
    rt = INF
    rt = min(rt, dist2(l1.s, l2.s))
    rt = min(rt, dist2(l1.s, l2.e))
    rt = min(rt, dist2(l1.e, l2.s))
    rt = min(rt, dist2(l1.e, l2.e))

    rt = math.sqrt(rt)

    if valid(l1.s, l2.s, l2.e):
        rt = min(rt, 2.0 * area(vec(l1.s, l2.s), vec(l1.s, l2.e)) / l2.length())
    
    if valid(l1.e, l2.s, l2.e):
        rt = min(rt, 2.0 * area(vec(l1.e, l2.s), vec(l1.e, l2.e)) / l2.length())
    
    if valid(l2.s, l1.s, l1.e):
        rt = min(rt, 2.0 * area(vec(l2.s, l1.s), vec(l2.s, l1.e)) / l1.length())

    if valid(l2.e, l1.s, l1.e):
        rt = min(rt, 2.0 * area(vec(l2.e, l1.s), vec(l2.e, l1.e)) / l1.length())

    return rt

def get_clearance(A, B):
    a = A.n_points
    b = B.n_points
    min_a = -1
    min_b = -1
    rt = INF
    for i in range(a):
        for j in range(b):
            d = math.sqrt(dist2(A.points[i], B.points[j]))
            if rt > d:
                rt = d
                min_a = i
                min_b = j
    rt = min(rt, line_distance(Line(A.points[(min_a-1+a)%a], A.points[min_a]), Line(B.points[(min_b-1+b)%b],B.points[min_b])));
    rt = min(rt, line_distance(Line(A.points[(min_a-1+a)%a], A.points[min_a]), Line(B.points[(min_b+1+b)%b],B.points[min_b])));
    rt = min(rt, line_distance(Line(A.points[(min_a+1+a)%a], A.points[min_a]), Line(B.points[(min_b-1+b)%b],B.points[min_b])));
    rt = min(rt, line_distance(Line(A.points[(min_a+1+a)%a], A.points[min_a]), Line(B.points[(min_b+1+b)%b],B.points[min_b])));

    return rt

def merge_convex_hull(cvh_list):
    pts = []
    for cvh in cvh_list:
        pts += cvh.points
    return ConvexHull(pts)

# demo = []
# demo.append(Point(1,0))
# demo.append(Point(2,0))
# demo.append(Point(1,1))
# demo.append(Point(2,1))
# demo.append(Point(2,2))

# cvh = ConvexHull(demo)
# print(cvh.get_center())