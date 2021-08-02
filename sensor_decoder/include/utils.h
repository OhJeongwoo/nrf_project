#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <ctime>
#include <tuple>
#include <vector>
#include <stack>
#include "ros/ros.h"

using namespace std;

typedef pair<double, int> pdi;

const double INF = 1e9;
const double EPS = 1e-6;


inline double norm(double x, double y){
    return sqrt(pow(x,2) + pow(y,2));
}

inline void as_unit_vector(tuple<double, double>& vec){
    double magnitude = norm(get<0>(vec), get<1>(vec));
    if (magnitude > 0){
        get<0>(vec) = get<0>(vec) / magnitude;
        get<1>(vec) = get<1>(vec) / magnitude;
    }
}

inline double dot(const tuple<double, double>& vec1, const tuple<double, double>& vec2){
    return get<0>(vec1) * get<0>(vec2) + get<1>(vec1) * get<1>(vec2);
}


struct point{
    double x,y;
    double theta;
    bool valid;
    bool occluded;
    point() : point(0,0){}
    point(double x1, double y1): x(x1),y(y1), theta(0){}
    void update(point p){
        theta = atan2(y-p.y,x-p.x);
    }
    bool operator <(const point& o){
        if(abs(theta-o.theta)>EPS) return theta<o.theta;
        if(abs(y-o.y)>EPS) return y<o.y;
        return x<o.x;
    }
    point operator +(const point &o) const{return point(x+o.x, y+o.y);}
    point operator -(const point &o) const{return point(x-o.x, y-o.y);}
    point operator -() const{return point(-x, -y);}
    double operator * (const point &o) const{ return x*o.x + y*o.y;}
    point operator *(const double t) const{return point(x*t, y*t);}
};

struct line{
    point s,e;
    line(): line(0,0,0,0){}
    line(double x1, double y1, double x2, double y2):s(x1,y1),e(x2,y2){}
    line(point p1, point p2):s(p1),e(p2){}
};

inline int ccw(const point &a, const point &b, const point &c){
    double rt = (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
    if(abs(rt)<EPS)return 0;
    if(rt<0) return -1;
    return 1;
}

inline bool intersect(const line &l1, const line &l2){
    int ab = ccw(l1.s,l1.e, l2.s)*ccw(l1.s,l1.e,l2.e);
    int cd = ccw(l2.s,l2.e,l1.s)*ccw(l2.s,l2.e,l1.e);

    if(ab==0 && cd ==0){
        if((l1.s.x>l2.s.x && l1.s.x>l2.e.x&&l1.e.x>l2.s.x&&l1.e.x>l2.e.x) ||(l1.s.x<l2.s.x && l1.s.x<l2.e.x&&l1.e.x<l2.s.x&&l1.e.x<l2.e.x)) return false;
        if((l1.s.y>l2.s.y && l1.s.y>l2.e.y&&l1.e.y>l2.s.y&&l1.e.y>l2.e.y) ||(l1.s.y<l2.s.y && l1.s.y<l2.e.y&&l1.e.y<l2.s.y&&l1.e.y<l2.e.y)) return false;
    }
    return (ab<=0)&&(cd<=0);
}

inline double sq(double x){ return x*x;}

inline double dist(const point &a, const point &b){return sqrt(sq(a.x-b.x)+sq(a.y-b.y));}
inline double dist2(const point &a, const point &b){return sq(a.x-b.x)+sq(a.y-b.y);}

inline double area(const point &a, const point &b){
    return 0.5*abs(a.y*b.x-a.x*b.y);
}

inline point vec(const point &s, const point &e){
    return point(e.x-s.x,e.y-s.y);
}

inline double length(const line &l){
    return dist(l.s,l.e);
}

inline bool valid(const point &p, const point &s, const point &e){ //수선의 발이 line(s,e)위에 있는지 확인
    double d = sq(dist(s,e));
    double x = sq(dist(p,s));
    double y = sq(dist(p,e));
    if(d+x<y||d+y<x) return false;
    return true;
}

inline bool valid2(const point &p, const point &s, const point &e){ //수선의 발이 line(s,e)위에 있는지 확인
    double d = dist2(s,e);
    double x = dist2(p,s);
    double y = dist2(p,e);
    if(d+x<y||d+y<x) return false;
    return true;
}

inline double line_distance(const line &l1, const line &l2){
    double rt = INF;
    rt = min<double>(rt, dist(l1.s, l2.s));
    rt = min<double>(rt, dist(l1.s, l2.e));
    rt = min<double>(rt, dist(l1.e, l2.s));
    rt = min<double>(rt, dist(l1.e, l2.e));

    if(valid(l1.s, l2.s, l2.e)) rt = min<double>(rt, 2.0*area(vec(l1.s,l2.s),vec(l1.s, l2.e))/length(l2));
    if(valid(l1.e, l2.s, l2.e)) rt = min<double>(rt, 2.0*area(vec(l1.e,l2.s),vec(l1.e, l2.e))/length(l2));
    if(valid(l2.s, l1.s, l1.e)) rt = min<double>(rt, 2.0*area(vec(l2.s,l1.s),vec(l2.s, l1.e))/length(l1));
    if(valid(l2.e, l1.s, l1.e)) rt = min<double>(rt, 2.0*area(vec(l2.e,l1.s),vec(l2.e, l1.e))/length(l1));

    return rt;
}



inline double distance_to_segment(double x, double y, double x1, double y1, double x2, double y2){
    point p = point(x,y);
    line l = line(x1,y1,x2,y2);
    if(valid(p, l.s, l.e)) return 2.0*area(vec(p,l.s),vec(p,l.e))/length(l);
    return min<double>(dist(p,l.s),dist(p,l.e));
}

inline double getTriangleArea(const point &a, const point &b, const point &c){
    return 0.5*abs((a.x-b.x)*(a.y-c.y)-(a.y-b.y)*(a.x-c.x));
}


struct ConvexHull{
    public:
    vector<point> p;
    int size;
    bool valid;
    string id;
    point center;
    double radius;

    ConvexHull(){
        size = 0;
        valid = true;
    }
    ConvexHull(vector<point> v){
        valid = true;
        double cx = 0;
        double cy = 0;
        radius = 0.0;
        size = 0;
        if(v.size()<3) {
            p=v;
            center = point(cx, cy);
            ROS_ERROR("ConvexHull Error : few point");
        }
        sort(v.begin(), v.end());
        for(int i=1;i<v.size();i++) v[i].update(v[0]);
        sort(v.begin()+1, v.end());
        stack<int> s;
        s.push(0);
        s.push(1);
        int next = 2;
        while(next<v.size()){
            while(s.size()>=2){
                int first, second;
                first = s.top();
                s.pop();
                second = s.top();
                if(ccw(v[second],v[first],v[next])>0){
                    s.push(first);
                    break;
                }
            }
            s.push(next);
            next++;
        }
        while(!s.empty()){
            p.push_back(v[s.top()]);
            cx += v[s.top()].x;
            cy += v[s.top()].y;
            s.pop();
            size++;
        }
        cx /= size;
        cy /= size;
        center = point(cx,cy);
        for(int i=0;i<size;i++) radius = max<double>(radius, dist(center, p[i]));
    }

    ConvexHull(vector<point> v, string id){
        valid = true;
        double cx = 0;
        double cy = 0;
        radius = 0.0;
        size = 0;
        if(v.size()<3) {
            p=v;
            center = point(cx, cy);
            ROS_ERROR("ConvexHull Error : few point");
        }
        sort(v.begin(), v.end());
        for(int i=1;i<v.size();i++) v[i].update(v[0]);
        sort(v.begin()+1, v.end());
        stack<int> s;
        s.push(0);
        s.push(1);
        int next = 2;
        while(next<v.size()){
            while(s.size()>=2){
                int first, second;
                first = s.top();
                s.pop();
                second = s.top();
                if(ccw(v[second],v[first],v[next])>0){
                    s.push(first);
                    break;
                }
            }
            s.push(next);
            next++;
        }
        while(!s.empty()){
            p.push_back(v[s.top()]);
            cx += v[s.top()].x;
            cy += v[s.top()].y;
            s.pop();
            size++;
        }
        cx /= size;
        cy /= size;
        center = point(cx,cy);
        for(int i=0;i<size;i++) radius = max<double>(radius, dist(center, p[i]));
        
        this->id = id;
    }
    const void print_all() const{
        cout << "ConvexHull" <<endl;
        for(int i=0;i<size;i++){
            cout << p[i].x << " " << p[i].y << endl;
        }
        cout << endl;
        cout << endl;
        cout << endl;
    }

    const double getArea() const{
        double rt = 0.0;
        for(int i=1;i<p.size()-2;i++){
            rt += getTriangleArea(p[0],p[i],p[i+1]);
        }
        return rt;
    }

};

inline bool checkCollision(const ConvexHull &a, const ConvexHull &b){
    for(int i=0;i<a.size;i++){
        for(int j=0;j<b.size;j++){
            line l1 = line(a.p[(i%a.size)],a.p[((i+1)%a.size)]);
            line l2 = line(b.p[(j%b.size)],b.p[((j+1)%b.size)]);
            if(intersect(l1,l2)) return true;
        }
    }

    bool check = true;
    point p = b.p[0];
    int val = ccw(p,a.p[a.size-1],a.p[0]);
    for(int i=0;i<a.size-1;i++){
        if(ccw(p,a.p[i],a.p[i+1])!=val) {
            check = false;
            break;
        }
    }
    if(check) return true;
    check = true;
    p=a.p[0];
    val = ccw(p,b.p[b.size-1],b.p[0]);
    for(int i=0;i<b.size-1;i++){
        if(ccw(p,b.p[i],b.p[i+1])){
            check = false;
            break;
        }
    }
    if(check) return true;
    return false;
}

inline bool checkCollision_test(const ConvexHull &A, const ConvexHull &B){
    double rt = INF;
    int a = A.size;
    int b = B.size;

    line l1;
    line l2;
    l1 = line(A.p[a-1], A.p[0]);
    int id = -1;
    for(int i=0;i<b;i++){
        l2 = line(B.p[i],B.p[(i+1)%b]);
        double d = line_distance(l1, l2);
        if(rt>d){
            rt =d;
            id = i;
        }
    }
    for(int i=0;i<a-1;i++){
        l1 = line(A.p[i], A.p[i+1]);
        l2 = line(B.p[id], B.p[(id+1)%b]);
        double local_min = line_distance(l1, l2);
        bool inc = true;
        if(line_distance(l1, line(B.p[(id-1+b)%b],B.p[id])) < line_distance(l1, line(B.p[id],B.p[(id+1)%b]))) inc = false;
        if(inc){
            while(1){
                int tmp = (id+1)%b;
                double d = line_distance(l1, line(B.p[tmp],B.p[(tmp+1)%b]));
                if(d> local_min) break;
                local_min = d;
                id = tmp;
            }
        }
        else{
            while(1){
                int tmp = (id-1+b)%b;
                double d = line_distance(l1, line(B.p[tmp], B.p[(tmp+1)%b]));
                if(d>local_min) break;
                local_min =d;
                id = tmp;
            }
        }
        rt = min<double>(rt, local_min);
    }
    if(rt<EPS) return true;
    bool check = true;
    point p = B.p[0];
    int val = ccw(p,A.p[a-1],A.p[0]);
    for(int i=0;i<a-1;i++){
        if(ccw(p,A.p[i],A.p[i+1])!=val) {
            check = false;
            break;
        }
    }
    if(check) return true;
    check = true;
    p=A.p[0];
    val = ccw(p,B.p[b-1],B.p[0]);
    for(int i=0;i<b-1;i++){
        if(ccw(p,B.p[i],B.p[i+1])){
            check = false;
            break;
        }
    }
    if(check) return true;
    return false;
}

inline ConvexHull UnionConvexHull(const ConvexHull &a, const ConvexHull &b){
    vector<point> np;
    for(point p : a.p) np.push_back(p);
    for(point p : b.p) np.push_back(p);

    return ConvexHull(np);
}

inline ConvexHull UnionConvexHull(const vector<ConvexHull>& v){
    vector<point> p;
    for(ConvexHull cvh : v) p.insert(p.end(), cvh.p.begin(), cvh.p.end());
    return ConvexHull(p);
}

inline bool isIncludeConvexHull(const ConvexHull &c, const point &v){
    bool rt = true;
    if(dist(c.center, v)>c.radius) false;
    int val = ccw(v,c.p[c.size-1],c.p[0]);
    for(int i=0;i<c.size-1;i++){
        if(ccw(v,c.p[i],c.p[i+1])!=val) {
            rt = false;
            break;
        }
    }
    return rt;
}

struct BoundingBox{
    public:
    double x,y,theta,l,w;
    double loss_threshold = 1e-3;
    int maxstep = 100;
    double lr_init = 3e-2;
    double decay_k = 5e-2;
    double l_coeff = 0.1;
    double w_coeff = 0.1;
    double box_coeff = 1.0;
    double cvh_coeff = 1.0;
    double outer_box_coeff = 1.0;
    double l_g = 5.0;
    double w_g = 2.5;
    double box_loss = 0.0;
    double loss = 0.0;
    BoundingBox() : BoundingBox(0,0,0,0,0){}
    BoundingBox(double x, double y, double theta, double l, double w): x(x), y(y), theta(theta), l(l), w(w){}

    bool operator <(const BoundingBox& b){
        return abs(0.5*x) + abs(y) < abs(0.5 * b.x) + abs(b.y);
    }

    void optimize(const vector<point> &points, double cvh_dist){
        double prev_loss;
        for(int iter = 0; iter < maxstep; iter++){
            theta = theta - M_PI * 2 * int(theta / M_PI / 2);
            if (theta < - M_PI) theta += M_PI;
            if (theta > M_PI) theta += M_PI;
            loss = 0.0;
            double grad_x = 0.0;
            double grad_y = 0.0;
            double grad_theta = 0.0;
            double grad_l = 0.0;
            double grad_w = 0.0;
            double lr = lr_init / (1+decay_k*iter);
            
            loss += l_coeff * (l - l_g) * (l - l_g);
            grad_l += l_coeff * 2 * (l - l_g);

            loss += w_coeff * (w - w_g) * (w - w_g);
            grad_w += w_coeff * 2 * (w - w_g);

            double total_box_loss = 0.0;
            double total_box_grad_x = 0.0;
            double total_box_grad_y = 0.0;
            double total_box_grad_theta = 0.0;
            double total_box_grad_l = 0.0;
            double total_box_grad_w = 0.0;


            int N = 0;
            for(const point& p : points){
                // if(p.occluded) continue;
                N ++;
                vector<double> rt = calculate_box_loss(p);
            
                total_box_loss += rt[0];
                total_box_grad_x += rt[1];
                total_box_grad_y += rt[2];
                total_box_grad_theta += rt[3];
                total_box_grad_l += rt[4];
                total_box_grad_w += rt[5];
            }

            if(N>0){
                total_box_loss /= N;
                total_box_grad_x /= N;
                total_box_grad_y /= N;
                total_box_grad_theta /= N;
                total_box_grad_l /= N;
                total_box_grad_w /= N;
            }

            box_loss = total_box_loss;
            loss += box_coeff * total_box_loss;
            grad_x += box_coeff * total_box_grad_x;
            grad_y += box_coeff * total_box_grad_y;
            grad_theta += box_coeff * total_box_grad_theta;
            grad_l += box_coeff * total_box_grad_l;
            grad_w += box_coeff * total_box_grad_w;

            vector<double> cvh_loss = calculate_box_loss(point());
            double d = sqrt(cvh_loss[0]);
            if (d > 1e-3){
                double f = (d - cvh_dist) / d;
                loss += cvh_coeff * f * f * cvh_loss[0];
                grad_x += cvh_coeff * f * cvh_loss[1];
                grad_y += cvh_coeff * f * cvh_loss[2];
                grad_theta += cvh_coeff * f * cvh_loss[3];
                grad_l += cvh_coeff * f * cvh_loss[4];
                grad_w += cvh_coeff * f * cvh_loss[5];
            }

            // update parameters
            x -= lr * grad_x;
            y -= lr * grad_y;
            theta -= lr * grad_theta;
            l -= lr * grad_l;
            w -= lr * grad_w;
            

            if(iter > 0 && abs(loss - prev_loss) < loss_threshold) break;
            prev_loss = loss;
        }
        
    }

    vector<double> calculate_loss_and_grad(int type, double px, double py, double rx, double ry, double st, double ct){
        vector<double> values;
        values.push_back(abs(rx-l/2));
        values.push_back(abs(rx+l/2));
        values.push_back(abs(ry-w/2));
        values.push_back(abs(ry+w/2));
        vector<double> rt;
            
        if (type == 0){
            rt.push_back(values[type] * values[type]);
            rt.push_back(2.0 * (l/2 - rx) * ct);
            rt.push_back(2.0 * (l/2 - rx) * st);
            rt.push_back(-2.0 * (l/2 - rx) * ((px - x) * (-st) + (py - y) * ct));
            rt.push_back(l/2 - rx);
            rt.push_back(0);
            return rt;
        }
        if (type == 1){
            rt.push_back(values[type] * values[type]);
            rt.push_back(-2.0 * (l/2 + rx) * ct);
            rt.push_back(-2.0 * (l/2 + rx) * st);
            rt.push_back(2.0 * (l/2 + rx) * ((px - x) * (-st) + (py - y) * ct));
            rt.push_back(l/2 + rx);
            rt.push_back(0);
            return rt;
        }
        if (type == 2){
            rt.push_back(values[type] * values[type]);
            rt.push_back(-2.0 * (w/2 - ry) * st);
            rt.push_back(2.0 * (w/2 - ry) * ct);
            rt.push_back(2.0 * (w/2 - ry) * ((px - x) * ct + (py - y) * st));
            rt.push_back(0);
            rt.push_back(w/2 - ry);
            return rt;
        }
        if (type == 3){
            rt.push_back(values[type] * values[type]);
            rt.push_back(2.0 * (w/2 + ry) * st);
            rt.push_back(-2.0 * (w/2 + ry) * ct);
            rt.push_back(-2.0 * (w/2 + ry) * ((px - x) * ct + (py - y) * st));
            rt.push_back(0);
            rt.push_back(w/2 + ry);
            return rt;
        }   
    }

    vector<double> calculate_box_loss(const point& p){
        double px = p.x;
        double py = p.y;
        double ct = cos(theta);
        double st = sin(theta);
        double rx = (px - x) * ct + (py - y) * st;
        double ry = - (px - x) * st + (py - y) * ct;
            

        if (abs(rx)<l/2 && abs(ry) <w/2){
            vector<double> values;
            values.push_back(abs(rx-l/2));
            values.push_back(abs(rx+l/2));
            values.push_back(abs(ry-w/2));
            values.push_back(abs(ry+w/2));
            int type = 0;
            double value = values[0];
            for(int i=1;i<=3;i++){
                if(values[i] < value){
                    type = i;
                    value = values[i];
                }
            }
            return calculate_loss_and_grad(type, px, py, rx, ry, st, ct);
        }
        else{
            double rt_loss = 0.0;
            double rt_grad_x = 0.0;
            double rt_grad_y = 0.0;
            double rt_grad_theta = 0.0;
            double rt_grad_l = 0.0;
            double rt_grad_w = 0.0;
            if(rx>l/2){
                vector<double> rt = calculate_loss_and_grad(0, px, py, rx, ry, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_theta += rt[3];
                rt_grad_l += rt[4];
                rt_grad_w += rt[5];
            }
            if(rx<-l/2){
                vector<double> rt = calculate_loss_and_grad(1, px, py, rx, ry, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_theta += rt[3];
                rt_grad_l += rt[4];
                rt_grad_w += rt[5];
            }
            if(ry>w/2){
                vector<double> rt = calculate_loss_and_grad(2, px, py, rx, ry, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_theta += rt[3];
                rt_grad_l += rt[4];
                rt_grad_w += rt[5];
            }
            if(ry<-w/2){
                vector<double> rt = calculate_loss_and_grad(3, px, py, rx, ry, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_theta += rt[3];
                rt_grad_l += rt[4];
                rt_grad_w += rt[5];
            }
            vector<double> final_rt;
            final_rt.push_back(outer_box_coeff * rt_loss);
            final_rt.push_back(outer_box_coeff * rt_grad_x);
            final_rt.push_back(outer_box_coeff * rt_grad_y);
            final_rt.push_back(outer_box_coeff * rt_grad_theta);
            final_rt.push_back(outer_box_coeff * rt_grad_l);
            final_rt.push_back(outer_box_coeff * rt_grad_w);
            return final_rt;
        }
    }
};

double lengthSquared (const point & v);

point tripleProduct (const point & a, const point & b, const point & c);
point averagePoint(const vector<point> & vertices);
int indexOfFurthestPoint(const vector<point> & vertices, const point & d);
point support(const vector<point> & vertices1, const vector<point> & vertices2, const point & d);
point closest_point_to_origin(const point & a);
point closest_point_to_origin(const point & a, const point & b);
point closest_point_to_origin(const point & a, const point & b, const point & c);
double gjk(const vector<point>& vertices1, const vector<point>& vertices2);

#endif
