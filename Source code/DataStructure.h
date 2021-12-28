// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

class Point;
class Edge;
class Line;

class Point
{
public:
    Point(const int p_x = 0, const int p_y = 0);

    // Operators overloading.
    inline Point operator+(const Point &b) {return Point(x + b.x, y + b.y);}
    inline Point operator-(const Point &b) {return Point(x - b.x, y - b.y);}
    inline Point operator*(const int a) {return Point(a * x, a * y);}
    inline Point operator/(const int a) {return Point(x / a, y / a);}
    inline int operator==(Point &b) {return x == b.x && y == b.y;}
    inline int operator!=(Point &b) {return x != b.x || y != b.y;}
    inline int operator>(Point &b) {return x > b.x && y > b.y;}
    inline int operator>=(Point &b) {return x >= b.x && y >= b.y;}
    inline int operator<(Point &b) {return x < b.x && y < b.y;}
    inline int operator<=(Point &b) {return x <= b.x && y <= b.y;}

    // Calculate function
    inline Point calSlope() {return calSlope(*this);}
    inline Point calNormal() {return calNormal(*this);}
    inline float calLength() {return calLength(*this);}

    // Set function
    inline void setVal(const int p_x, const int p_y){x = p_x; y = p_y;}

    // Direction.
    bool isInRange(const int width, const int height) const;
    bool isInRange(const Line &line) const;
    bool isOnBoundary(const int xBoundary, const int yBoundary) const;
    int isRight() const; // 0 : left, 1 : right, 2 : vertical
    int isDown() const; // 0 : up, 1 : down, 2 : horizontal

    // Print function
    inline void print() const {std::cout << x << ", " << y << std::endl;}
    inline std::string toString() const {return "P " + std::to_string(x) + " " + std::to_string(y);}

    // Static function.
    inline static bool cmp(const Point &a, const Point &b) {return a.x != b.x ? a.x < b.x : a.y < b.y;}
    inline static bool cmpY(const Point a, const Point b) {return a.y > b.y;}
    inline static bool cmpYPtr(const Point *a, const Point *b) {return a->y > b->y;}
    static Point calSlope(const Point &point);
    static int GCD(const int a, const int b);
    static Point calNormal(Point point);
    inline static float calLength(Point point) {return sqrt(point.x * point.x + point.y * point.y);}
    static int isSimilar(Point &a, Point &b);

    // Data
    int x;
    int y;
};

class Edge
{
public:
    Edge(const int p_x1 = 0, const int p_y1 = 0, const int p_x2 = 0, const int p_y2 = 0,
         const int p_x3 = 0, const int p_y3 = 0, const int p_x4 = 0, const int p_y4 = 0);
    Edge(const Point &point1, const Point &point2,
         const Point &point3 = Point(0, 0), const Point &point4 = Point(0, 0));

    // Operators overloading
    Edge operator+(const Edge &b);
    Edge operator-(const Edge &b);
    int operator==(Edge &b); // Strictly equal.

    // Calculate function
    inline Edge calPerpendicularBisector() {return calPerpendicularBisector(startPoint, endPoint);}
    inline float calLength() {return calLength(startPoint, endPoint);}

    // Set function
    void setVal(const int p_x1, const int p_y1, const int p_x2, const int p_y2,
                const int p_x3 = 0, const int p_y3 = 0, const int p_x4 = 0, const int p_y4 = 0);
    void setVal(const Point &point1, const Point &point2,
                const Point &point3 = Point(0, 0), const Point &point4 = Point(0, 0));
    void adjst(const int width, const int height);
    void adjst(const int width, const int height, const Point &p_starPoint);
    void fitRange(const int width, const int height);

    // Direction funciotn
    void toDirection(char dir);
    void lexicalOrder(); // The direction always is right.
    bool isInRange(const int xBoundary, const int yBoundary);
    inline int isRight() {return slope.isRight();} // 0 : left, 1 : right, 2 : vertical
    inline int isDown() {return slope.isDown();} // 0 : up, 1 : down, 2 : horizontal

    // Print function
    void print() const;
    std::string toString() const;

    // Static function.
    static bool cmp(const Edge &a, const Edge &b);
    inline static Point calSlope(Point &a, Point &b) {return Point::calSlope(b - a);}
    inline static Point calNormal(Point &a, Point &b) {return Point::calNormal(b - a);}
    static Edge calPerpendicularBisector(Point &a, Point &b);
    inline static float crossProduct(Point &a, Point &b) {return a.x * b.y - a.y * b.x;}
    inline static float calLength(Point &a, Point &b) {return Point::calLength(b - a);}

    // Data
    Point startPoint;
    Point endPoint;

    Point normStartPoint;
    Point normEndPoint;

    Point slope;
    Point normal;
    int type; // 1 : slash, 2 : vertical, 3 : horizontal 4 : point.
};


class Line : public Edge
{
public:
    Line(const int p_x1 = 0, const int p_y1 = 0, const int p_x2 = 0, const int p_y2 = 0,
         const int p_x3 = 0, const int p_y3 = 0, const int p_x4 = 0, const int p_y4 = 0);
    Line(const Point &p_startPoint, const Point &p_endPoint,
         const Point &point3 = Point(0, 0), const Point &point4 = Point(0, 0));
    Line(const Edge &p_edge,
         const Point &point3 = Point(0, 0), const Point &point4 = Point(0, 0));

    // Operators overloading
    int operator==(Line &b) {return startPoint == b.startPoint && endPoint == b.endPoint;} // Strictly equal.

    // Get function.
    Edge getEdge() const {return Edge(startPoint, endPoint);}

    // Set funciotn
    void setVal(const int p_x1, const int p_y1, const int p_x2, const int p_y2,
                const int p_x3 = 0, const int p_y3 = 0, const int p_x4 = 0, const int p_y4 = 0);
    void setVal(const Point &point1, const Point &point2,
                const Point &point3 = Point(0, 0), const Point &point4 = Point(0, 0));

    // Calculate function.
    float calX(const int p_y) const;
    float calY(const int p_x) const;
    Line calPerpendicularBisector();

    // Comparison function.
    int isEqual(Line &line2) {return isEqual(*this, line2);} // Non-strictly equal.

    // Static function.
    inline static bool cmp(const Line &line1, const Line &line2) {return Edge::cmp(line1.getEdge(), line2.getEdge());}
    static int isEqual(Line &line1, Line &line2); // Non-strictly equal.
    inline static bool isSolved(const Line &a, const Line &b) {return (a.a * b.b - a.b * b.a) != 0;} // Delta != 0 -> solved.
    static Point cramersRule(const Line &a, const Line &b);

    // Data : ax + by + c = 0
    float a;
    float b;
    float c;
};

class Polygon
{
public:
    explicit Polygon(){};
    Polygon(std::vector<Point> &points, const int left, const int right, const int isClockwise = 0);
    Polygon(std::vector<Point> &points, const int isClockwise = 0);

    // Operators overloading
    Line& operator[](const int i) {return lines[i];}

    // Get funciotn.
    inline int size() const {return lines.size();}

    // Set function.
    void setVal(std::vector<Point> &points, const int left, const int right, const int isClockwise = 0);
    void setVal(std::vector<Point> &points, const int isClockwise = 0);

    // Static function.
    static Polygon mergeConvexHull(Polygon &A,
                                   Polygon &B,
                                   std::vector<int> &tangent);
    static int isCrossingPolgon(Point &a1, Point &b1, Point &b2); // Help mergeConvexHull().

    // Print function
    inline void print() const {for (auto &l : lines) {l.print();}}

    // Data
    std::vector<Line> lines;
};


class Record
{
public:
    explicit Record(){};

    // Data
    Polygon convexHull;
    std::vector<Line> hyperPlane;
    std::vector<Line> voronoiLines;
};

class Trend
{
public:
    Trend(const int p_trend = 0, const int p_dir = 0, const int p_index = -1) {setVal(p_trend, p_dir, p_index);}

    // Operators overloading
    inline bool operator==(const Trend &b){return trend == b.trend && dir == b.dir && index == b.index;}

    // Set function
    void setVal(const int p_trend, const int p_dir, const int p_index);

    int trend; // 0: None, 1: Counterclockwise, 2: Clockwise.
    int dir;   // 0: None, 1: Left, 2: Right.
    int index; // -1: None.
};

void sortAndUnique(std::vector<Point> &points);

#endif // DATASTRUCTURE_H
