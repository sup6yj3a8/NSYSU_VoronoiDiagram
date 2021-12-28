// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#include "DataStructure.h"

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>

using namespace std;

/* Help function*/

void sortAndUnique(std::vector<Point> &points)
{
    sort(points.begin(), points.end(), Point::cmp);
    int fast = 1;
    int last = 0;
    int isCopy = 0;

    for (; fast < points.size(); ++fast) {
        if (points[fast] != points[last]) {
            if (isCopy) {
                points[++last] = points[fast];
            } else {
                ++last;
            }
        } else {
            isCopy = 1;
        }
    }
    points.resize(last + 1);
}

/* Point */

Point::Point(const int p_x, const int p_y):
    x(p_x),
    y(p_y)
{;}

bool Point::isInRange(const int width, const int height) const
{
    return 0 <= x && x <= width && 0 <= y && y <= height;
}

bool Point::isInRange(const Line &line) const
{
    const int minX = min(line.startPoint.x, line.endPoint.x);
    const int maxX = max(line.startPoint.x, line.endPoint.x);
    const int minY = min(line.startPoint.y, line.endPoint.y);
    const int maxY = max(line.startPoint.y, line.endPoint.y);
    return minX<= x && x <= maxX && minY <= y && y <= maxY;
}

bool Point::isOnBoundary(const int xBoundary, const int yBoundary) const
{
    return x == 0 || x == xBoundary || y == 0 || y == yBoundary;
}

int Point::isRight() const
{
    if (x < 0) {
        return 0; // 0 : left
    } else if (x > 0) {
        return 1; // 1 : right
    } else {
        return 2; // 2 : vertical
    }
}

int Point::isDown() const
{
    if (y > 0) {
        return 0; // 0 : up
    } else if (y < 0) {
        return 1; // 1 : down
    } else {
        return 2; // 2 : horizontal
    }
}

Point Point::calSlope(const Point &point)
{
    // 1. x == 0 || y == 0.
    if (point.x == 0 && point.y != 0) {
        return Point(0, point.y / abs(point.y));
    } else if (point.x != 0 && point.y == 0) {
        return Point(point.x / abs(point.x), 0);
    } else if (point.x == 0 && point.y == 0) {
        return Point(0, 0);
    }

    // 2. x != 0 && y != 0.
    const int gcd = GCD(abs(point.x), abs(point.y));
    return Point(point.x / gcd, point.y / gcd);
}

// Greatest common divisor
int Point::GCD(const int a, const int b)
{
    int d = 1;
    for (int i = 2; i <= std::min(a, b); ++i)  {
        if (a % i == 0 && b % i == 0) {d = i;}
    }
    return d;
}

Point Point::calNormal(Point point)
{
    if (point.x != 0 && point.y != 0){
        const int gcd = GCD(abs(point.x), abs(point.y));
        point.x /= gcd;
        point.y /= gcd;
    } else if (point.x == 0 && point.y != 0) {
        point.y /= abs(point.y);
    } else if (point.x != 0 && point.y == 0) {
        point.x /= abs(point.x);
    }
    swap(point.x, point.y);
    point.y *= -1;
    return point;
}

int Point::isSimilar(Point &a, Point &b)
{
    const int offset = 1;
    return b.x - offset <= a.x && a.x <= b.x + offset &&
           b.y - offset <= a.y && a.y <= b.y + offset &&
           a.x - offset <= b.x && b.x <= a.x + offset &&
           a.y - offset <= b.y && b.y <= a.y + offset;
}

/* Edge */

Edge::Edge(const int p_x1, const int p_y1, const int p_x2, const int p_y2,
           const int p_x3, const int p_y3, const int p_x4, const int p_y4)
{
    setVal(p_x1, p_y1, p_x2, p_y2, p_x3, p_y3, p_x4, p_y4);
}

Edge::Edge(const Point &point1, const Point &point2,
           const Point &point3, const Point &point4)
{
    setVal(point1, point2, point3, point4);
}

Edge Edge::operator+(const Edge &b)
{
    return Edge(startPoint + b.startPoint, endPoint + b.endPoint);
}

Edge Edge::operator-(const Edge &b)
{
    return Edge(startPoint - b.startPoint, endPoint - b.endPoint);
}

int Edge::operator==(Edge &b)
{
    return startPoint == b.startPoint && endPoint == b.endPoint;
}


void Edge::setVal(const int p_x1, const int p_y1, const int p_x2, const int p_y2,
                  const int p_x3, const int p_y3, const int p_x4, const int p_y4)
{
    startPoint.x = p_x1;
    startPoint.y = p_y1;
    endPoint.x = p_x2;
    endPoint.y = p_y2;

    normStartPoint.x = p_x3;
    normStartPoint.y = p_y3;
    normEndPoint.x = p_x4;
    normEndPoint.y = p_y4;

    slope = calSlope(startPoint, endPoint);
    normal = calNormal(startPoint, endPoint);

    // 1 : slash, 2 : vertical, 3 : horizontal 4 : point.
    if (slope.x != 0 && slope.y != 0){
        type = 1;
    } else if (slope.x == 0 && slope.y != 0) {
        type = 2;
    } else if (slope.x != 0 && slope.y == 0) {
        type = 3;
    } else {
        type = 4;
    }
}

void Edge::setVal(const Point &point1, const Point &point2,
                  const Point &point3, const Point &point4)
{
    setVal(point1.x, point1.y, point2.x, point2.y,
           point3.x, point3.y, point4.x, point4.y);
}

void Edge::adjst(const int width, const int height)
{
    Line line(*this);

    if (line.type == 1){
        // 1 : slash
        // 1. Calculate four candidate points.
        Point x0(0, line.calY(0));
        Point y0(line.calX(0), 0);
        Point xB(width, line.calY(width));
        Point yB(line.calX(height), height);

        // 2. Pick up all poins on the bondary.
        vector<Point> inRangePoints;
        if (x0.isInRange(width, height)) {inRangePoints.push_back(x0);}
        if (y0.isInRange(width, height)) {inRangePoints.push_back(y0);}
        if (xB.isInRange(width, height)) {inRangePoints.push_back(xB);}
        if (yB.isInRange(width, height)) {inRangePoints.push_back(yB);}

        sortAndUnique(inRangePoints);

        // 3. Update
        if ( (inRangePoints[1] - inRangePoints[0]).calSlope().isRight() == slope.isRight()) {
            startPoint = inRangePoints[0];
            endPoint = inRangePoints[1];
        } else {
            startPoint = inRangePoints[1];
            endPoint = inRangePoints[0];
        }

    } else if (line.type == 2) {
        // 2 : vertical
        if (line.isDown()) {
            startPoint.y = height;
            endPoint.y = 0;
        } else {
            startPoint.y = 0;
            endPoint.y = height;
        }
    } else if (line.type == 3) {
        // 3 : horizontal
        if (line.isRight()) {
            startPoint.x = 0;
            endPoint.x = height;
        } else {
            startPoint.x = height;
            endPoint.x = 0;
        }
    }
}

void Edge::adjst(const int width, const int height, const Point &p_starPoint)
{
    adjst(width, height);
    startPoint = p_starPoint;
}

void Edge::fitRange(const int width, const int height)
{
    if (!isInRange(width, height)){
        Point startPointCopy = startPoint;
        Point endPointCopy = endPoint;
        adjst(width, height);
        if (startPointCopy.isInRange(width, height)) {startPoint = startPointCopy;}
        if (endPointCopy.isInRange(width, height)) {endPoint = endPointCopy;}
    }
}

void Edge::toDirection(char dir)
{
    // 1. toRight (r) : Let the slope.x be positive(+).
    // 2. toLeft (l)  : Let the slope.x be negative(-).
    // 3. toUp (u)    : Let the slope.y be positive(+).
    // 4. toDown (d)  : Let the slope.y be negative(-).

    if ( (dir == 'r' && slope.x < 0) ||
         (dir == 'l' && slope.x > 0) ||
         (dir == 'u' && slope.y < 0) ||
         (dir == 'd' && slope.y > 0) ){
        swap(startPoint, endPoint);
        slope = slope * -1;
        normal = slope.calNormal();
    }
}

void Edge::lexicalOrder()
{
    if ( startPoint.x > endPoint.x || (startPoint.x == endPoint.x && startPoint.y > endPoint.y) ) {
        swap(startPoint, endPoint);
        slope = slope * -1;
        normal = slope.calNormal();
    }
}

bool Edge::isInRange(const int xBoundary, const int yBoundary)
{
    return startPoint.isInRange(xBoundary, yBoundary) && endPoint.isInRange(xBoundary, yBoundary);
}

void Edge::print() const
{
    cout << startPoint.x << ", " << startPoint.y << ", " << endPoint.x << ", " << endPoint.y << endl;
}

string Edge::toString() const
{
    return "E " + to_string(startPoint.x) + " " + to_string(startPoint.y) + " " +  to_string(endPoint.x) + " " + to_string(endPoint.y);
}


bool Edge::cmp(const Edge &a, const Edge &b)
{
    if (a.startPoint.x != b.startPoint.x) {
        return a.startPoint.x < b.startPoint.x;
    } else if (a.startPoint.y != b.startPoint.y) {
        return a.startPoint.y < b.startPoint.y;
    } else if (a.endPoint.x != b.endPoint.x) {
        return a.endPoint.x < b.endPoint.x;
    } else {
        return a.endPoint.y < b.endPoint.y;
    }
}

Edge Edge::calPerpendicularBisector(Point &a, Point &b)
{
    Edge PB; // PerpendicularBisector
    Point mid = (a + b) / 2;
    Point normal = Edge::calNormal(a, b);

    PB.setVal(mid, mid + normal);
    PB.normStartPoint = a;
    PB.normEndPoint = b;
    return PB;
}

/* Line */
Line::Line(const int p_x1, const int p_y1, const int p_x2, const int p_y2,
           const int p_x3, const int p_y3, const int p_x4, const int p_y4):
    Edge(p_x1, p_y1, p_x2, p_y2, p_x3, p_y3, p_x4, p_y4)
{
    Line::setVal(p_x1, p_y1, p_x2, p_y2, p_x3, p_y3, p_x4, p_y4);
}

Line::Line(const Point &p_startPoint, const Point &p_endPoint,
           const Point &point3, const Point &point4):
    Edge(p_startPoint, p_endPoint, point3, point4)
{
    Line::setVal(p_startPoint, p_endPoint, point3, point4);
}

Line::Line(const Edge &p_edge,
           const Point &point3, const Point &point4):
    Edge(p_edge.startPoint, p_edge.endPoint, point3, point4)
{
    Line::setVal(p_edge.startPoint, p_edge.endPoint, point3, point4);
};

void Line::setVal(const int p_x1, const int p_y1, const int p_x2, const int p_y2,
                  const int p_x3, const int p_y3, const int p_x4, const int p_y4)
{
    Edge::setVal(p_x1, p_y1, p_x2, p_y2, p_x3, p_y3, p_x4, p_y4);
    a = normal.x;
    b = normal.y;
    c = -1 * (a * startPoint.x + b *startPoint.y);
}

void Line::setVal(const Point &point1, const Point &point2,
                  const Point &point3, const Point &point4)
{
    Line::setVal(point1.x, point1.y, point2.x, point2.y,
                 point3.x, point3.y, point4.x, point4.y);
}


float Line::calX(const int p_y) const
{
    // 1 : slash, 2 : vertical, 3 : horizontal 4 : point.
    if (type == 1 || type == 2) {
        return (b * p_y + c) / (-1 * a);
    }
    return 0;
}

float Line::calY(const int p_x) const
{
    // 1 : slash, 2 : vertical, 3 : horizontal 4 : point.
    if (type == 1 || type == 3) {
        return (a * p_x + c) / (-1 * b);
    }
    return 0;
}

Line Line::calPerpendicularBisector()
{
    Edge PB = Edge::calPerpendicularBisector();
    return Line(PB, PB.normStartPoint, PB.normEndPoint);
}


// Non-strictly equal.
int Line::isEqual(Line &line1, Line &line2)
{
    return (line1.startPoint == line2.startPoint && line1.endPoint == line2.endPoint  ) ||
           (line1.startPoint == line2.endPoint   && line2.endPoint == line2.startPoint);
}

Point Line::cramersRule(const Line &a, const Line &b)
{
    float delta = a.a * b.b - a.b * b.a; // cross product.
    float deltaX = (-1 * a.c * b.b) - (a.b * -1 * b.c);
    float deltaY = (a.a * -1 * b.c) - (-1 * a.c * b.a);
    return Point(deltaX / delta, deltaY / delta);
}

/* Polygon */

Polygon::Polygon(std::vector<Point> &points,
                 const int left,
                 const int right,
                 const int isClockwise)
{
    setVal(points, left, right, isClockwise);
}

Polygon::Polygon(vector<Point> &points, const int isClockwise)
{
    setVal(points, 0, points.size() - 1, isClockwise);
}

void Polygon::setVal(vector<Point> &points, const int left, const int right, const int isClockwise)
{
    Edge edge1(points[left], points[left + 1]);
    Edge edge2(points[left + 1], points[left + 2]);
    const float CP = Edge::crossProduct(edge1.slope, edge2.slope);

    // CP > 0 : counterclockwise , CP < 0 : clockwise.
    // isClockwise = 0 : become counterclockwise, isClockwise = 1 : become clockwise.
    if ( (CP > 0 && isClockwise == 0) || (CP < 0 && isClockwise == 1)  || (CP == 0) ){
        // 1. The order of points and polygon are same.
        for (int i = left; i < right; ++i) {
            lines.push_back(Line(points[i], points[i + 1]));
        }
        lines.push_back(Line(points[right], points[left]));
    } else if ( (CP < 0 && isClockwise == 0) || (CP > 0 && isClockwise == 1)) {
        // 2. The order of points and polygon are different.
        for (int i = right; i > left; --i) {
            lines.push_back(Line(points[i], points[i - 1]));
        }
        lines.push_back(Line(points[left], points[right]));
    }
}


void Polygon::setVal(vector<Point> &points, const int isClockwise)
{
    setVal(points, 0, points.size() - 1, isClockwise);
}


// Merge a convex hull A and a convex hull polygon B.
Polygon Polygon::mergeConvexHull(Polygon &A,
                                 Polygon &B,
                                 vector<int> &tangent)
{
    const int aSize = A.size();
    const int bSize = B.size();

    // 1.1 Find the rightmost (iA) point of A.
    int iA = 0;
    for (int i = 1; i < aSize; ++ i) {
        if (A[i].startPoint.x > A[iA].startPoint.x) {iA = i;}
    }
    // 1.2 Find the leftmost (iB) point of B.
    int iB = 0;
    for (int i = 1; i < bSize; ++i) {
        if (B[i].startPoint.x < B[iB].startPoint.x) {iB = i;}
    }

    // 2.1 Find the upper tangent.
    int indA = iA;
    int indB = iB;
    int done = 0;
    while (!done) {
        done = 1; // Finish.
        // 2.1.1 A <- B
        while (isCrossingPolgon(B[indB].startPoint,
                                A[indA].startPoint,
                                A[(indA + 1) % aSize].startPoint) > 0) {
            indA = (indA + 1) % aSize;
        }
        // 2.1.1 A -> B
        while (isCrossingPolgon(A[indA].startPoint,
                                B[indB].startPoint,
                                B[(bSize + indB - 1) % bSize].startPoint) < 0) {
            indB = (bSize + indB - 1) % bSize;
            done = 0; // Continue.
        }
    }
    const int upperA = indA;
    const int upperB = indB;
    // 2.2 Find the lower tangent.
    indA = iA;
    indB = iB;
    done = 0;
    while (!done) {
        done = 1; // Finish.
        // 2.2.1 A -> B
        while (isCrossingPolgon(A[indA].startPoint,
                                B[indB].startPoint,
                                B[(indB + 1) % bSize].startPoint) > 0) {
            indB = (indB + 1) % bSize;
        }
        // 2.2.2 A <- B
        while (isCrossingPolgon(B[indB].startPoint,
                                A[indA].startPoint,
                                A[(aSize + indA - 1) % aSize].startPoint) < 0) {
            indA = (aSize + indA - 1) % aSize;
            done = 0; // Continue.
        }
    }
    const int lowerA = indA;
    const int lowerB = indB;
    // 2.3 Recodr the upper and lower tangent.
    tangent = {upperA, upperB, lowerA, lowerB};

    // 3. Merge A and B by counterclockwise.
    // 3.1 Cellect the merged points by counterclockwise.
    vector<Point> mergePoints;
    int ind = upperA;
    mergePoints.push_back(A[upperA].startPoint);
    while (ind != lowerA) {
        ind = (ind + 1) % aSize;
        mergePoints.push_back(A[ind].startPoint);
    }
    ind = lowerB;
    mergePoints.push_back(B[lowerB].startPoint);
    while (ind != upperB) {
        ind = (ind + 1) % bSize;
        mergePoints.push_back(B[ind].startPoint);
    }
    // 3.2 Return a merged polgon.
    return Polygon(mergePoints);
}

// Checks whether the line (a1b1) is crossing the polygon (B)
int Polygon::isCrossingPolgon(Point &a1, Point &b1, Point &b2)
{
    // 1. B (b1b2) X A (a1b2)
    Point B = b2 - b1;
    Point A = b1 - a1;
    const float res = Edge::crossProduct(B, A);

    // 2. Determine the relationship of slope1 and slope2.
    if (res > 0) {
        // 2.1 Clockwise
        return 1;
    } else if (res < 0) {
        // 2.2 Counterclockwise
        return -1;
    } else {
        // 2.3 Parallel
        return 0;
    }
}

/* Trend */

void Trend::setVal(const int p_trend, const int p_dir, const int p_index)
{
    trend = p_trend;
    dir = p_dir;
    index = p_index;
}

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
// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#include "DiagramScene.h"

#include <QLabel>
#include <QGraphicsSceneMouseEvent>
#include <QPointF>
#include <QString>
#include <QDebug>
#include <QPolygonF>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QList>
#include <QGraphicsTextItem>
#include <QStatusBar>

#include <vector>

using namespace std;

vector<QColor> colorPalette = {Qt::black, Qt::red, Qt::green, Qt::blue, Qt::cyan,
                            Qt::magenta};

/* Diagram scene */

DiagramScene::DiagramScene(QLabel* p_mousePos,
                           QLabel* p_pointsSize,
                           QLabel* p_edgesSize,
                           QTextBrowser* p_pointsData,
                           QTextBrowser* p_edgesData,
                           QStatusBar* p_status,
                           vector<Point> &p_points,
                           Record &p_voronoi,
                           vector<Record> &p_steps,
                           const int p_width,
                           const int p_height):
    mousePos(p_mousePos),
    pointsSize(p_pointsSize),
    edgesSize(p_edgesSize),
    pointsData(p_pointsData),
    edgesData(p_edgesData),
    status(p_status),
    points(p_points),
    voronoi(p_voronoi),
    steps(p_steps)
{
    setSceneRect(0, 0, p_width, p_height); // Set the size of scene
    drawBoundary();
}

DiagramScene::~DiagramScene()
{
    mousePos = nullptr;
    pointsSize = nullptr;
    edgesSize = nullptr;
    pointsData = nullptr;
    edgesData = nullptr;
}

// Get the position of scene
void DiagramScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    const QPointF pos = mouseEvent->scenePos();
    mousePos->setText("(" + QString::number(pos.x()) + ", " + QString::number(pos.y()) + ")");
}

// Add a new point into the diagram.
void DiagramScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    const QPointF pos = mouseEvent->scenePos();
    addPoint(pos.x(), pos.y());
}

void DiagramScene::addPoint(const int x, const int y, const int isAdd, const int color)
{
    const QPen pen(Qt::black);
    const QBrush brush(colorPalette[color], Qt::SolidPattern);
    const int pointSize = 4;

    if (isInRange(x, y)) {
        if (isAdd) {points.push_back(Point(x, y));}
        addEllipse(x - 4, y - 4, pointSize, pointSize, pen, brush);
        pointsData->append("P " + QString::number(x) + " " + QString::number(y));
        pointsSize->setText("點資料：" + QString::number(points.size()) + "點");
    }
}

void DiagramScene::addEdge(const int x1, const int y1, const int x2, const int y2,
                           const int isAdd, const int isPrint, const int color)
{
    const QPen pen(colorPalette[color]);

    if (isInRange(x1, y1, x2, y2)) {
        addLine(x1, y1, x2, y2, pen);
        if (isAdd) {voronoi.voronoiLines.push_back(Line(x1, y1, x2, y2));}
        if (isPrint) {
            edgesData->append("E " + QString::number(x1) + " " + QString::number(y1) + " " +
                                      QString::number(x2) + " " + QString::number(y2));
            edgesSize->setText("邊資料：" + QString::number(voronoi.voronoiLines.size()) + "邊");
        }
    }
}

void DiagramScene::addPolygon(Polygon &polygon, const int color)
{
    const QPen pen(colorPalette[color]);
    const int offset = -2;

    for (int i = 0; i < polygon.size(); ++i) {
        addLine(polygon[i].startPoint.x + offset, polygon[i].startPoint.y + offset,
                polygon[i].endPoint.x + offset  , polygon[i].endPoint.y + offset  , pen);
        QGraphicsTextItem *text = addText(QString::number(i + 1));
        text->setPos(polygon[i].startPoint.x, polygon[i].startPoint.y);
        text = nullptr;
    }
}


void DiagramScene::update()
{
    // 1. Restore the initial state.
    recover();

    // 2. Upadet the canvas.
    for (const auto &e : voronoi.voronoiLines) {addEdge(e, 0);}
    addPolygon(voronoi.convexHull);
    for (const auto &hp : voronoi.hyperPlane) {addEdge(hp, 0, 0);}
}

void DiagramScene::update(int &i)
{
    // 1. Recover.
    if (i == 1) {recover();}

    // 2. Upadet the canvas.
    int node = 0;
    int p_i = i;
    updateStep(points.size(), p_i, node, 1);

    // 3. Increasing and cycling index (i).
    ++i;
    if (node == steps.size()) {
        i = 1;
        recover();
    }
}

int DiagramScene::updateStep(int size, int &step, int &node, const int isLeft)
{
    if (size == 2 || size == 3) {
        // Stpe1, 3: Draw a convex hull.
        if (step == 1) {
            if (isLeft) {
                addPolygon(steps[node].convexHull);
                status->showMessage("第1步：畫Left convex hull", 3000);
            } else {
                addPolygon(steps[node].convexHull);
                status->showMessage("第3步：畫Right convex hull", 3000);
            }
            return 0;
        }
        --step;
        // Stpe2, 4: Draw a voronoi.
        if (step == 1) {
            if (isLeft) {
                for (const auto &e : steps[node].voronoiLines) {addEdge(e, 0, 1, 4);}
                status->showMessage("第2步：畫Left voronoi", 3000);
            } else {
                for (const auto &e : steps[node].voronoiLines) {addEdge(e, 0, 1, 5);}
                status->showMessage("第4步：畫Right voronoi", 3000);
                return 0;
            }
            return 0;
        }
        --step;
        ++node;
    } else {
        // 1. Divide
        const int mid = size / 2;
        int iL, iR;
        // 1.1 Left paert.
        if (size % 2) {
            if (updateStep(mid + 1, step, node, 1) == 0) {return 0;}
        } else {
            if (updateStep(mid, step, node, 1) == 0) {return 0;}
        }
        iL = node - 1;
        // 1.2 Right part.
        if (updateStep(mid, step, node, 0) == 0) {return 0;}
        iR = node - 1;

        // 2. Merge
        // Step5: Draw a mergerd convex hull.
        if (step == 1) {
            // Step5.1 Recover.
            recover();
            // Step5.2 Draw left and right voronoi.
            for (const auto &e : steps[iL].voronoiLines) {addEdge(e, 0, 1, 4);}
            for (const auto &e : steps[iR].voronoiLines) {addEdge(e, 0, 1, 5);}
            // Step5.3 Draw a mergerd convex hull.
            addPolygon(steps[node].convexHull);
            status->showMessage("第5步：畫merged convex hull", 3000);
            return 0;
        }
        --step;
        // Step6: Draw a hyperplane.
        if (step == 1) {
            for (const auto &hp : steps[node].hyperPlane) {addEdge(hp, 0, 0, 3);}
            status->showMessage("第6步：畫hyperplane", 3000);
            return 0;
        }
        --step;
        // Step7: Elimination.
        if (step == 1) {
            // Stpe7.1 Recover.
            recover();
            // Step7.2 Elimination
            for (const auto &e : steps[node].voronoiLines) {addEdge(e, 0, 1);}
            for (const auto &hp : steps[node].hyperPlane) {addEdge(hp, 0, 0, 3);}
            addPolygon(steps[node].convexHull);
            status->showMessage("第7步：消線", 3000);
            return 0;
        }
        --step;
        // Step8: Finish
        if (step == 1) {
            // Stpe7.1 Recover.
            recover();
            // Step7.2 Finish
            for (const auto &e : steps[node].voronoiLines) {addEdge(e, 0);}
            addPolygon(steps[node].convexHull);
            status->showMessage("第8步：完成", 3000);
            return 0;
        }
        --step;
        ++node;
    }
    return 1;
}


void DiagramScene::recover()
{
    // 1. Initialize all objects.
    clear();
    drawBoundary();

    pointsData->clear();
    edgesData->clear();

    // 2. Draw all points.
    for (const auto &p : points) {addPoint(p, 0);}
}

void DiagramScene::clearScene()
{
    // 1. Initialize all objects.
    clear();
    drawBoundary();

    // 2. Clear all points and edges.
    points.clear();
    voronoi.voronoiLines.clear();

    // 3. Initialize the information of all data.
    pointsData->clear();
    pointsSize->setText("點資料：" + QString::number(points.size()) + "點");
    edgesData->clear();
    edgesSize->setText("邊資料：" + QString::number(voronoi.voronoiLines.size()) + "邊");
}

void DiagramScene::drawBoundary()
{
    QPolygonF boundary;
    boundary.resize(4);
    boundary[0] = QPointF(0       , 0  );
    boundary[1] = QPointF(0       , width());
    boundary[2] = QPointF(height(), width());
    boundary[3] = QPointF(height(), 0  );
    QGraphicsScene::addPolygon(boundary);
}

bool DiagramScene::isInRange(const int x,const int y)
{
    return 0 <= x && x <= width() && 0 <= y && y <= height();
}
// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include "DataStructure.h"

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QLabel>
#include <QTextBrowser>
#include <QStatusBar>

#include <vector>

class DiagramScene : public QGraphicsScene
{
public:
    DiagramScene(QLabel* p_mousePos,
                 QLabel* p_pointsSize,
                 QLabel* p_edgesSize,
                 QTextBrowser* p_pointsData,
                 QTextBrowser* p_edgesData,
                 QStatusBar* p_status,
                 std::vector<Point> &p_points,
                 Record &p_voronoi,
                 std::vector<Record> &p_steps,
                 const int p_width,
                 const int p_height);
    virtual ~DiagramScene();

    // Add function
    void addPoint(const int x, const int y, const int isAdd = 1, const int color = 1);
    inline void addPoint(const Point &point, const int isAdd = 1, const int color = 1) {addPoint(point.x, point.y, isAdd, color);}
    void addEdge(const int x1, const int y1, const int x2, const int y2,
                 const int isAdd = 1, const int isPrint = 1, const int color = 0);
    inline void addEdge(const Edge &edge, const int isAdd = 1,
                         const int isPrint = 1, const int color = 0) {addEdge(edge.startPoint.x, edge.startPoint.y, edge.endPoint.x, edge.endPoint.y, isAdd, isPrint, color);}
    inline void addEdge(const Line &line, const int isAdd = 1,
                        const int isPrint = 1, const int color = 0) {addEdge(line.startPoint.x, line.startPoint.y, line.endPoint.x, line.endPoint.y, isAdd, isPrint, color);}
    void addPolygon(Polygon &polygon, const int color = 2);

    // Updates function.
    void update();
    void update(int &i);
    int updateStep(int size, int &step, int &node, const int isLeft);

    void recover();
    void clearScene();


protected:
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent);
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);

private:
    // Data members
    QLabel* mousePos;
    QLabel* pointsSize;
    QLabel* edgesSize;
    QTextBrowser* pointsData;
    QTextBrowser* edgesData;
    QStatusBar* status;
    std::vector<Point> &points;
    Record &voronoi;
    std::vector<Record> &steps;

    // Member functions
    void drawBoundary();
    bool isInRange(const int x,const int y);
    bool isInRange(const int x1,const int y1, const int x2,const int y2) {return isInRange(x1, y1) && isInRange(x2, y2);}
};

#endif // DIAGRAMSCENE_H
// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#include "Information.h"
#include "ui_Information.h"

Information::Information(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Information)
{
    ui->setupUi(this);
}

Information::~Information()
{
    delete ui;
}

void Information::on_close_clicked()
{
    close();
}

// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#ifndef INFORMATION_H
#define INFORMATION_H

#include <QDialog>

namespace Ui {
class Information;
}

class Information : public QDialog
{
    Q_OBJECT

public:
    explicit Information(QWidget *parent = nullptr);
    ~Information();

private slots:
    void on_close_clicked();

private:
    Ui::Information *ui;
};

#endif // INFORMATION_H
<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>Information</class>
 <widget class="QDialog" name="Information">
  <property name="geometry">
   <rect>
    <x>0</x>
    <y>0</y>
    <width>349</width>
    <height>378</height>
   </rect>
  </property>
  <property name="windowTitle">
   <string>Dialog</string>
  </property>
  <layout class="QVBoxLayout" name="verticalLayout">
   <item>
    <widget class="QLabel" name="label_17">
     <property name="text">
      <string>&lt;html&gt;&lt;head/&gt;&lt;body&gt;&lt;p align=&quot;center&quot;&gt;&lt;span style=&quot; font-size:18pt; font-weight:600; color:#0433ff;&quot;&gt;Voronoi diagram 軟體資訊&lt;/span&gt;&lt;/p&gt;&lt;/body&gt;&lt;/html&gt;</string>
     </property>
    </widget>
   </item>
   <item>
    <layout class="QFormLayout" name="formLayout">
     <item row="0" column="0">
      <widget class="QLabel" name="label_2">
       <property name="text">
        <string>作者</string>
       </property>
      </widget>
     </item>
     <item row="0" column="1">
      <widget class="QLabel" name="label">
       <property name="text">
        <string>呂宗霖 (Tsung-Lin Lu)</string>
       </property>
      </widget>
     </item>
     <item row="1" column="0">
      <widget class="QLabel" name="label_7">
       <property name="text">
        <string>學號</string>
       </property>
      </widget>
     </item>
     <item row="1" column="1">
      <widget class="QLabel" name="label_8">
       <property name="text">
        <string>M093040114</string>
       </property>
      </widget>
     </item>
     <item row="2" column="0">
      <widget class="QLabel" name="label_9">
       <property name="text">
        <string>單位</string>
       </property>
      </widget>
     </item>
     <item row="2" column="1">
      <widget class="QLabel" name="label_10">
       <property name="text">
        <string>國立中山大學資訊工程學系碩士班</string>
       </property>
      </widget>
     </item>
     <item row="3" column="0">
      <widget class="QLabel" name="label_3">
       <property name="text">
        <string>發布日期</string>
       </property>
      </widget>
     </item>
     <item row="3" column="1">
      <widget class="QLabel" name="label_4">
       <property name="text">
        <string>2021/12</string>
       </property>
      </widget>
     </item>
     <item row="4" column="0">
      <widget class="QLabel" name="label_5">
       <property name="text">
        <string>程式語言</string>
       </property>
      </widget>
     </item>
     <item row="4" column="1">
      <widget class="QLabel" name="label_6">
       <property name="text">
        <string>C++</string>
       </property>
      </widget>
     </item>
     <item row="5" column="0">
      <widget class="QLabel" name="label_13">
       <property name="text">
        <string>開發框架</string>
       </property>
      </widget>
     </item>
     <item row="5" column="1">
      <widget class="QLabel" name="label_14">
       <property name="text">
        <string>Qt 6.1.3.</string>
       </property>
      </widget>
     </item>
     <item row="11" column="0">
      <widget class="QLabel" name="label_11">
       <property name="text">
        <string>電子郵件</string>
       </property>
      </widget>
     </item>
     <item row="11" column="1">
      <widget class="QLabel" name="label_12">
       <property name="text">
        <string>sup6yj3a8@yahoo.com.tw</string>
       </property>
      </widget>
     </item>
     <item row="12" column="0">
      <widget class="QLabel" name="label_15">
       <property name="text">
        <string>GitHub</string>
       </property>
      </widget>
     </item>
     <item row="12" column="1">
      <widget class="QLabel" name="label_16">
       <property name="text">
        <string>@sup6yj3a8</string>
       </property>
      </widget>
     </item>
     <item row="6" column="0">
      <widget class="QLabel" name="label_20">
       <property name="text">
        <string>畫布大小</string>
       </property>
      </widget>
     </item>
     <item row="6" column="1">
      <widget class="QLabel" name="label_21">
       <property name="text">
        <string>600 x 600</string>
       </property>
      </widget>
     </item>
     <item row="7" column="0">
      <widget class="QLabel" name="label_18">
       <property name="text">
        <string>Vorono algorithm</string>
       </property>
      </widget>
     </item>
     <item row="7" column="1">
      <widget class="QLabel" name="label_19">
       <property name="text">
        <string>Divide and conquer</string>
       </property>
      </widget>
     </item>
     <item row="8" column="0">
      <widget class="QLabel" name="label_22">
       <property name="text">
        <string>修課資訊</string>
       </property>
      </widget>
     </item>
     <item row="8" column="1">
      <widget class="QLabel" name="label_23">
       <property name="text">
        <string>110-1 高等演算法</string>
       </property>
      </widget>
     </item>
     <item row="9" column="1">
      <widget class="QLabel" name="label_26">
       <property name="text">
        <string>楊昌彪 老師</string>
       </property>
      </widget>
     </item>
     <item row="9" column="0">
      <widget class="QLabel" name="label_25">
       <property name="text">
        <string>授課教師</string>
       </property>
      </widget>
     </item>
     <item row="10" column="0">
      <widget class="QLabel" name="label_27">
       <property name="text">
        <string>&lt;html&gt;&lt;head/&gt;&lt;body&gt;&lt;p&gt;&lt;span style=&quot; font-weight:600; color:#ff2600;&quot;&gt;使用警語&lt;/span&gt;&lt;/p&gt;&lt;/body&gt;&lt;/html&gt;</string>
       </property>
      </widget>
     </item>
     <item row="10" column="1">
      <widget class="QLabel" name="label_28">
       <property name="text">
        <string>&lt;html&gt;&lt;head/&gt;&lt;body&gt;&lt;p&gt;&lt;span style=&quot; font-weight:600; color:#ff2600;&quot;&gt;4點以上有機會當掉，請小心服用&lt;/span&gt;&lt;/p&gt;&lt;/body&gt;&lt;/html&gt;</string>
       </property>
      </widget>
     </item>
    </layout>
   </item>
   <item>
    <layout class="QHBoxLayout" name="horizontalLayout">
     <item>
      <widget class="QLabel" name="label_24">
       <property name="text">
        <string>&lt;html&gt;&lt;head/&gt;&lt;body&gt;&lt;p&gt;&lt;span style=&quot; font-weight:600; color:#0433ff;&quot;&gt;©2021 呂宗霖 版權所有 &lt;/span&gt;&lt;/p&gt;&lt;/body&gt;&lt;/html&gt;</string>
       </property>
      </widget>
     </item>
     <item>
      <widget class="QPushButton" name="close">
       <property name="text">
        <string>關閉</string>
       </property>
      </widget>
     </item>
    </layout>
   </item>
  </layout>
 </widget>
 <resources/>
 <connections/>
</ui>
// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#include "VoronoiDiagram.h"
#include "DataStructure.h"

#include <vector>
#include <algorithm>

using namespace std;

/* remove line */

void removeLine(vector<Line> &voronoiLines, const int index)
{
    // 1. Check if the index is valid.
    if (index < 0 && index >= voronoiLines.size()) {return;}

    // 2. Remove a line.
    for (int i = index; i < voronoiLines.size() - 1; ++i) {
        voronoiLines[i] = voronoiLines[i + 1];
    }
    voronoiLines.pop_back();
}


/* Voronoi Diagram */

VoronoiDiagram::VoronoiDiagram(const int p_width,
                               const int p_height,
                               vector<Point>& p_points):
    width(p_width),
    height(p_height),
    points(p_points)
{
    // 1. Sort all points and delete repeated points.
    sortAndUnique(points);
}


Record VoronoiDiagram::divideAndConquer(const int left, const int right)
{
    vector<Record> steps;
    return stepBystep(left, right, steps, 0);
}


Record VoronoiDiagram::stepBystep(const int left,
                                  const int right,
                                  std::vector<Record>& steps,
                                  const int isStep)
{
    const int pointsSize = right - left + 1;
    Record result;

    if (pointsSize == 1) {
        return result;
    } else if (pointsSize == 2) {
        result = voronoi2(left, right);
        if (isStep) {steps.push_back(result);}
    } else if (pointsSize == 3) {
        result = voronoi3(left, right);
        if (isStep) {steps.push_back(result);}
    } else if (pointsSize >= 4) {
        // 1. Divide.
        const int mid = (right + left) / 2;
        Record voronoiLeft = stepBystep(left, mid, steps, isStep);
        Record voronoiRight = stepBystep(mid + 1, right, steps, isStep);
        // 2. Merge.
        result = merge(voronoiLeft, voronoiRight);
        if (isStep) {steps.push_back(result);}
    }
    return result;
}



Line VoronoiDiagram::calPerpendicularBisector(Point &a, Point &b)
{
    Line pb = Line(a, b).calPerpendicularBisector();
    pb.adjst(width, height);
    pb.lexicalOrder();
    return pb;
}


Record VoronoiDiagram::voronoi2(const int left, const int right)
{
    // 1. Calculate and adjust a perpendicular bisector.
    //    Store it into the result.
    Record result;
    result.voronoiLines.push_back(calPerpendicularBisector(points[left], points[right]));
    result.convexHull = Polygon(points, left, right);
    return result;
}

Record VoronoiDiagram::voronoi3(const int left, const int right)
{
    Record result;
    result.convexHull = Polygon(points, left, right, 0);
    Polygon triangle = result.convexHull; // Alias

    // 1. The triange is a triange and the circumcenter is in the range.
    if (triangle[0].slope != triangle[1].slope) {
        Point intersection = Line::cramersRule(triangle[0].calPerpendicularBisector(), triangle[1].calPerpendicularBisector());
        if (intersection.isInRange(width, height)) {
            for (int i = 0; i < triangle.size(); ++i) {
                Line pb  = triangle[i].calPerpendicularBisector();
                pb.adjst(width, height, intersection);
                result.voronoiLines.push_back(pb);
            }
            return result;
        }
    }

    // 2. The triangle is a line or the circumcenter is not in the range.
    int maxLength = 0;
    if (triangle[1].calLength() > triangle[maxLength].calLength()) {maxLength = 1;}
    if (triangle[2].calLength() > triangle[maxLength].calLength()) {maxLength = 2;}
    for (int i = 0; i < 3; ++i){if (i != maxLength) {result.voronoiLines.push_back(triangle[i].calPerpendicularBisector());}}
    for (auto &line : result.voronoiLines) {line.adjst(width, height);}

    return result;
}

Record VoronoiDiagram::merge(Record voronoiLeft, Record voronoiRight)
{
    Record result;

    // 1.1 Merge left and right convex hull.
    Polygon &lConvexHull = voronoiLeft.convexHull;  // Alias.
    Polygon &rConvexHull = voronoiRight.convexHull; // Alias.
    vector<int> tangent;
    result.convexHull = Polygon::mergeConvexHull(lConvexHull, rConvexHull, tangent);

    // 2. Find a hyper planes.
    vector<Line> &hyperPlane = result.hyperPlane; // Alias.

    Line scanLine(lConvexHull[tangent[0]].startPoint, rConvexHull[tangent[1]].startPoint); // Upper tangent.
    Line lowerTangent(lConvexHull[tangent[2]].startPoint, rConvexHull[tangent[3]].startPoint);
    Line scanPB;
    vector<Trend> scanTrend;

    vector<Point> intersection(2);
    int nearIS = -1, lIS = -1, rIS = -1;
    int choose = 0; // 0 : None, 1 : Left, 2 : Right

    while (true) {
        // 2.0 Initionlization
        for (auto & p : intersection) {p.setVal(-1, -1);}
        nearIS = -1, lIS = -1, rIS = -1;
        choose = 0;

        // 2.1 Calculate the perpendicular bisector of scan.
        scanPB = scanLine.calPerpendicularBisector();
        adjustStartPoint(scanPB, hyperPlane); // Adjust the stratPoint of the PB of the scan.

        // 2.2 Calculate the nearest intersection of scan.
        findNearIntraIS(scanPB, voronoiLeft.voronoiLines, intersection[0], lIS);
        findNearIntraIS(scanPB, voronoiRight.voronoiLines, intersection[1], rIS);
        findNearInterIS(scanPB, intersection, lIS, rIS, nearIS, choose);
        const int IS = nearIS ? rIS : lIS;
        scanTrend.push_back({0, choose, IS}); // Record the ray being faced by the scan line.

        // 2.3 Add the new hyperplane.
        addHyperPlane(scanPB, intersection, nearIS, hyperPlane,
                      Line::isEqual(scanLine, lowerTangent));

        // 2.4 Move the scan and next loop.
        if (Line::isEqual(scanLine, lowerTangent)) { // Stop condition.
            break;
        } else {
            moveScanNextLoop(scanLine,
                             voronoiLeft.voronoiLines, voronoiRight.voronoiLines,
                             lIS,rIS,choose);
        }
    }
    // 2.5 Update trends of scan lines
    updateTrend(hyperPlane, scanTrend);

    // 3.1 Elimination
    elimination(hyperPlane, scanTrend,
                voronoiLeft.voronoiLines,
                voronoiRight.voronoiLines);
    // 3.2 Merge left and right voronoi lines.
    mergeVoronoiLines(result.voronoiLines, voronoiLeft.voronoiLines);
    mergeVoronoiLines(result.voronoiLines, voronoiRight.voronoiLines);
    mergeVoronoiLines(result.voronoiLines, result.hyperPlane);

    return result;
}


// Adjust the stratPoint of the PB of the scan.
void VoronoiDiagram::adjustStartPoint(Line &PB,
                                      const vector<Line> &hyperPlane)
{
    // 1.1 Adjst a starPoint and an endPoint of the new hyperplane.
    PB.adjst(width, height);
    PB.toDirection('d');
    // 1.2 Link the previos hyperplane and the new htperplane.
    //     Update the startPoint of the new hyperplane.
    if (hyperPlane.size() > 0 &&
        !hyperPlane[hyperPlane.size() - 1].endPoint.isOnBoundary(width, height)) {
        PB.startPoint = hyperPlane[hyperPlane.size() - 1].endPoint;
    }
}


void VoronoiDiagram::findNearIntraIS(Line &scanPB,
                     std::vector<Line> &voronoiLines,
                     Point &intersection,
                     int &IS)
{
    Point tmpIS(-1, -1);
    Point noIS(-1, -1);

    // 1. Find a nearest intra IS in the voronoiLines.
    for (int i = 0; i < voronoiLines.size(); ++i) {
        // 1.1 The voronoiLines must contain an normPoint of scanLine.
        if (scanPB.normStartPoint == voronoiLines[i].normStartPoint ||
            scanPB.normEndPoint == voronoiLines[i].normStartPoint ||
            scanPB.normStartPoint == voronoiLines[i].normEndPoint ||
            scanPB.normEndPoint == voronoiLines[i].normEndPoint) {
            tmpIS = Line::isSolved(scanPB, voronoiLines[i]) ?
                    Line::cramersRule(scanPB, voronoiLines[i]) :
                    noIS;
            // 2.2 Find a nearest intra IS.
            if (tmpIS != noIS &&
                !Point::isSimilar(tmpIS, scanPB.startPoint) &&
                (intersection == noIS ||
                 Point::calLength(tmpIS - scanPB.startPoint) <
                 Point::calLength(intersection - scanPB.startPoint))) {
                intersection = tmpIS;
                IS = i;
            }
        }
    }
}


void VoronoiDiagram::findNearInterIS(Line &scanPB,
                                     vector<Point> &intersection,
                                     const int lIS,
                                     const int rIS,
                                     int &nearIS,
                                     int & choose)
{
    Point noIS(-1, -1);

    if (lIS != -1 && rIS != -1) {
        nearIS = 0;
        choose = 1; // Left
        if (( intersection[1] != noIS &&
              intersection[1].isInRange(scanPB) &&
              Point::calLength(intersection[1] - scanPB.startPoint) <
              Point::calLength(intersection[0] - scanPB.startPoint) &&
              !Point::isSimilar(scanPB.startPoint, intersection[1]) ) ||
            ( Point::isSimilar(scanPB.startPoint, intersection[0])  ) ) {
            nearIS = 1;
            choose = 2; // Right
        }
        if (Point::calLength(intersection[1] - scanPB.startPoint) ==
            Point::calLength(intersection[0] - scanPB.startPoint)) {
            nearIS = -1;
            choose = 0;
        }
    } else if (lIS != -1 && rIS == -1) { // Left
        nearIS = 0;
        choose = 1;
    } else if (lIS == -1 && rIS != -1) { // Right
        nearIS = 1;
        choose = 2;
    }
}


void VoronoiDiagram::addHyperPlane(Line &scanPB,
                                   vector<Point> &intersection,
                                   const int nearIS,
                                   vector<Line> &hyperPlane,
                                   const int isLowerTangent)
{
    // 2.1 Update the endPoint of the new hyperplane.
    if (nearIS != -1 && !isLowerTangent && intersection[nearIS].isInRange(width, height)) {
        scanPB.endPoint = intersection[nearIS];
    }
    // 2.2 Add
    hyperPlane.push_back(scanPB);
}


void VoronoiDiagram::moveScanNextLoop(Line &scanLine,
                                      vector<Line> &lVoronoiLines,
                                      vector<Line> &rVoronoiLines,
                                      const int lIS,
                                      const int rIS,
                                      const int choose)
{
    if (choose == 1 || choose == 0) { // Left
        if (scanLine.startPoint == lVoronoiLines[lIS].normStartPoint) {
            scanLine.startPoint = lVoronoiLines[lIS].normEndPoint;
        } else {
            scanLine.startPoint = lVoronoiLines[lIS].normStartPoint;
        }
    }

    if (choose == 2 || choose == 0 ) { // Right
        if (scanLine.endPoint == rVoronoiLines[rIS].normStartPoint) {
            scanLine.endPoint = rVoronoiLines[rIS].normEndPoint;
        } else {
            scanLine.endPoint = rVoronoiLines[rIS].normStartPoint;
        }
    }
}

void VoronoiDiagram::updateTrend(vector<Line> &hyperPlane,
                                 vector<Trend> &scanTrend)
{
    // 1. Check the size of hyperplans isn't less 1.
    if (hyperPlane.size() < 1) {return;}

    // 2. Determine the trend (counterclockwise / clockwise) of hyperplanes.
    for (int i = 0; i < hyperPlane.size() - 1; ++i) {
        if (Line::crossProduct(hyperPlane[i].slope, hyperPlane[i + 1].slope) > 0) {
            scanTrend[i].trend = 1; // 1: Counterclockwise.
        } else if (Line::crossProduct(hyperPlane[i].slope, hyperPlane[i + 1].slope) < 0) {
            scanTrend[i].trend = 2; // 2: Clockwise.
        }
    }
}


void VoronoiDiagram::elimination(vector<Line> &hyperPlane,
                                 vector<Trend> &scanTrend,
                                 vector<Line> &lVoronoiLines,
                                 vector<Line> &rVoronoiLines)
{
    vector<Line> *voronoiLines;

    for (int i = 0; i < scanTrend.size() - 1; ++i) {
        // 1. Determine the direction of the voronoiLines.
        if (scanTrend[i].dir == 0) {        // 0: None.
            continue;
        } else if (scanTrend[i].dir == 1) { // 1: Left.
            voronoiLines = &lVoronoiLines;
        } else if (scanTrend[i].dir == 2) { // 2: Right.
            voronoiLines = &rVoronoiLines;
        }
        // 2. Shorten the corresponding ray of voronoiLines.
        (*voronoiLines)[scanTrend[i].index].toDirection('r');
        // 2.1 The endPoint of the hyperplane must be not on the boundary.
        if (hyperPlane[i].endPoint.isOnBoundary(width, height)) {continue;}
        // 2.2 Shorten base on trend.
        if (scanTrend[i].trend == 1) {        // 1: Counterclockwise.
            if ((*voronoiLines)[scanTrend[i].index].endPoint.isOnBoundary(width, height)) {
                (*voronoiLines)[scanTrend[i].index].endPoint = hyperPlane[i].endPoint;
            } else if (isSameTrend(scanTrend, i)) {
                (*voronoiLines)[scanTrend[i].index].startPoint = hyperPlane[i].endPoint;
            } else {
                Point delPoint = (*voronoiLines)[scanTrend[i].index].endPoint;
                (*voronoiLines)[scanTrend[i].index].endPoint = hyperPlane[i].endPoint;
                removeDanglingLine(*voronoiLines, delPoint);
            }
        } else if (scanTrend[i].trend == 2) { // 2: Clockwise.
            if ((*voronoiLines)[scanTrend[i].index].startPoint.isOnBoundary(width, height)) {
                (*voronoiLines)[scanTrend[i].index].startPoint = hyperPlane[i].endPoint;
            } else if (isSameTrend(scanTrend, i)) {
                (*voronoiLines)[scanTrend[i].index].endPoint = hyperPlane[i].endPoint;
            } else {
                Point delPoint = (*voronoiLines)[scanTrend[i].index].startPoint;
                (*voronoiLines)[scanTrend[i].index].startPoint = hyperPlane[i].endPoint;
                removeDanglingLine(*voronoiLines, delPoint);
            }

        }
    }
}


int VoronoiDiagram::isSameTrend(vector<Trend> &scanTrend,
                                const int j)
{
    for (int i = 0; i < j; ++i) {
        if (scanTrend[i] == scanTrend[j]) {return true;}
    }
    return false;
}

void VoronoiDiagram::removeDanglingLine(vector<Line> &voronoiLines,
                                        Point &delPoint)
{
    // 1. Count the size of dangling line.
    int count = 0;  // The size of dangling line.
    int index = -1; // The index of a dangling line.
    for (int i = 0; i < voronoiLines.size(); ++i) {
        if (voronoiLines[i].startPoint == delPoint ||
            voronoiLines[i].endPoint == delPoint) {
            ++count;
            index = i;
        }
    }

    // 2. Remove a dangling line.
    if (count == 1) {removeLine(voronoiLines, index);}
}


// A + A = B
void VoronoiDiagram::mergeVoronoiLines(std::vector<Line> &A,
                                       const std::vector<Line> &B)
{
    for (const auto &b : B) {A.push_back(b);}
}
// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#ifndef VORONOIDIAGRAM_H
#define VORONOIDIAGRAM_H

#include "DataStructure.h"

#include <vector>

class VoronoiDiagram
{
public:
    VoronoiDiagram(const int p_width,
                   const int p_height,
                   std::vector<Point>& p_points);

    inline Record run() {return divideAndConquer(0, points.size() - 1);}
    Record divideAndConquer(const int left, const int right);
    inline Record runStepByStep(std::vector<Record>& steps) {return stepBystep(0, points.size() - 1, steps, 1);}
    Record stepBystep(const int left,
                      const int right,
                      std::vector<Record>& steps,
                      const int isStep = 1);

private:
    // Data
    int width;
    int height;
    std::vector<Point> &points;

    // Functions
    Line calPerpendicularBisector(Point &a, Point &b);
    Record voronoi2(const int left, const int right);
    Record voronoi3(const int left, const int right);
    Record merge(Record voronoiLeft, Record voronoiRight);

    // Help finding hyperPlanes.
    void adjustStartPoint(Line &PB,
                          const std::vector<Line> &hyperPlane);
    void findNearIntraIS(Line &scanPB,
                         std::vector<Line> &voronoiLines,
                         Point &intersection,
                         int &IS);
    void findNearInterIS(Line &scanPB,
                         std::vector<Point> &intersection,
                         const int lIS,
                         const int rIS,
                         int &nearIS,
                         int & choose);
    void addHyperPlane(Line &scanPB,
                       std::vector<Point> &intersection,
                       const int nearIS,
                       std::vector<Line> &hyperPlane,
                       const int isLowerTangent);
    void moveScanNextLoop(Line &scanLine,
                          std::vector<Line> &lVoronoiLines,
                          std::vector<Line> &rVoronoiLines,
                          const int lIS,
                          const int rIS,
                          const int choose);
    void updateTrend(std::vector<Line> &hyperPlane,
                     std::vector<Trend> &scanTrend);

    // Help elimination.
    void elimination(std::vector<Line> &hyperPlane,
                     std::vector<Trend> &scanTrend,
                     std::vector<Line> &lVoronoiLines,
                     std::vector<Line> &rVoronoiLines);
    int isSameTrend(std::vector<Trend> &scanTrend,
                    const int j);
    void removeDanglingLine(std::vector<Line> &voronoiLines,
                            Point &delPoint);
    void mergeVoronoiLines(std::vector<Line> &A,
                           const std::vector<Line> &B);

};

void removeLine(std::vector<Line> &voronoiLines, const int index);



#endif // VORONOIDIAGRAM_H
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Information.cpp \
    VoronoiDiagram.cpp \
    DataStructure.cpp \
    DiagramScene.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    Information.h \
    VoronoiDiagram.h \
    DataStructure.h \
    DiagramScene.h \
    mainwindow.h

FORMS += \
    Information.ui \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Information.h"

#include "DiagramScene.h"
#include "DataStructure.h"
#include "VoronoiDiagram.h"

#include <QGraphicsScene>
#include <QFileDialog>
#include <QIntValidator>
#include <QDir>
#include <QMessageBox>

#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <random>
#include <ctime>
#include <string>
#include <cctype>
using namespace std;


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , width(600)
    , height(600)
    , filePos(0)
    , stepNum(-1)
{
    ui->setupUi(this);
    setWindowTitle("Voronoi Diagram");
    defaultPath = QDir::homePath();

    info = new Information;
    info->setWindowTitle("軟體資訊");

    scene = new DiagramScene(ui->mousePos,
                             ui->pointsSize,
                             ui->edgesSize,
                             ui->pointsData,
                             ui->edgesData,
                             ui->statusbar,
                             points,
                             voronoi,
                             steps,
                             width,
                             height);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->setMouseTracking(true);

    ui->xValue->setValidator(new QIntValidator(0, width, this));
    ui->xValue->setPlaceholderText("輸入0~" + QString::number(width) + "整數");

    ui->yValue->setValidator(new QIntValidator(0, height, this));
    ui->yValue->setPlaceholderText("輸入0~" + QString::number(height) + "整數");

    ui->addPointsSize->setValidator(new QIntValidator(1, 999, this));
    ui->addPointsSize->setPlaceholderText("輸入1~999整數");
}

MainWindow::~MainWindow()
{
    delete ui;
    delete info;
    delete scene;
}

void MainWindow::on_clear_clicked()
{
    scene->clearScene();
    stepNum = -1;
    steps.clear();
    ui->statusbar->showMessage("畫面與資料已清空", 3000);
}

void MainWindow::on_readInput_clicked()
{
    QString filter = "Text (*.txt)";

    // 1. Get the name of a new input file.
    QString newFileName = QFileDialog::getOpenFileName(this, "開啟輸入檔", defaultPath, filter);
    if (newFileName == "") {
        ui->statusbar->showMessage("沒有選擇任何檔案", 3000);
        return;
    } else {
        // 1.1 Update the fileName.
        fileName = newFileName;
        defaultPath = getFilePath(newFileName);
        ui->statusbar->showMessage("已經選擇 " + shortFileName(newFileName), 3000);

        // 1.2 Reset.
        on_clear_clicked();
        fileContents.clear();
        filePos = 0;

        // 1.3 Open the file and check if it is opened.
        ifstream file(fileName.toLocal8Bit().constData());
        if (!file){
            ui->statusbar->showMessage("沒有開啟任何檔案", 3000);
            return;
        }

        // 2. Read all data.
        string lineText;
        Point point;
        while (true) {
            getline(file, lineText);
            // 2.1 Ignore the comments in the file.
            while (lineText[0] == '#' || lineText.size() == 0) {
                getline(file, lineText);
            }

            // 2.2 Read the set of data.
            if (lineText == "0") {
                fileContents.push_back(lineText);
                ui->statusbar->showMessage("全部資料已讀取完畢", 3000);
                break;
            } else if (!isdigit(lineText[0])) {
                ui->statusbar->showMessage("此檔案不符合輸入檔格式", 3000);
                return;
            } else {
                fileContents.push_back(lineText);
                unsigned int setSize = stoi(lineText);
                while (setSize--) {
                    getline(file, lineText);
                    fileContents.push_back(lineText);
                }
            }
        }

        // 3. Close the file and read the next set of data.
        file.close();
        on_nextData_clicked();
    }
}

// Read a next set of data.
void MainWindow::on_nextData_clicked()
{
    // 0. Check if there is a opened file.
    if (fileName == "") {
        ui->statusbar->showMessage("沒有選擇任何檔案", 3000);
        return;
    }

    // 1 Reset and read this set of data.
    on_clear_clicked(); // Reset.
    int j = 0;

    if (fileContents[filePos][j] == '0') {
        // 1.1 Have read all sets of data.
        filePos = 0; // Update the current position of the file.
        ui->statusbar->showMessage("全部資料已讀取完畢，下次將重頭讀取檔案。", 3000);
        return;
    } else {
        // 1.2 Read a next set of data.
        Point point;
        unsigned int setSize = stoi(fileContents[filePos]);
        int i = filePos + 1;
        for (; i <= filePos + setSize; ++i) {
            // 1.2.1 Find the position of blank.
            int blankPos = -1;
            while (fileContents[i][++blankPos] != ' ') {}

            // 1.2.2 Add x value and y value
            point.x = stoi(fileContents[i].substr(0, blankPos));
            point.y = stoi(fileContents[i].substr(blankPos + 1, fileContents[i].size() - 1 - blankPos));
            scene->addPoint(point);
        }
        // 1.2.3 Update the filePos.
        filePos = i;
        ui->statusbar->showMessage("已讀取下一組資料", 3000);
    }
}


void MainWindow::on_readOutput_clicked()
{
    QString filter = "Text (*.txt)";

    // 1. Get the name of a new input file.
    QString newFileName = QFileDialog::getOpenFileName(this, "開啟輸出檔", defaultPath, filter);
    if (newFileName == "") {
        ui->statusbar->showMessage("沒有選擇任何檔案", 3000);
        return;
    } else {
        // 1.1 Update the fileName.
        fileName = newFileName;
        defaultPath = getFilePath(newFileName);
        ui->statusbar->showMessage("已經選擇 " + shortFileName(newFileName), 3000);
    }

    // 2. Open the file and check if it is opened.
    ifstream file(fileName.toLocal8Bit().constData());
    if (!file){
        ui->statusbar->showMessage("沒有開啟任何檔案", 3000);
        return;
    } else {
        // Reset.
        on_clear_clicked();
        filePos = 0;
        ui->statusbar->showMessage("已經選擇 " + shortFileName(newFileName), 3000);
    }

    // 3. Read all points and edges in the file.
    string str;
    Point point;
    Edge edge;
    int x1, y1, x2, y2;
    while (file >> str) {
        if (str == "P") {
            file >> point.x >> point.y;
            if ( point.isInRange(width, height) ) {scene->addPoint(point);}
        } else if (str == "E") {
            file >> x1 >> y1 >> x2 >> y2;
            edge.setVal(x1, y1, x2, y2);
            if ( edge.isInRange(width, height) ) {
                scene->addEdge(edge);
            } else {
                edge.fitRange(width, height);
                scene->addEdge(edge);
            }
        } else {
            ui->statusbar->showMessage("此檔案不符合輸出檔格式", 3000);
            return;
        }
    }

    // 4. Close the file.
    file.close();
}


void MainWindow::on_writeOutput_clicked()
{
    QString filter = "Text (*.txt)";

    // 1. Get the name of a new output file.
    QString outputFileName = QFileDialog::getSaveFileName(this, "輸出文字檔", defaultPath, filter);
    if (outputFileName == "") {
        ui->statusbar->showMessage("輸出失敗，因為沒有給予檔案名稱", 3000);
        return;
    }

    // 2. Open the output file and check if it is opened.
    ofstream file(outputFileName.toLocal8Bit().constData());
    if (!file){
        ui->statusbar->showMessage("輸出失敗", 3000);
        return;
    } else {
        ui->statusbar->showMessage("已經輸出 " + shortFileName(outputFileName), 3000);
    }

    // 3. Lexical order.
    sort(points.begin(), points.end(), Point::cmp);
    for(auto e : voronoi.voronoiLines) {e.lexicalOrder();}
    sort(voronoi.voronoiLines.begin(), voronoi.voronoiLines.end(), Line::cmp);

    // 4. Writes all points and egdes into a new output file.
    for (const auto p : points) {file << p.toString() << endl;} // Points
    for (const auto e : voronoi.voronoiLines) {file << e.toString() << endl;} // Edges

    // 5. Close the file.
    file.close();
}


void MainWindow::on_run_clicked()
{
    // 0. Check if there are points.
    if (points.size() == 0) {
        ui->statusbar->showMessage("沒有任何點", 3000);
        return;
    }
    stepNum = -1;

    // 1. Run.
    VoronoiDiagram VD(width, height, points);
    voronoi = VD.run();
    sort(voronoi.voronoiLines.begin(), voronoi.voronoiLines.end(), Line::cmp);
    scene->update();
    ui->statusbar->showMessage("執行完畢", 3000);
}


void MainWindow::on_stepByStep_clicked()
{
    // 0. Check if there are points.
    if (points.size() == 0) {
        ui->statusbar->showMessage("沒有任何點", 3000);
        return;
    }

    // 1. If the size of pointe is less and equal 3, call on_run_clicked().
    if (points.size() <= 3) {
        on_run_clicked();
        return;
    }

    // 2. Run step by step.
    if (stepNum == -1) {
        VoronoiDiagram VD(width, height, points);
        voronoi = VD.runStepByStep(steps);
        sort(voronoi.voronoiLines.begin(), voronoi.voronoiLines.end(), Line::cmp);
        stepNum = 1;
    }

    // 3. Print the situation of the current step.
    const int pointsSize = points.size();
    if (stepNum != -1) {
        scene->update(stepNum);
    }
}


void MainWindow::on_addPoint_clicked()
{
    scene->addPoint(ui->xValue->text().toInt(), ui->yValue->text().toInt());
}


void MainWindow::on_generPoints_clicked()
{
    // 1. Random engine and distridution.
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> xDst(0, width);
    uniform_int_distribution<int> yDst(0, height);

    // 2. Generator points.
    // 2.1 Clear all points.
    const int addPointsSize = ui->addPointsSize->text().toInt();
    scene->clearScene();

    // 2.2 Generator different points.
    ADD:
    while (points.size() < addPointsSize) {
        points.push_back(Point( xDst(generator), yDst(generator) ));
    }
    sortAndUnique(points);
    if (points.size() < addPointsSize) {goto ADD;} // Double cheack the size of all points.

    // 3. Add points into scene.
    for (const auto &p : points) {scene->addPoint(p, 0);}
}


QString MainWindow::shortFileName(const QString &fileName)
{
    int i = fileName.size();
    while (fileName[--i] != '/' && i >= 0) {} // Find the last index that the character is '/'.
    ++i;
    return fileName.right(fileName.size() - i); // Substring
}

QString MainWindow::getFilePath(const QString &fileName)
{
    int i = fileName.size();
    while (fileName[--i] != '/' && i >= 0) {} // Find the last index that the character is '/'.
    return fileName.left(i); // Substring
}

void MainWindow::printFileContents()
{
    for (const auto &con : fileContents) {cout << con << endl;}
}






void MainWindow::on_actionReadInput_triggered()
{
    on_readInput_clicked();
}


void MainWindow::on_actionReadOutput_triggered()
{
    on_readOutput_clicked();
}


void MainWindow::on_actionWriteOutput_triggered()
{
    on_writeOutput_clicked();
}


void MainWindow::on_actionRun_triggered()
{
    on_run_clicked();
}


void MainWindow::on_actionNextData_triggered()
{
    on_nextData_clicked();
}


void MainWindow::on_actionStepByStep_triggered()
{
    on_stepByStep_clicked();
}


void MainWindow::on_actionClear_triggered()
{
    on_clear_clicked();
}


void MainWindow::on_actionInformation_triggered()
{
    info->exec();
}


void MainWindow::on_actionQtVersion_triggered()
{
    QMessageBox::aboutQt(this, "Qt版本資訊");
}

// $LAN=C++$
// Author : 呂宗霖 Tsung-Lin Lu
// Student ID : M093040114
// Date : 2021/12/19

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "DiagramScene.h"
#include "DataStructure.h"
#include "Information.h"

#include <QMainWindow>
#include <QString>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_clear_clicked();
    void on_readInput_clicked();
    void on_nextData_clicked();
    void on_readOutput_clicked();
    void on_writeOutput_clicked();
    void on_run_clicked();
    void on_stepByStep_clicked();
    void on_addPoint_clicked();
    void on_generPoints_clicked();

    void on_actionReadInput_triggered();
    void on_actionReadOutput_triggered();
    void on_actionWriteOutput_triggered();
    void on_actionRun_triggered();
    void on_actionNextData_triggered();
    void on_actionStepByStep_triggered();
    void on_actionClear_triggered();
    void on_actionInformation_triggered();
    void on_actionQtVersion_triggered();

private:
    // Data members
    Ui::MainWindow *ui;
    const int width;
    const int height;
    QString defaultPath;

    Information* info;
    DiagramScene* scene;
    QString fileName;
    unsigned int filePos;
    std::vector<Point> points;
    Record voronoi;
    std::vector<Record> steps;
    int stepNum;
    std::vector<std::string> fileContents;

    // Member functions
    QString shortFileName(const QString &fileName);
    QString getFilePath(const QString &fileName);
    bool cmpPoint(Point a, Point b);
    bool cmpEdge(Edge a, Edge b);
    void printFileContents();
};
#endif // MAINWINDOW_H
<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>MainWindow</class>
 <widget class="QMainWindow" name="MainWindow">
  <property name="geometry">
   <rect>
    <x>0</x>
    <y>0</y>
    <width>1040</width>
    <height>740</height>
   </rect>
  </property>
  <property name="windowTitle">
   <string>MainWindow</string>
  </property>
  <widget class="QWidget" name="centralwidget">
   <widget class="QGraphicsView" name="graphicsView">
    <property name="geometry">
     <rect>
      <x>20</x>
      <y>20</y>
      <width>650</width>
      <height>650</height>
     </rect>
    </property>
   </widget>
   <widget class="QGroupBox" name="groupBox_2">
    <property name="geometry">
     <rect>
      <x>890</x>
      <y>180</y>
      <width>131</width>
      <height>121</height>
     </rect>
    </property>
    <property name="title">
     <string>檔案</string>
    </property>
    <widget class="QWidget" name="layoutWidget">
     <property name="geometry">
      <rect>
       <x>0</x>
       <y>20</y>
       <width>121</width>
       <height>100</height>
      </rect>
     </property>
     <layout class="QVBoxLayout" name="verticalLayout_3">
      <item>
       <widget class="QPushButton" name="readInput">
        <property name="text">
         <string>讀取輸入檔</string>
        </property>
       </widget>
      </item>
      <item>
       <widget class="QPushButton" name="readOutput">
        <property name="text">
         <string>讀取輸出檔</string>
        </property>
       </widget>
      </item>
      <item>
       <widget class="QPushButton" name="writeOutput">
        <property name="text">
         <string>輸出文字檔</string>
        </property>
       </widget>
      </item>
     </layout>
    </widget>
   </widget>
   <widget class="QGroupBox" name="groupBox_3">
    <property name="geometry">
     <rect>
      <x>890</x>
      <y>20</y>
      <width>131</width>
      <height>161</height>
     </rect>
    </property>
    <property name="title">
     <string>執行</string>
    </property>
    <widget class="QWidget" name="layoutWidget">
     <property name="geometry">
      <rect>
       <x>0</x>
       <y>20</y>
       <width>122</width>
       <height>134</height>
      </rect>
     </property>
     <layout class="QVBoxLayout" name="verticalLayout_4">
      <item>
       <widget class="QPushButton" name="run">
        <property name="text">
         <string>執行</string>
        </property>
       </widget>
      </item>
      <item>
       <widget class="QPushButton" name="nextData">
        <property name="text">
         <string>下一組資料</string>
        </property>
       </widget>
      </item>
      <item>
       <widget class="QPushButton" name="stepByStep">
        <property name="text">
         <string>一步一步執行</string>
        </property>
       </widget>
      </item>
      <item>
       <widget class="QPushButton" name="clear">
        <property name="text">
         <string>清空畫面</string>
        </property>
       </widget>
      </item>
     </layout>
    </widget>
   </widget>
   <widget class="QGroupBox" name="groupBox_4">
    <property name="geometry">
     <rect>
      <x>690</x>
      <y>20</y>
      <width>181</width>
      <height>281</height>
     </rect>
    </property>
    <property name="title">
     <string>添加點</string>
    </property>
    <widget class="QWidget" name="layoutWidget">
     <property name="geometry">
      <rect>
       <x>10</x>
       <y>40</y>
       <width>161</width>
       <height>181</height>
      </rect>
     </property>
     <layout class="QFormLayout" name="formLayout">
      <item row="0" column="0">
       <widget class="QLabel" name="label">
        <property name="text">
         <string>滑鼠位置</string>
        </property>
       </widget>
      </item>
      <item row="0" column="1">
       <widget class="QLabel" name="mousePos">
        <property name="text">
         <string>(0,  0 )</string>
        </property>
       </widget>
      </item>
      <item row="1" column="0">
       <widget class="QLabel" name="label_2">
        <property name="text">
         <string>Ｘ</string>
        </property>
       </widget>
      </item>
      <item row="1" column="1">
       <widget class="QLineEdit" name="xValue">
        <property name="placeholderText">
         <string/>
        </property>
       </widget>
      </item>
      <item row="2" column="0">
       <widget class="QLabel" name="label_3">
        <property name="text">
         <string>Ｙ</string>
        </property>
       </widget>
      </item>
      <item row="2" column="1">
       <widget class="QLineEdit" name="yValue">
        <property name="placeholderText">
         <string/>
        </property>
       </widget>
      </item>
      <item row="3" column="1">
       <widget class="QPushButton" name="addPoint">
        <property name="text">
         <string>添加點</string>
        </property>
       </widget>
      </item>
      <item row="4" column="0">
       <widget class="QLabel" name="label_4">
        <property name="text">
         <string>數量</string>
        </property>
       </widget>
      </item>
      <item row="4" column="1">
       <widget class="QLineEdit" name="addPointsSize"/>
      </item>
      <item row="5" column="1">
       <widget class="QPushButton" name="generPoints">
        <property name="text">
         <string>隨機產生</string>
        </property>
       </widget>
      </item>
     </layout>
    </widget>
   </widget>
   <widget class="QWidget" name="layoutWidget">
    <property name="geometry">
     <rect>
      <x>690</x>
      <y>300</y>
      <width>331</width>
      <height>371</height>
     </rect>
    </property>
    <layout class="QVBoxLayout" name="verticalLayout_2">
     <item>
      <widget class="QLabel" name="pointsSize">
       <property name="text">
        <string>點資料：0點</string>
       </property>
      </widget>
     </item>
     <item>
      <widget class="QTextBrowser" name="pointsData"/>
     </item>
     <item>
      <widget class="QLabel" name="edgesSize">
       <property name="text">
        <string>邊資料：0邊</string>
       </property>
      </widget>
     </item>
     <item>
      <widget class="QTextBrowser" name="edgesData"/>
     </item>
    </layout>
   </widget>
  </widget>
  <widget class="QMenuBar" name="menubar">
   <property name="geometry">
    <rect>
     <x>0</x>
     <y>0</y>
     <width>1040</width>
     <height>22</height>
    </rect>
   </property>
   <widget class="QMenu" name="menu">
    <property name="title">
     <string>檔案</string>
    </property>
    <addaction name="actionReadInput"/>
    <addaction name="actionReadOutput"/>
    <addaction name="actionWriteOutput"/>
   </widget>
   <widget class="QMenu" name="menu_2">
    <property name="title">
     <string>執行</string>
    </property>
    <addaction name="actionRun"/>
    <addaction name="actionNextData"/>
    <addaction name="actionStepByStep"/>
    <addaction name="actionClear"/>
   </widget>
   <widget class="QMenu" name="menu_3">
    <property name="title">
     <string>資訊</string>
    </property>
    <addaction name="actionInformation"/>
    <addaction name="actionQtVersion"/>
   </widget>
   <addaction name="menu"/>
   <addaction name="menu_2"/>
   <addaction name="menu_3"/>
  </widget>
  <widget class="QStatusBar" name="statusbar"/>
  <action name="actionReadInput">
   <property name="text">
    <string>讀取輸入檔</string>
   </property>
  </action>
  <action name="actionReadOutput">
   <property name="text">
    <string>讀取輸出檔</string>
   </property>
  </action>
  <action name="actionWriteOutput">
   <property name="text">
    <string>輸出文字檔</string>
   </property>
  </action>
  <action name="actionRun">
   <property name="text">
    <string>執行</string>
   </property>
  </action>
  <action name="actionNextData">
   <property name="text">
    <string>下一組資料</string>
   </property>
  </action>
  <action name="actionStepByStep">
   <property name="text">
    <string>一步一步執行</string>
   </property>
  </action>
  <action name="actionClear">
   <property name="text">
    <string>清空畫面</string>
   </property>
  </action>
  <action name="actionInformation">
   <property name="text">
    <string>軟體資訊</string>
   </property>
  </action>
  <action name="actionQtVersion">
   <property name="text">
    <string>Qt版本資訊</string>
   </property>
  </action>
 </widget>
 <resources/>
 <connections/>
</ui>
