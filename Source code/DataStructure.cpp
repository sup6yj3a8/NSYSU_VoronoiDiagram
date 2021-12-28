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

