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
