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
