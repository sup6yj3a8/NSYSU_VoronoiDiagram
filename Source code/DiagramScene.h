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
