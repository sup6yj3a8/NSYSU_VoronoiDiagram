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
