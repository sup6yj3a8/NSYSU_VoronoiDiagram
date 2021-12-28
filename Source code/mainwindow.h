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
