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

