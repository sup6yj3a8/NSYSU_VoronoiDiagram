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
