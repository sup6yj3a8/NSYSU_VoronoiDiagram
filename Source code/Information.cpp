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

