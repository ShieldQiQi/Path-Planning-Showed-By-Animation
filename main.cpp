#include "mainwindow.h"

#include <QApplication>

MainWindow *w;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
//    MainWindow w;

    w = new MainWindow();
    w->setGeometry(400,100,840,900);
    w->show();
    return a.exec();
}
