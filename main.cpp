#include "mainwindow.h"

#include <QApplication>
#include "robot_common.h"
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    qRegisterMetaType<JointState>("JointState");
    qRegisterMetaType<ControlAlgorithm>("ControlAlgorithm");
    MainWindow w;
    w.show();
    return app.exec();
}
