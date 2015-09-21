#include <QApplication>
#include "PlannerMonitor.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PlannerMonitor w;
    w.show();
    
    return a.exec();
}
