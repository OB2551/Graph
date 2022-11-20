#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <QFile>


int main(int argc, char *argv[])
{
 
    QApplication a(argc, argv);
    QFile styleSheetFile("ElegantDark.qss");
    styleSheetFile.open(QFile::ReadOnly);
    QString styleSheet = QLatin1String(styleSheetFile.readAll());
    a.setStyleSheet(styleSheet);
    Graph w;
  
    w.show();
    return a.exec();
}
