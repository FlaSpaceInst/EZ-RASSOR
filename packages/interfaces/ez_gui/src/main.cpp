#include <QtGui>
#include <QApplication>
#include "../include/ez_gui/main_window.hpp"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    ez_gui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
