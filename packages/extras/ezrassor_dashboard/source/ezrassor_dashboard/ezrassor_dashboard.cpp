#include <QApplication>
#include "main_window.h"
#include "about_window.h"

int main(int argumentCount, char** argumentVector) {
    QApplication application(argumentCount, argumentVector);

    MainWindow mainWindow;
    mainWindow.show();
    AboutWindow aboutWindow;
    aboutWindow.show();

    return application.exec();
}
