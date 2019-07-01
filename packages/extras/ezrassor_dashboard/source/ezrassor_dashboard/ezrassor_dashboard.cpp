#include <QApplication>
#include "main_window.h"
#include "welcome_window.h"

int main(int argument_count, char** argument_vector) {
    QApplication application(argument_count, argument_vector);

    MainWindow main_window;
    main_window.show();
    WelcomeWindow welcome_window;
    welcome_window.show();

    return application.exec();
}
