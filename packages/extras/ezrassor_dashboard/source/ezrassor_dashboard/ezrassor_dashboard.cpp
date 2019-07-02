#include <QApplication>
#include "main_window.h"
#include "help_window.h"

int main(int argument_count, char** argument_vector) {
    QApplication application(argument_count, argument_vector);

    MainWindow main_window;
    main_window.show();
    HelpWindow help_window;
    help_window.show();

    return application.exec();
}
