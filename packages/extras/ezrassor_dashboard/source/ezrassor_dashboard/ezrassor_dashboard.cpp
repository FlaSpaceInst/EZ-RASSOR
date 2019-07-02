#include <QApplication>
#include "main_window.h"
#include "about_window.h"

int main(int argument_count, char** argument_vector) {
    QApplication application(argument_count, argument_vector);

    MainWindow main_window;
    main_window.show();
    AboutWindow about_window;
    about_window.show();

    return application.exec();
}
