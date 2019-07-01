#include "welcome_window.h"

WelcomeWindow::WelcomeWindow(QWidget *parent) : QWidget(parent) {
    setupUi(this);
}

void WelcomeWindow::on_ConnectButton_pressed(void) {
    printf("hello world\n");
}
