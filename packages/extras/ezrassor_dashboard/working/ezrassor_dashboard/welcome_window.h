#ifndef WELCOME_WINDOW_HEADER
#define WELCOME_WINDOW_HEADER

#include <QWidget>
#include "ui_welcome_window.h"

class WelcomeWindow : public QWidget, private Ui::WelcomeWindow {
    Q_OBJECT
    public:
        WelcomeWindow(QWidget *parent = NULL);

    private slots:
        void on_ConnectButton_pressed();
};
#endif
