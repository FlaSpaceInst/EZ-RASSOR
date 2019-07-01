#ifndef MAIN_WINDOW_HEADER
#define MAIN_WINDOW_HEADER

#include <QWidget>
#include "ui_main_window.h"

class MainWindow : public QWidget, private Ui::MainWindow {
    Q_OBJECT
    public:
        MainWindow(QWidget *parent = NULL);
};
#endif
