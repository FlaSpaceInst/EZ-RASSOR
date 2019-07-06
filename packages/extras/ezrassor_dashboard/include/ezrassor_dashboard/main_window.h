// Defines the MainWindow class which holds all of the Dashboard's interfaces.
// Written by Tiger Sachse.
#ifndef MAIN_WINDOW_HEADER
#define MAIN_WINDOW_HEADER
#include <QWidget>
#include "ui_main_window.h"

class MainWindow : public QWidget, public Ui_mainWindow {
    Q_OBJECT
    public:
        MainWindow(QWidget *parent = NULL);
};
#endif
