// Defines a main window that holds all of the Dashboard's interfaces.
// Written by Tiger Sachse.

#ifndef MAIN_WINDOW_HEADER
#define MAIN_WINDOW_HEADER

#include "QCloseEvent"
#include "QWidget"
#include "ui_main_window.h"

class MainWindow : public QWidget, public Ui_mainWindow {
    Q_OBJECT

    public:
        MainWindow(QWidget* = NULL);

    signals:
        void closed(void);

    protected:
        void closeEvent(QCloseEvent*);
};

#endif
