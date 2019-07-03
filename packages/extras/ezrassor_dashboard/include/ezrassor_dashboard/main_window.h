#ifndef MAIN_WINDOW_HEADER
#define MAIN_WINDOW_HEADER

#include <QWidget>
#include "ui_main_window.h"

class MainWindow : public QWidget, public Ui_MainWindow {
    Q_OBJECT
    public:
        MainWindow(QWidget *parent = NULL);
    signals:
        void connectionRequested(const std::string&);
    public slots:
        void handleConnectionRequest();
};
#endif
