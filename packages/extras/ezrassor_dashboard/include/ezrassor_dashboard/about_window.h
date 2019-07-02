#ifndef ABOUT_WINDOW_HEADER
#define ABOUT_WINDOW_HEADER

#include <QWidget>
#include "ui_about_window.h"

class AboutWindow : public QWidget, private Ui::AboutWindow {
    Q_OBJECT
    public:
        AboutWindow(QWidget *parent = NULL);
};
#endif
