// Defines the AboutWindow class which shows a small instructions window.
// Written by Tiger Sachse.
#ifndef ABOUT_WINDOW_HEADER
#define ABOUT_WINDOW_HEADER
#include <QWidget>
#include "ui_about_window.h"

class AboutWindow : public QWidget, public Ui::AboutWindow {
    Q_OBJECT
    public:
        AboutWindow(QWidget *parent = NULL);
};
#endif
