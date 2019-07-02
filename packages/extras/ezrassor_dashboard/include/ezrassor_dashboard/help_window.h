#ifndef HELP_WINDOW_HEADER
#define HELP_WINDOW_HEADER

#include <QWidget>
#include "ui_help_window.h"

class HelpWindow : public QWidget, private Ui::HelpWindow {
    Q_OBJECT
    public:
        HelpWindow(QWidget *parent = NULL);
};
#endif
