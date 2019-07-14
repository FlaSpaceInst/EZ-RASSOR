// Define the HelpWindow that is displayed in cases of catastrophic failure.
// Written by Tiger Sachse.

#ifndef HELP_WINDOW_HEADER
#define HELP_WINDOW_HEADER

#include <QWidget>
#include "ui_help_window.h"

class HelpWindow : public QWidget, public Ui_helpWindow {
    Q_OBJECT
    public:
        HelpWindow(QWidget *parent = NULL);
};

#endif
