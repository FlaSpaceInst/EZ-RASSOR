// A help menu for when things go wrong.
// Written by Tiger Sachse.

#include <QWidget>
#include "help_window.h"

// Initialize the help menu with the help of a UI file.
HelpWindow::HelpWindow(QWidget *parent) : QWidget(parent) {
    setupUi(this);
}
