// A main window which contains all of the interfaces that make up the Dashboard.
// Written by Tiger Sachse.

#include <QWidget>
#include <QCloseEvent>
#include "main_window.h"

// Initialize this MainWindow class and set up the UI.
MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
    setupUi(this);
}

// When the main window closes, emit a SIGNAL.
void MainWindow::closeEvent(QCloseEvent* closeEvent) {
    Q_EMIT closed();
    closeEvent->accept();
}
