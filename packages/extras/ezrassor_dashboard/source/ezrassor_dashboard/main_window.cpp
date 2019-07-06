// The MainWindow class holds all of the interfaces for the Dashboard.
// Written by Tiger Sachse.
#include <QWidget>
#include <QCloseEvent>
#include "main_window.h"

// Initialize this MainWindow class and set up the UI.
MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
    setupUi(this);
}

void MainWindow::closeEvent(QCloseEvent* closeEvent) {
    Q_EMIT closed();
    closeEvent->accept();
}
