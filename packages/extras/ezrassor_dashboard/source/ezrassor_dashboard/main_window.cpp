// The MainWindow class holds all of the interfaces for the Dashboard.
// Written by Tiger Sachse.
#include <QWidget>
#include "main_window.h"

// Initialize this MainWindow class and set up the UI.
MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
    setupUi(this);
    connect(
        connectButton,
        SIGNAL(clicked()),
        this,
        SLOT(handleConnectionRequest())
    );
}

// When a connection is requested, emit a signal with the appropriate Master URI.
void MainWindow::handleConnectionRequest(void) {
    Q_EMIT connectionRequested(masterURILineEdit->text().toStdString());
}
