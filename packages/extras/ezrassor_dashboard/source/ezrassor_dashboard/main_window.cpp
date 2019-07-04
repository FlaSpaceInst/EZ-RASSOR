// The MainWindow class holds all of the interfaces for the Dashboard.
// Written by Tiger Sachse.
#include <QWidget>
#include "main_window.h"

// Initialize this MainWindow class and set up the UI.
MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
    setupUi(this);
    connect(
        connectButton,
        SIGNAL(clicked(void)),
        this,
        SLOT(handleConnectButtonPress(void))
    );
}

void MainWindow::handleConnectButtonPress(void) {
    if (connectButton->text() == "Connect") {
        Q_EMIT connectionRequested(masterURILineEdit->text().toStdString());
    }
    else {
        Q_EMIT disconnectionRequested();
    }
}

#include <iostream>
void MainWindow::recoverFromConnectionFailure(void) {
    std::cout << "failed" << std::endl;
}

void MainWindow::finalizeConnection(void) {
    connectButton->setText("Disconnect");
    masterURILineEdit->setEnabled(false);
}

void MainWindow::finalizeDisconnection(void) {
    connectButton->setText("Connect");
    masterURILineEdit->setEnabled(true);
}
