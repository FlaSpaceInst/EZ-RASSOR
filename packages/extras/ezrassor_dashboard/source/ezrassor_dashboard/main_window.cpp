// The MainWindow class holds all of the interfaces for the Dashboard.
// Written by Tiger Sachse.
#include <QWidget>
#include <QCloseEvent>
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

// Close the window and tell everyone about it.
void MainWindow::closeEvent(QCloseEvent* closeEvent) {
    Q_EMIT closed();
    closeEvent->accept();
}

// Change the GUI to disable new connections.
void MainWindow::finalizeConnection(void) {
    connectButton->setText("Disconnect");
    masterURILineEdit->setEnabled(false);
}

// Change the GUI to enable reconnections.
void MainWindow::finalizeDisconnection(void) {
    connectButton->setText("Connect");
    masterURILineEdit->setEnabled(true);
}

// Attempt to connect or disconnect from ROS based on the state of the connectButton.
void MainWindow::handleConnectButtonPress(void) {

    // I tried making the button checkable, but then it looked grayed out when
    // checked which I didn't like. Instead, I check the button's text to determine
    // its state. This isn't the cleanest but none of us are perfect, right?
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
