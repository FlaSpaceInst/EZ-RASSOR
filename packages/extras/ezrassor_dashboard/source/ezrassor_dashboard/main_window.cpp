#include <QWidget>
#include "main_window.h"

MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
    setupUi(this);
    connect(
        ConnectButton,
        SIGNAL(clicked()),
        this,
        SLOT(handleConnectionRequest())
    );
}

void MainWindow::handleConnectionRequest(void) {
    Q_EMIT connectionRequested(ROSMasterURILineEdit->text().toStdString());
}
