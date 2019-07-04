// The main entry point of the EZ-RASSOR Dashboard. This file configures the
// Dashboard components and launches the GUI.
// Written by Tiger Sachse.
#include <QApplication>
#include "main_window.h"
#include "about_window.h"
#include "topic_translator.h"

// The main entry point of the EZ-RASSOR Dashboard.
int main(int argumentCount, char** argumentVector) {

    // Initialize the application, its windows, and its topic translator.
    QApplication application(argumentCount, argumentVector);
    MainWindow mainWindow;
    AboutWindow aboutWindow;
    TopicTranslator topicTranslator(
        "dashboard",
        "cpu_usage",
        "memory_usage",
        "battery_remaining",
        100
    );

    // Open a help window when the helpButton is pressed.
    application.connect(
        mainWindow.helpButton,
        SIGNAL(clicked(void)),
        &aboutWindow,
        SLOT(show(void))
    );

    // Close down the ROS node thread and aboutWindow if the mainWindow is closed.
    application.connect(
        &mainWindow,
        SIGNAL(closed(void)),
        &topicTranslator,
        SLOT(disconnectFromMaster(void))
    );
    application.connect(
        &mainWindow,
        SIGNAL(closed(void)),
        &aboutWindow,
        SLOT(close(void))
    );

    // Show relevant data from ROS topics in the GUI.
    application.connect(
        &topicTranslator,
        SIGNAL(processorDataReceived(int)),
        mainWindow.processorUsageBar,
        SLOT(setValue(int))
    );
    application.connect(
        &topicTranslator,
        SIGNAL(memoryDataReceived(int)),
        mainWindow.memoryUsageBar,
        SLOT(setValue(int))
    );
    application.connect(
        &topicTranslator,
        SIGNAL(batteryDataReceived(int)),
        mainWindow.batteryRemainingBar,
        SLOT(setValue(int))
    );

    // Handle connecting and disconnecting with the connectButton.
    application.connect(
        &mainWindow,
        SIGNAL(connectionRequested(const std::string&)),
        &topicTranslator,
        SLOT(connectToMaster(const std::string&))
    );
    application.connect(
        &mainWindow,
        SIGNAL(disconnectionRequested(void)),
        &topicTranslator,
        SLOT(disconnectFromMaster(void))
    );
    application.connect(
        &topicTranslator,
        SIGNAL(connectionSucceeded(void)),
        &mainWindow,
        SLOT(finalizeConnection(void))
    );
    application.connect(
        &topicTranslator,
        SIGNAL(connectionFailed(void)),
        &mainWindow,
        SLOT(recoverFromConnectionFailure(void))
    );
    application.connect(
        &topicTranslator,
        SIGNAL(disconnectionSucceeded(void)),
        &mainWindow,
        SLOT(finalizeDisconnection(void))
    );

    // Run the application.
    mainWindow.show();
    int returnValue = application.exec();

    // The application dies once all windows are closed. After this, signal for
    // the ROS node thread to disconnect from master and die.
    topicTranslator.disconnectFromMaster();
    while (topicTranslator.isRunning()) {};

    return returnValue;
}
