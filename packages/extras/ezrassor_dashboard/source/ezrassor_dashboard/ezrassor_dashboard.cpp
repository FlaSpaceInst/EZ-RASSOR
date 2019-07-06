// The main entry point of the EZ-RASSOR Dashboard. This file configures the
// Dashboard components and launches the GUI.
// Written by Tiger Sachse.
#include <QApplication>
#include "main_window.h"
#include "topic_translator.h"

// The main entry point of the EZ-RASSOR Dashboard.
int main(int argumentCount, char** argumentVector) {

    // Initialize the application and its main window, and its topic translator.
    QApplication application(argumentCount, argumentVector);
    MainWindow mainWindow;
    TopicTranslator topicTranslator(
        argumentCount,
        argumentVector,
        "dashboard",
        100,
        "memory_usage",
        "cpu_usage",
        "battery_remaining"
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

    // Run the application.
    mainWindow.show();

    return application.exec();
}
