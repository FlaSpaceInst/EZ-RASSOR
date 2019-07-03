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
        "battery_usage",
        100
    );

    // Connect together all necessary signals and slots. This is how the GUI
    // and the topic translator are glued together.
    application.connect(
        &mainWindow,
        SIGNAL(connectionRequested(const std::string&)),
        &topicTranslator,
        SLOT(connectToMaster(const std::string&))
    );
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
        mainWindow.batteryUsageBar,
        SLOT(setValue(int))
    );

    // Show the windows.
    mainWindow.show();
    aboutWindow.show();

    // Run the application.
    return application.exec();
}
