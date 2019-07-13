// The main entry point of the EZ-RASSOR Dashboard. This file configures the
// Dashboard components and launches the GUI.
// Written by Tiger Sachse.
#include <QApplication>
#include "main_window.h"
#include "help_window.h"
#include "topic_translator.h"

// The main entry point of the EZ-RASSOR Dashboard.
int main(int argumentCount, char** argumentVector) {

    // Initialize the application and its main window, and its topic translator.
    QApplication application(argumentCount, argumentVector);
    MainWindow mainWindow;
    HelpWindow helpWindow;
    TopicTranslator* topicTranslator;

    mainWindow.show();

    application.connect(
        &mainWindow,
        SIGNAL(closed(void)),
        &application,
        SLOT(quit(void))
    );

    try {
        topicTranslator = new TopicTranslator(
            argumentCount,
            argumentVector,
            "dashboard",
            100,
            "memory_usage",
            "cpu_usage",
            "battery_remaining",
            "left/image_raw",
            "right/image_raw"
        );

        // Show relevant data from ROS topics in the GUI.
        application.connect(
            topicTranslator,
            SIGNAL(processorDataReceived(int)),
            mainWindow.processorUsageBar,
            SLOT(setValue(int))
        );
        application.connect(
            topicTranslator,
            SIGNAL(memoryDataReceived(int)),
            mainWindow.memoryUsageBar,
            SLOT(setValue(int))
        );
        application.connect(
            topicTranslator,
            SIGNAL(batteryDataReceived(int)),
            mainWindow.batteryRemainingBar,
            SLOT(setValue(int))
        );
        application.connect(
            topicTranslator,
            SIGNAL(leftCameraImageReceived(const QPixmap&)),
            mainWindow.leftCameraFeedLabel,
            SLOT(setPixmap(const QPixmap&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(rightCameraImageReceived(const QPixmap&)),
            mainWindow.rightCameraFeedLabel,
            SLOT(setPixmap(const QPixmap&))
        );

        topicTranslator->start();
    }
    catch (int TRANSLATOR_INITIALIZATION_FAILED) {
        helpWindow.show();
    }

    // Run the application.
    return application.exec();
}
