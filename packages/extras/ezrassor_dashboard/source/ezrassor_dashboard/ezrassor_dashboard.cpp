// The main entry point to the EZ-RASSOR Dashboard. This file configures the
// Dashboard components and launches the GUI.
// Inspired by Chris Taliaferro, Lucas Gonzalez, Sean Rapp, and Samuel Lewis.
// Written by Tiger Sachse.

#include "help_window.h"
#include "main_window.h"
#include "QApplication"
#include "rviz_plugin.h"
#include "topic_translator.h"

// The main entry point to the EZ-RASSOR Dashboard.
int main(int argumentCount, char** argumentVector) {
    //RvizPlugin* orientationViewer;
    //RvizPlugin* pointCloudViewer;
    //RvizPlugin* poseViewer;
    /*
    orientationViewer = new RvizPlugin(
        mainWindow.orientationFeedWidget,
        RvizPlugin::ORIENTATION_VIEWER,
        "imu"
    );*/

    /*
    pointCloudViewer = new RvizPlugin(
        mainWindow.pointCloudFeedWidget,
        RvizPlugin::POINT_CLOUD_VIEWER,
        "points2"
    );

    poseViewer = new RvizPlugin(
        mainWindow.poseFeedWidget,
        RvizPlugin::POSE_VIEWER
    );
    */

    QApplication application(argumentCount, argumentVector);
    MainWindow mainWindow;
    HelpWindow helpWindow;
    mainWindow.show();

    // If the main window closes, the whole program should quit.
    application.connect(
        &mainWindow,
        SIGNAL(closed(void)),
        &application,
        SLOT(quit(void))
    );

    // Create a topic translator. Start its thread If it initializes properly,
    // otherwise show a help menu.
    TopicTranslator topicTranslator(
        argumentCount,
        argumentVector,
        100,
        "ezrassor_dashboard",
        "dashboard",
        "imu",
        "rosout_agg",
        "memory_usage",
        "cpu_usage",
        "battery_remaining",
        "left/image_raw",
        "right/image_raw",
        "disparity"
    );
    if (topicTranslator.initialized()) {
        topicTranslator.start();

        // Connect the refresh button to the namespace selection combo box.
        // This connect() function uses the newer Qt signal/slot syntax (because
        // it supports calling lambdas). The lambda here ensures that the combo
        // box is cleared *before* new namespaces are discovered. If two
        // separate connect() functions were used instead clear() would not be
        // guaranteed to come before discoverNamespaces().
        application.connect(
            mainWindow.refreshButton,
            &QPushButton::pressed,
            [&]() {
                mainWindow.namespaceSelectionComboBox->clear();
                topicTranslator.discoverNamespaces();
            }
        );

        // When namespaces are discovered, add them to the namespace selection
        // combo box. The new signal/slot syntax enables calling a non-slot
        // function here.
        application.connect(
            &topicTranslator,
            &TopicTranslator::namespacesDiscovered,
            mainWindow.namespaceSelectionComboBox,
            &QComboBox::addItems
        );
        application.connect(
            mainWindow.namespaceSelectionComboBox,
            SIGNAL(currentTextChanged(const QString&)),
            &topicTranslator,
            SLOT(updateNamespace(const QString&))
        );

        // Clear everything each time the subscribers are re-initialized.
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.xOrientationLineEdit,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.yOrientationLineEdit,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.zOrientationLineEdit,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.xAngularVelocityLineEdit,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.yAngularVelocityLineEdit,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.zAngularVelocityLineEdit,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.xLinearAccelerationLineEdit,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.yLinearAccelerationLineEdit,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.zLinearAccelerationLineEdit,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.processorUsageBar,
            SLOT(reset(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.memoryUsageBar,
            SLOT(reset(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.batteryRemainingBar,
            SLOT(reset(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.leftCameraFeedLabel,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.rightCameraFeedLabel,
            SLOT(clear(void))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(subscribersInitialized(void)),
            mainWindow.disparityMapFeedLabel,
            SLOT(clear(void))
        );

        // Connect IMU data to the GUI.
        application.connect(
            &topicTranslator,
            SIGNAL(xOrientationReceived(const QString&)),
            mainWindow.xOrientationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(yOrientationReceived(const QString&)),
            mainWindow.yOrientationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(zOrientationReceived(const QString&)),
            mainWindow.zOrientationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(xAngularVelocityReceived(const QString&)),
            mainWindow.xAngularVelocityLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(yAngularVelocityReceived(const QString&)),
            mainWindow.yAngularVelocityLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(zAngularVelocityReceived(const QString&)),
            mainWindow.zAngularVelocityLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(xLinearAccelerationReceived(const QString&)),
            mainWindow.xLinearAccelerationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(yLinearAccelerationReceived(const QString&)),
            mainWindow.yLinearAccelerationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(zLinearAccelerationReceived(const QString&)),
            mainWindow.zLinearAccelerationLineEdit,
            SLOT(setText(const QString&))
        );

        // Connect log messages to the GUI.
        application.connect(
            &topicTranslator,
            SIGNAL(logDataReceived(const QString&)),
            mainWindow.rosoutTextEdit,
            SLOT(append(const QString&))
        );

        // Connect status monitor data to the GUI.
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

        // Connect camera feeds to the GUI.
        application.connect(
            &topicTranslator,
            SIGNAL(leftCameraImageReceived(const QPixmap&)),
            mainWindow.leftCameraFeedLabel,
            SLOT(setPixmap(const QPixmap&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(rightCameraImageReceived(const QPixmap&)),
            mainWindow.rightCameraFeedLabel,
            SLOT(setPixmap(const QPixmap&))
        );
        application.connect(
            &topicTranslator,
            SIGNAL(disparityMapImageReceived(const QPixmap&)),
            mainWindow.disparityMapFeedLabel,
            SLOT(setPixmap(const QPixmap&))
        );

        topicTranslator.discoverNamespaces();
    }
    else {
        helpWindow.show();
        mainWindow.refreshButton->setDisabled(true);
        mainWindow.namespaceSelectionComboBox->setDisabled(true);
    }

    // Run the application.
    return application.exec();

    //delete orientationViewer;
    //delete pointCloudViewer;
    //delete poseViewer;
}
