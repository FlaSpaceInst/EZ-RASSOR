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

    // Initialize the application, its windows, and its topic translator.
    QApplication application(argumentCount, argumentVector);
    TopicTranslator* topicTranslator;
    RvizPlugin* orientationViewer;
    RvizPlugin* pointCloudViewer;
    RvizPlugin* poseViewer;
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

    try {
        topicTranslator = new TopicTranslator(
            argumentCount,
            argumentVector,
            100,
            "ezrassor_dashboard",
            "dashboard",
            "imu",
            "/rosout_agg",
            "memory_usage",
            "cpu_usage",
            "battery_remaining",
            "left/image_raw",
            "right/image_raw",
            "disparity"
        );

        /*
        orientationViewer = new RvizPlugin(
            mainWindow.orientationFeedWidget,
            RvizPlugin::ORIENTATION_VIEWER,
            "imu"
        );*/

        pointCloudViewer = new RvizPlugin(
            mainWindow.pointCloudFeedWidget,
            RvizPlugin::POINT_CLOUD_VIEWER,
            "points2"
        );

        poseViewer = new RvizPlugin(
            mainWindow.poseFeedWidget,
            RvizPlugin::POSE_VIEWER
        );

        // Show relevant data from ROS topics in the GUI.
        application.connect(
            topicTranslator,
            SIGNAL(xOrientationReceived(const QString&)),
            mainWindow.xOrientationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(yOrientationReceived(const QString&)),
            mainWindow.yOrientationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(zOrientationReceived(const QString&)),
            mainWindow.zOrientationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(xAngularVelocityReceived(const QString&)),
            mainWindow.xAngularVelocityLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(yAngularVelocityReceived(const QString&)),
            mainWindow.yAngularVelocityLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(zAngularVelocityReceived(const QString&)),
            mainWindow.zAngularVelocityLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(xLinearAccelerationReceived(const QString&)),
            mainWindow.xLinearAccelerationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(yLinearAccelerationReceived(const QString&)),
            mainWindow.yLinearAccelerationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(zLinearAccelerationReceived(const QString&)),
            mainWindow.zLinearAccelerationLineEdit,
            SLOT(setText(const QString&))
        );
        application.connect(
            topicTranslator,
            SIGNAL(logDataReceived(const QString&)),
            mainWindow.rosoutTextEdit,
            SLOT(append(const QString&))
        );
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
        application.connect(
            topicTranslator,
            SIGNAL(disparityMapImageReceived(const QPixmap&)),
            mainWindow.disparityMapFeedLabel,
            SLOT(setPixmap(const QPixmap&))
        );

        topicTranslator->start();
    }
    catch (int ROS_MASTER_UNREACHABLE) {
        helpWindow.show();
    }
    catch (int WHITELIST_FILE_MISSING) {
        helpWindow.show();
    }

    // Run the application.
    int exitCode = application.exec();

    //delete orientationViewer;
    delete pointCloudViewer;
    delete poseViewer;
    delete topicTranslator;

    return exitCode;
}
