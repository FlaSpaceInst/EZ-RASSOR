// A ROS node that translates data from ROS topics into data that Qt can work with.
// Inspired by Chris Taliaferro, Samuel Lewis, and Lucas Gonzalez.
// Written by Tiger Sachse and Sean Rapp.

#include "cv_bridge/cv_bridge.h"
#include "fstream"
#include "QImage"
#include "QPixmap"
#include "QString"
#include "QThread"
#include "ros/console.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64.h"
#include "stereo_msgs/DisparityImage.h"
#include "string"
#include "topic_translator.h"
#include "unordered_set"

// Initialize the topic translator with a bunch of constants.
TopicTranslator::TopicTranslator(
    int& argumentCount,
    char** argumentVector,
    int queueSize,
    const std::string& packageName,
    const std::string& nodeName,
    const std::string& imuTopic,
    const std::string& logTopic,
    const std::string& memoryUsageTopic,
    const std::string& processorUsageTopic,
    const std::string& batteryRemainingTopic,
    const std::string& leftCameraImageTopic,
    const std::string& rightCameraImageTopic,
    const std::string& disparityMapImageTopic):
        queueSize(queueSize),
        imuTopic(imuTopic),
        logTopic(logTopic),
        memoryUsageTopic(memoryUsageTopic),
        processorUsageTopic(processorUsageTopic),
        batteryRemainingTopic(batteryRemainingTopic),
        leftCameraImageTopic(leftCameraImageTopic),
        rightCameraImageTopic(rightCameraImageTopic),
        disparityMapImageTopic(disparityMapImageTopic) {

    ros::init(argumentCount, argumentVector, nodeName);

    currentLeftCameraImage = QPixmap();//
    currentRightCameraImage = QPixmap();//
    currentDisparityMapImage = QPixmap();//

    // Read the whitelisted nodes into an unordered set.
    whitelistedNodes = std::unordered_set<std::string>();
    std::ifstream whitelistedNodesFile(
        ros::package::getPath(packageName) + WHITELISTED_NODES_RELATIVE_PATH
    );
    if (whitelistedNodesFile.is_open()) {
        std::string line;
        while (getline(whitelistedNodesFile, line)) {
            whitelistedNodes.insert(line);
        }
        whitelistedNodesFile.close();
    }
    else {
        ROS_WARN("Node whitelist not found. Expect log to be blank.");
    }
}

// Destroy this translator. Hangs if destroyed right after creation.
TopicTranslator::~TopicTranslator(void) {
    ros::shutdown();
    ros::waitForShutdown();
    wait();
}

// Confirm that the topic translator can talk to ROS Master.
bool TopicTranslator::initialized(void) {
    if (!ros::master::check()) {
        ROS_ERROR("ROS Master cannot be contacted.");

        return false;
    }
    else {
        return true;
    }
}

// Run the translator ROS node.
void TopicTranslator::run(void) {
    ros::NodeHandle nodeHandle;
    ros::Subscriber imuSubscriber = nodeHandle.subscribe(
        imuTopic,
        queueSize,
        &TopicTranslator::routeIMUData,
        this
    );
    ros::Subscriber logSubscriber = nodeHandle.subscribe(
        logTopic,
        queueSize,
        &TopicTranslator::routeLogData,
        this
    );
    ros::Subscriber processorUsageSubscriber = nodeHandle.subscribe(
        processorUsageTopic,
        queueSize,
        &TopicTranslator::routeProcessorData,
        this
    );
    ros::Subscriber memoryUsageSubscriber = nodeHandle.subscribe(
        memoryUsageTopic,
        queueSize,
        &TopicTranslator::routeMemoryData,
        this
    );
    ros::Subscriber batteryRemainingSubscriber = nodeHandle.subscribe(
        batteryRemainingTopic,
        queueSize,
        &TopicTranslator::routeBatteryData,
        this
    );
    ros::Subscriber leftCameraImageSubscriber = nodeHandle.subscribe(
        leftCameraImageTopic,
        queueSize,
        &TopicTranslator::routeLeftCameraImage,
        this
    );
    ros::Subscriber rightCameraImageSubscriber = nodeHandle.subscribe(
        rightCameraImageTopic,
        queueSize,
        &TopicTranslator::routeRightCameraImage,
        this
    );
    ros::Subscriber disparityMapImageSubscriber = nodeHandle.subscribe(
        disparityMapImageTopic,
        queueSize,
        &TopicTranslator::routeDisparityMapImage,
        this
    );

    ros::spin();
}

// Route incoming IMU data from ROS.
void TopicTranslator::routeIMUData(const sensor_msgs::Imu::ConstPtr& message) {
    Q_EMIT xOrientationReceived(
        QString().setNum(message->orientation.x, 'f', 3)
    );
    Q_EMIT yOrientationReceived(
        QString().setNum(message->orientation.y, 'f', 3)
    );
    Q_EMIT zOrientationReceived(
        QString().setNum(message->orientation.z, 'f', 3)
    );
    Q_EMIT xAngularVelocityReceived(
        QString().setNum(message->angular_velocity.x, 'f', 3)
    );
    Q_EMIT yAngularVelocityReceived(
        QString().setNum(message->angular_velocity.y, 'f', 3)
    );
    Q_EMIT zAngularVelocityReceived(
        QString().setNum(message->angular_velocity.z, 'f', 3)
    );
    Q_EMIT xLinearAccelerationReceived(
        QString().setNum(message->linear_acceleration.x, 'f', 3)
    );
    Q_EMIT yLinearAccelerationReceived(
        QString().setNum(message->linear_acceleration.y, 'f', 3)
    );
    Q_EMIT zLinearAccelerationReceived(
        QString().setNum(message->linear_acceleration.z, 'f', 3)
    );
}

// Route incoming log data from ROS.
void TopicTranslator::routeLogData(const rosgraph_msgs::Log::ConstPtr& message) {

    // Get the node name and node namespace from the message.
    int nodeNamePosition = (int) message->name.rfind('/');
    int namespaceNamePosition = (int) message->name.rfind(
        '/',
        nodeNamePosition - 1
    );
    std::string nodeName = message->name.substr(nodeNamePosition + 1);
    std::string namespaceName;
    if (nodeNamePosition <= 0) {
        namespaceName = "";
    }
    else {
        namespaceName = message->name.substr(
            namespaceNamePosition + 1,
            nodeNamePosition - namespaceNamePosition - 1
        );
    }

    // Filter out unwanted messages.
    if (namespaceName != "") {
        return;
    }
    if ((whitelistedNodes.find(nodeName)) == whitelistedNodes.end()) {
        return;
    }
    if ((int) message->level != LOG_LEVEL_INFO) {
        return;
    }

    // Create a QString with the node name prefixed to the message.
    QString logLine = QString::fromStdString(
        "[" + nodeName + "] " + message->msg
    );

    Q_EMIT logDataReceived(logLine);
}

// Route incoming memory data from ROS.
void TopicTranslator::routeMemoryData(
    const std_msgs::Float64::ConstPtr& message) {

    Q_EMIT memoryDataReceived((int) message->data);
}

// Route incoming battery data from ROS.
void TopicTranslator::routeBatteryData(
    const std_msgs::Float64::ConstPtr& message) {

    Q_EMIT batteryDataReceived((int) message->data);
}

// Route incoming processor data from ROS.
void TopicTranslator::routeProcessorData(
    const std_msgs::Float64::ConstPtr& message) {

    Q_EMIT processorDataReceived((int) message->data);
}

// Route incoming left camera images from ROS.
void TopicTranslator::routeLeftCameraImage(
    const sensor_msgs::ImageConstPtr& message) {

    processCameraImage(message, &currentLeftCameraImage);
    Q_EMIT leftCameraImageReceived(currentLeftCameraImage);
}

// Route incoming right camera images from ROS.
void TopicTranslator::routeRightCameraImage(
    const sensor_msgs::ImageConstPtr& message) {

    processCameraImage(message, &currentRightCameraImage);
    Q_EMIT rightCameraImageReceived(currentRightCameraImage);
}

// Route incoming disparity images from ROS. This function is 90% duplicated from
// processCameraImage() because C++ sucks. Someone who is better with C++ than I
// am should combine the two functions (I was salty when I wrote this).
void TopicTranslator::routeDisparityMapImage(
    const stereo_msgs::DisparityImage& message) {

    cv_bridge::CvImagePtr cvImage;
    try {
        cvImage = cv_bridge::toCvCopy(message.image);
    }
    catch (cv_bridge::Exception& cvException) {
        ROS_ERROR(
            "Exception handling disparity feed in cv_bridge: %s",
            cvException.what()
        );

        return;
    }

    cv::Mat monochromeImage = cv::Mat(cvImage->image.size(), CV_8UC1);
    cv::convertScaleAbs(cvImage->image, monochromeImage, 100, 0.0);

    QImage::Format imageFormat;
    if (monochromeImage.channels() == 3) {
        imageFormat = QImage::Format_RGB888;
    }
    else {
        imageFormat = QImage::Format_Grayscale8;
    }

    QImage qtImage(monochromeImage.cols, monochromeImage.rows, imageFormat);
    int lineSize = monochromeImage.cols * monochromeImage.channels();
    for (int row = 0; row < monochromeImage.rows; row++) {
        memcpy(qtImage.scanLine(row), monochromeImage.ptr(row), lineSize);
    }

    if (monochromeImage.channels() == 3) {
        currentDisparityMapImage = QPixmap::fromImage(qtImage.rgbSwapped());
    }
    else {
        currentDisparityMapImage = QPixmap::fromImage(qtImage);
    }

    Q_EMIT disparityMapImageReceived(currentDisparityMapImage);
}

// Transform images from a ROS format to a Qt format using cv_bridge.
void TopicTranslator::processCameraImage(
    const sensor_msgs::ImageConstPtr& message,
    QPixmap* currentCameraImage) {

    cv_bridge::CvImagePtr cvImage;
    try {
        cvImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& cvException) {
        ROS_ERROR(
            "Exception handling camera feed in cv_bridge: %s",
            cvException.what()
        );

        return;
    }

    QImage::Format imageFormat;
    if (cvImage->image.channels() == 3) {
        imageFormat = QImage::Format_RGB888;
    }
    else {
        imageFormat = QImage::Format_RGB32;
    }

    QImage qtImage(cvImage->image.cols, cvImage->image.rows, imageFormat);
    int lineSize = cvImage->image.cols * cvImage->image.channels();
    for (int row = 0; row < cvImage->image.rows; row++) {
        memcpy(qtImage.scanLine(row), cvImage->image.ptr(row), lineSize);
    }

    if (cvImage->image.channels() == 3) {
        *currentCameraImage = QPixmap::fromImage(qtImage.rgbSwapped());
    }
    else {
        *currentCameraImage = QPixmap::fromImage(qtImage);
    }
}
