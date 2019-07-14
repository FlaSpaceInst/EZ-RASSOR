// A ROS node that translates data from ROS topics into data that Qt can work with.
// Inspired by Chris Taliaferro, Samuel Lewis, and Lucas Gonzalez.
// Written by Tiger Sachse and Sean Rapp.

#include <QImage>
#include <QThread>
#include <QPixmap>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "topic_translator.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

// Initialize the topic translator with a bunch of constants.
TopicTranslator::TopicTranslator(
    int& argumentCount,
    char** argumentVector,
    const std::string& nodeName,
    int queueSize,
    const std::string& memoryUsageTopic,
    const std::string& processorUsageTopic,
    const std::string& batteryRemainingTopic,
    const std::string& leftCameraImageTopic,
    const std::string& rightCameraImageTopic)
    : queueSize(queueSize),
      memoryUsageTopic(memoryUsageTopic),
      processorUsageTopic(processorUsageTopic),
      batteryRemainingTopic(batteryRemainingTopic),
      leftCameraImageTopic(leftCameraImageTopic),
      rightCameraImageTopic(rightCameraImageTopic) {

    currentLeftCameraImage = QPixmap();
    currentRightCameraImage = QPixmap();

    // Attempt to contact ROS Master. If ROS Master can't be reached, throw
    // an error.
    ros::init(argumentCount, argumentVector, nodeName);
    if (!ros::master::check()) {
        throw TRANSLATOR_INITIALIZATION_FAILED;
    }
}

// Destroy this translator. Hangs if destroyed right after creation.
TopicTranslator::~TopicTranslator(void) {
    ros::shutdown();
    ros::waitForShutdown();
    wait();
}

// Run the translator ROS node.
void TopicTranslator::run(void) {
    ros::NodeHandle nodeHandle;
    ros::Subscriber processorUsageSubscriber = nodeHandle.subscribe(
        processorUsageTopic,
        queueSize,
        &TopicTranslator::handleProcessorData,
        this
    );
    ros::Subscriber memoryUsageSubscriber = nodeHandle.subscribe(
        memoryUsageTopic,
        queueSize,
        &TopicTranslator::handleMemoryData,
        this
    );
    ros::Subscriber batteryRemainingSubscriber = nodeHandle.subscribe(
        batteryRemainingTopic,
        queueSize,
        &TopicTranslator::handleBatteryData,
        this
    );
    ros::Subscriber leftCameraImageSubscriber = nodeHandle.subscribe(
        leftCameraImageTopic,
        queueSize,
        &TopicTranslator::handleLeftCameraImage,
        this
    );
    ros::Subscriber rightCameraImageSubscriber = nodeHandle.subscribe(
        rightCameraImageTopic,
        queueSize,
        &TopicTranslator::handleRightCameraImage,
        this
    );

    ros::spin();
}

// Handle incoming memory data from ROS.
void TopicTranslator::handleMemoryData(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT memoryDataReceived((int) message->data);
}

// Handle incoming battery data from ROS.
void TopicTranslator::handleBatteryData(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT batteryDataReceived((int) message->data);
}

// Handle incoming processor data from ROS.
void TopicTranslator::handleProcessorData(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT processorDataReceived((int) message->data);
}

// Handle incoming left camera images from ROS.
void TopicTranslator::handleLeftCameraImage(const sensor_msgs::ImageConstPtr& message) {
    processCameraImage(message, &currentLeftCameraImage);
    Q_EMIT leftCameraImageReceived(currentLeftCameraImage);
}

// Handle incoming right camera images from ROS.
void TopicTranslator::handleRightCameraImage(const sensor_msgs::ImageConstPtr& message) {
    processCameraImage(message, &currentRightCameraImage);
    Q_EMIT rightCameraImageReceived(currentRightCameraImage);
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
