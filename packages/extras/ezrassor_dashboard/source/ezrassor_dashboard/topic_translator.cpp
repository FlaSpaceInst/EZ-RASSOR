// Written by Tiger Sachse.
#include <QThread>
#include <QPixmap>
#include <QImage>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "topic_translator.h"
#include "sensor_msgs/Image.h"
//#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

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

    ros::init(argumentCount, argumentVector, nodeName);
    if (!ros::master::check()) {
        throw TRANSLATOR_INITIALIZATION_FAILED;
    }
}

// Hangs if destroyed right after creation.
TopicTranslator::~TopicTranslator(void) {
    ros::shutdown();
    ros::waitForShutdown();
    wait();
}

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

void TopicTranslator::handleLeftCameraImage(const sensor_msgs::ImageConstPtr& message) {
}

void TopicTranslator::handleRightCameraImage(const sensor_msgs::ImageConstPtr& message) {
    cv_bridge::CvImagePtr image;
    try {
        image = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::RGB16);
    }
    catch (cv_bridge::Exception& cvException) {
        return;
    }

    currentRightCameraImage.convertFromImage(
        QImage(
            image->image.data,
            image->image.cols,
            image->image.rows,
            image->image.cols * image->image.elemSize(),
            QImage::Format_RGB16
        )
    );

    Q_EMIT rightCameraImageReceived(currentRightCameraImage);
}
