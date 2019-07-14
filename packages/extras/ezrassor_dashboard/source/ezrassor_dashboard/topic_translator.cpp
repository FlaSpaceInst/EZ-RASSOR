// A ROS node that translates data from ROS topics into data that Qt can work with.
// Inspired by Chris Taliaferro, Samuel Lewis, and Lucas Gonzalez.
// Written by Tiger Sachse and Sean Rapp.

#include <QImage>
#include <QThread>
#include <QString>
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
    const std::string& imuTopic,
    const std::string& memoryUsageTopic,
    const std::string& processorUsageTopic,
    const std::string& batteryRemainingTopic,
    const std::string& leftCameraImageTopic,
    const std::string& rightCameraImageTopic)
    : queueSize(queueSize),
      imuTopic(imuTopic),
      memoryUsageTopic(memoryUsageTopic),
      processorUsageTopic(processorUsageTopic),
      batteryRemainingTopic(batteryRemainingTopic),
      leftCameraImageTopic(leftCameraImageTopic),
      rightCameraImageTopic(rightCameraImageTopic) {

    currentMemoryPercentage = 0;
    currentBatteryPercentage = 0;
    currentProcessorPercentage = 0;
    currentLeftCameraImage = QPixmap();
    currentRightCameraImage = QPixmap();
    currentXOrientation = QString();
    currentYOrientation = QString();
    currentZOrientation = QString();
    currentXAngularVelocity = QString();
    currentYAngularVelocity = QString();
    currentZAngularVelocity = QString();
    currentXLinearAcceleration = QString();
    currentYLinearAcceleration = QString();
    currentZLinearAcceleration = QString();

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
    ros::Subscriber imuSubscriber = nodeHandle.subscribe(
        imuTopic,
        queueSize,
        &TopicTranslator::saveIMUData,
        this
    );
    ros::Subscriber processorUsageSubscriber = nodeHandle.subscribe(
        processorUsageTopic,
        queueSize,
        &TopicTranslator::saveProcessorData,
        this
    );
    ros::Subscriber memoryUsageSubscriber = nodeHandle.subscribe(
        memoryUsageTopic,
        queueSize,
        &TopicTranslator::saveMemoryData,
        this
    );
    ros::Subscriber batteryRemainingSubscriber = nodeHandle.subscribe(
        batteryRemainingTopic,
        queueSize,
        &TopicTranslator::saveBatteryData,
        this
    );
    ros::Subscriber leftCameraImageSubscriber = nodeHandle.subscribe(
        leftCameraImageTopic,
        queueSize,
        &TopicTranslator::saveLeftCameraImage,
        this
    );
    ros::Subscriber rightCameraImageSubscriber = nodeHandle.subscribe(
        rightCameraImageTopic,
        queueSize,
        &TopicTranslator::saveRightCameraImage,
        this
    );

    ros::spin();
}

// Save incoming IMU data from ROS.
void TopicTranslator::saveIMUData(const sensor_msgs::Imu::ConstPtr& message) {
    currentXOrientation.setNum(message->orientation.x);
    currentYOrientation.setNum(message->orientation.y);
    currentZOrientation.setNum(message->orientation.z);
    currentXAngularVelocity.setNum(message->angular_velocity.x);
    currentYAngularVelocity.setNum(message->angular_velocity.y);
    currentZAngularVelocity.setNum(message->angular_velocity.z);
    currentXLinearAcceleration.setNum(message->linear_acceleration.x);
    currentYLinearAcceleration.setNum(message->linear_acceleration.y);
    currentZLinearAcceleration.setNum(message->linear_acceleration.z);

    Q_EMIT xOrientationReceived(currentXOrientation);
    Q_EMIT yOrientationReceived(currentYOrientation);
    Q_EMIT zOrientationReceived(currentZOrientation);
    Q_EMIT xAngularVelocityReceived(currentXAngularVelocity);
    Q_EMIT yAngularVelocityReceived(currentYAngularVelocity);
    Q_EMIT zAngularVelocityReceived(currentZAngularVelocity);
    Q_EMIT xLinearAccelerationReceived(currentXLinearAcceleration);
    Q_EMIT yLinearAccelerationReceived(currentYLinearAcceleration);
    Q_EMIT zLinearAccelerationReceived(currentZLinearAcceleration);
}

// Save incoming memory data from ROS.
void TopicTranslator::saveMemoryData(const std_msgs::Float64::ConstPtr& message) {
    currentMemoryPercentage = (int) message->data;
    Q_EMIT memoryDataReceived(currentMemoryPercentage);
}

// Save incoming battery data from ROS.
void TopicTranslator::saveBatteryData(const std_msgs::Float64::ConstPtr& message) {
    currentBatteryPercentage = (int) message->data;
    Q_EMIT batteryDataReceived(currentBatteryPercentage);
}

// Save incoming processor data from ROS.
void TopicTranslator::saveProcessorData(const std_msgs::Float64::ConstPtr& message) {
    currentProcessorPercentage = (int) message->data;
    Q_EMIT processorDataReceived(currentProcessorPercentage);
}

// Save incoming left camera images from ROS.
void TopicTranslator::saveLeftCameraImage(const sensor_msgs::ImageConstPtr& message) {
    processCameraImage(message, &currentLeftCameraImage);
    Q_EMIT leftCameraImageReceived(currentLeftCameraImage);
}

// Save incoming right camera images from ROS.
void TopicTranslator::saveRightCameraImage(const sensor_msgs::ImageConstPtr& message) {
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