// Written by Tiger Sachse.
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "topic_translator.h"

TopicTranslator::TopicTranslator(
    int& argumentCount,
    char** argumentVector,
    const std::string& nodeName,
    int queueSize,
    const std::string& memoryUsageTopic,
    const std::string& processorUsageTopic,
    const std::string& batteryRemainingTopic)
    : memoryUsageTopic(memoryUsageTopic),
      processorUsageTopic(processorUsageTopic),
      batteryRemainingTopic(batteryRemainingTopic),
      queueSize(queueSize) {

    ros::init(argumentCount, argumentVector, nodeName);
    if (ros::master::check()) {
        start();
    }
    else {
        Q_EMIT connectionFailed();
    }
}

TopicTranslator::~TopicTranslator(void) {
    ros::shutdown();
    ros::waitForShutdown();
    wait(5000);
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
