// The TopicTranslator class acts as a ROS node that reads all ROS topics that
// the Dashboard needs to operate. It emits signals that notify the Dashboard
// of any changes.
// Written by Tiger Sachse.
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "topic_translator.h"

// Initialize this TopicTranslator with some topics.
TopicTranslator::TopicTranslator(
    const std::string& nodeName,
    const std::string& processorUsageTopic,
    const std::string& memoryUsageTopic,
    const std::string& batteryRemainingTopic,
    int queueSize) {

    this->nodeName = nodeName;
    this->processorUsageTopic = processorUsageTopic;
    this->memoryUsageTopic = memoryUsageTopic;
    this->batteryRemainingTopic = batteryRemainingTopic;
    this->queueSize = queueSize;
}

// Disconnect from the ROS Master Node and shut down the translator node.
void TopicTranslator::disconnectFromMaster(void) {
    if (ros::ok()) {
        ros::shutdown();
    }
    while (ros::ok()) {};
    Q_EMIT disconnectionSucceeded();
}

// Attempt to connect to a ROS Master Node.
void TopicTranslator::connectToMaster(const std::string& masterURI) {
    std::map<std::string, std::string> masterURIRemap;
    masterURIRemap["__master"] = masterURI;
    ros::init(masterURIRemap, nodeName);
    
    // This method of confirming the connection has issues. It doesn't really work
    // with ROS Master Nodes on different ports, and it hasn't been tested with
    // ROS graphs on remote systems. This will likely need to be revisited.
    if (ros::master::check()) {
        Q_EMIT connectionSucceeded();
        start();
    }
    else {
        Q_EMIT connectionFailed();
    }
}

// Run this thread (as a ROS node).
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
