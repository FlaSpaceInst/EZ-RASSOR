// The TopicTranslator class acts as a ROS node that reads all ROS topics that
// the Dashboard needs to operate. It emits signals that notify the Dashboard
// of any changes.
// Written by Tiger Sachse.
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "topic_translator.h"

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

// Attempt to connect to a ROS Master Node.
void TopicTranslator::connectToMaster(const std::string& masterURI) {
    std::map<std::string, std::string> masterURIRemap;
    masterURIRemap["__master"] = masterURI;
    ros::init(masterURIRemap, nodeName);

    if (!ros::master::check()) {
        return;// false;
    }

    // explicitly needed bc nodehandle out of skope
    ros::start();
    ros::NodeHandle nodeHandle;

    processorUsageSubscriber = nodeHandle.subscribe(
        processorUsageTopic,
        queueSize,
        &TopicTranslator::handleProcessorData,
        this
    );
    memoryUsageSubscriber = nodeHandle.subscribe(
        memoryUsageTopic,
        queueSize,
        &TopicTranslator::handleMemoryData,
        this
    );
    batteryRemainingSubscriber = nodeHandle.subscribe(
        batteryRemainingTopic,
        queueSize,
        &TopicTranslator::handleBatteryData,
        this
    );

    //return true;
    start();
}

// Execute this thread (as a ROS node).
int TopicTranslator::exec(void) {
    ros::spin();
    //std::cout << "ROS has shutdown: closing Dashboard..." << std::endl;

    //Q_EMIT 
    return 0;
}

// Handle incoming processor data from ROS.
void TopicTranslator::handleProcessorData(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT processorDataReceived((int) message->data);
}

// Handle incoming memory data from ROS.
void TopicTranslator::handleMemoryData(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT memoryDataReceived((int) message->data);
}

// Handle incoming battery data from ROS.
void TopicTranslator::handleBatteryData(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT batteryDataReceived((int) message->data);
}
