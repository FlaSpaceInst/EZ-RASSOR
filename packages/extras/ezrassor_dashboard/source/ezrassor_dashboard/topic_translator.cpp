#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "topic_translator.h"

void TopicTranslator::connectToMaster(const std::string& masterURI) {
    std::cout << masterURI << std::endl;
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
    batteryUsageSubscriber = nodeHandle.subscribe(
        batteryUsageTopic,
        queueSize,
        &TopicTranslator::handleBatteryData,
        this
    );

    //return true;
    start();
}

int TopicTranslator::exec(void) {
    ros::spin();
    //std::cout << "ROS has shutdown: closing Dashboard..." << std::endl;

    //Q_EMIT 
    return 0;
}

void TopicTranslator::handleProcessorData(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT processorDataReceived((int) message->data);
}

void TopicTranslator::handleMemoryData(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT memoryDataReceived((int) message->data);
}

void TopicTranslator::handleBatteryData(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT batteryDataReceived((int) message->data);
}
