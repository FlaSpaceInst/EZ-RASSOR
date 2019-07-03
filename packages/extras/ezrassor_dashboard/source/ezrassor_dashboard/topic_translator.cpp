#include <QObject>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "topic_translator.h"

bool TopicTranslator::connectToMasterconst std::string& masterURI) {
    std::map<std::string, std::string> masterURIRemap;
    masterURIRemap["__master"] = masterURI;
    ros::init(masterURIRemap, nodeName);

    if (!ros::master::check()) {
        return false;
    }

    ros::NodeHandle nodeHandle;

    processorUsageSubscriber = nodeHandle.subscribe(
        "cpu_usage",
        1000,
        &TopicTranslator::handleProcessorDatum,
        this
    );

    return true;
}

void TopicTranslator::handleProcessorDatum(const std_msgs::Float64::ConstPtr& message) {
    Q_EMIT processorDatumHandled((double) message->data);
}
