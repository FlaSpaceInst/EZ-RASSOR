#include <QObject> //
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "topic_translator.h"

bool TopicTranslator::connectToMaster(const std::string& masterURL) {
    std::map<std::string, std::string> masterURLRemap;
    masterURLRemap["__master"] = masterURL;
    ros::init(masterURLRemap, nodeName);

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
