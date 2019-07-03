#ifndef TOPIC_TRANSLATOR_HEADER
#define TOPIC_TRANSLATOR_HEADER

#include <QObject>
#include <ros/ros.h>
#include "std_msgs/Float64.h"

class TopicTranslator : public QObject {
    Q_OBJECT
    public:
        TopicTranslator(const std::string& nodeName) : nodeName(nodeName) {};
        bool connectToMaster(const std::string&);

    signals:
        void processorDatumHandled(double);

    private:
        const std::string& nodeName;
        ros::Subscriber processorUsageSubscriber;
        
        void handleProcessorDatum(const std_msgs::Float64::ConstPtr&);
};
#endif
