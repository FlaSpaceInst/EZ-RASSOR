// Defines the TopicTranslator class which reads ROS topics and emits appropriate
// signals that notify the Dashboard of changes.
// Written by Tiger Sachse.
#ifndef TOPIC_TRANSLATOR_HEADER
#define TOPIC_TRANSLATOR_HEADER
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

class TopicTranslator : public QThread {
    Q_OBJECT
    public:
        TopicTranslator(
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&,
            int
        );
        int exec(void);

    public slots:
        void connectToMaster(const std::string&);

    signals:
        void processorDataReceived(int);
        void memoryDataReceived(int);
        void batteryDataReceived(int);

    private:
        int queueSize;
        std::string nodeName;
        std::string processorUsageTopic;
        std::string memoryUsageTopic;
        std::string batteryRemainingTopic;
        ros::Subscriber processorUsageSubscriber;
        ros::Subscriber memoryUsageSubscriber;
        ros::Subscriber batteryRemainingSubscriber;
        
        void handleProcessorData(const std_msgs::Float64::ConstPtr&);
        void handleMemoryData(const std_msgs::Float64::ConstPtr&);
        void handleBatteryData(const std_msgs::Float64::ConstPtr&);
};
#endif
