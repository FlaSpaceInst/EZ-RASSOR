#ifndef TOPIC_TRANSLATOR_HEADER
#define TOPIC_TRANSLATOR_HEADER

#include <QThread>
#include <ros/ros.h>
#include "std_msgs/Float64.h"

class TopicTranslator : public QThread {
    Q_OBJECT
    public:
        TopicTranslator(
            const std::string& nodeName,
            const std::string& processorUsageTopic,
            const std::string& memoryUsageTopic,
            const std::string& batteryUsageTopic,
            int queueSize)
            : nodeName(nodeName),
              processorUsageTopic(processorUsageTopic),
              memoryUsageTopic(memoryUsageTopic),
              batteryUsageTopic(batteryUsageTopic),
              queueSize(queueSize) {};
        int exec(void);

    public slots:
        void connectToMaster(const std::string&);

    signals:
        void processorDataReceived(int);
        void memoryDataReceived(int);
        void batteryDataReceived(int);

    private:
        int queueSize;
        const std::string& nodeName;
        const std::string& processorUsageTopic;
        const std::string& memoryUsageTopic;
        const std::string& batteryUsageTopic;
        ros::Subscriber processorUsageSubscriber;
        ros::Subscriber memoryUsageSubscriber;
        ros::Subscriber batteryUsageSubscriber;
        
        void handleProcessorData(const std_msgs::Float64::ConstPtr&);
        void handleMemoryData(const std_msgs::Float64::ConstPtr&);
        void handleBatteryData(const std_msgs::Float64::ConstPtr&);
};
#endif
