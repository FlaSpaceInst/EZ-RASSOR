// Written by Tiger Sachse.
#ifndef TOPIC_TRANSLATOR_HEADER
#define TOPIC_TRANSLATOR_HEADER
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Image.h"

const int TRANSLATOR_INITIALIZATION_FAILED = 1;

class TopicTranslator : public QThread {
    Q_OBJECT
    public:
        TopicTranslator(
            int&,
            char**,
            const std::string&,
            int,
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&
        );
        ~TopicTranslator(void);

    signals:
        void memoryDataReceived(int);
        void batteryDataReceived(int);
        void processorDataReceived(int);
        void leftCameraImageReceived(const QPixmap&);
        void rightCameraImageReceived(const QPixmap&);

    private:
        const int queueSize;
        const std::string nodeName;
        const std::string memoryUsageTopic;
        const std::string processorUsageTopic;
        const std::string batteryRemainingTopic;
        const std::string leftCameraImageTopic;
        const std::string rightCameraImageTopic;
        QPixmap currentLeftCameraImage;
        QPixmap currentRightCameraImage;
        
        void run(void);
        void handleMemoryData(const std_msgs::Float64::ConstPtr&);
        void handleBatteryData(const std_msgs::Float64::ConstPtr&);
        void handleProcessorData(const std_msgs::Float64::ConstPtr&);
        void handleLeftCameraImage(const sensor_msgs::ImageConstPtr&);
        void handleRightCameraImage(const sensor_msgs::ImageConstPtr&);
};
#endif
