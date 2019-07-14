// Define the topic translator, which translates data from ROS topics into
// data that Qt can work with.
// Written by Tiger Sachse.

#ifndef TOPIC_TRANSLATOR_HEADER
#define TOPIC_TRANSLATOR_HEADER

#include <QThread>
#include <QString>
#include <QPixmap>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"

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
            const std::string&,
            const std::string&,
            const std::string&
        );
        ~TopicTranslator(void);

    signals:
        void memoryDataReceived(int);
        void batteryDataReceived(int);
        void processorDataReceived(int);
        void xOrientationReceived(const QString&);
        void yOrientationReceived(const QString&);
        void zOrientationReceived(const QString&);
        void xAngularVelocityReceived(const QString&);
        void yAngularVelocityReceived(const QString&);
        void zAngularVelocityReceived(const QString&);
        void xLinearAccelerationReceived(const QString&);
        void yLinearAccelerationReceived(const QString&);
        void zLinearAccelerationReceived(const QString&);
        void leftCameraImageReceived(const QPixmap&);
        void rightCameraImageReceived(const QPixmap&);
        void disparityMapImageReceived(const QPixmap&);

    private:
        const int queueSize;
        const std::string nodeName;
        const std::string imuTopic;
        const std::string memoryUsageTopic;
        const std::string processorUsageTopic;
        const std::string batteryRemainingTopic;
        const std::string leftCameraImageTopic;
        const std::string rightCameraImageTopic;
        const std::string disparityMapImageTopic;
        QString currentXOrientation;
        QString currentYOrientation;
        QString currentZOrientation;
        QString currentXAngularVelocity;
        QString currentYAngularVelocity;
        QString currentZAngularVelocity;
        QString currentXLinearAcceleration;
        QString currentYLinearAcceleration;
        QString currentZLinearAcceleration;
        int currentMemoryPercentage;
        int currentBatteryPercentage;
        int currentProcessorPercentage;
        QPixmap currentLeftCameraImage;
        QPixmap currentRightCameraImage;
        QPixmap currentDisparityMapImage;
        
        void run(void);
        void saveIMUData(const sensor_msgs::Imu::ConstPtr&);
        void saveMemoryData(const std_msgs::Float64::ConstPtr&);
        void saveBatteryData(const std_msgs::Float64::ConstPtr&);
        void saveProcessorData(const std_msgs::Float64::ConstPtr&);
        void saveLeftCameraImage(const sensor_msgs::ImageConstPtr&);
        void saveRightCameraImage(const sensor_msgs::ImageConstPtr&);
        void saveDisparityMapImage(const stereo_msgs::DisparityImage&);
        void processCameraImage(const sensor_msgs::ImageConstPtr&, QPixmap*);
};

#endif
