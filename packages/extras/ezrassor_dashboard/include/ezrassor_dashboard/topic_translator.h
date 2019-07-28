// Define the topic translator, which translates data from ROS topics into
// data that Qt can work with.
// Written by Tiger Sachse.

#ifndef TOPIC_TRANSLATOR_HEADER
#define TOPIC_TRANSLATOR_HEADER

#include "mutex"
#include "QPixmap"
#include "QString"
#include "QStringList"
#include "QThread"
#include "rosgraph_msgs/Log.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "stereo_msgs/DisparityImage.h"
#include "string"
#include "unordered_set"

const int LOG_LEVEL_INFO = 2;
const std::string WHITELIST_PATH = "/config/whitelisted_nodes.txt";

class TopicTranslator : public QThread {
    Q_OBJECT

    public:
        TopicTranslator(
            int&,
            char**,
            int,
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&,
            const std::string&
        );
        ~TopicTranslator(void);
        bool initialized(void);

    public slots:
        void discoverNamespaces(void);
        void updateNamespace(const QString&);

    signals:
        void memoryDataReceived(int);
        void batteryDataReceived(int);
        void processorDataReceived(int);
        void logDataReceived(const QString&);
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
        void subscribersInitialized(void);
        void namespacesDiscovered(const QStringList&);

    private:
        const int queueSize;
        const std::string nodeName;
        const std::string imuTopic;
        const std::string logTopic;
        const std::string memoryUsageTopic;
        const std::string processorUsageTopic;
        const std::string batteryRemainingTopic;
        const std::string leftCameraImageTopic;
        const std::string rightCameraImageTopic;
        const std::string disparityMapImageTopic;
        bool currentNamespaceUpdated;
        std::string currentNamespace;
        std::mutex currentNamespaceMutex;
        QPixmap currentLeftCameraImage;
        QPixmap currentRightCameraImage;
        QPixmap currentDisparityMapImage;
        std::unordered_set<std::string> whitelistedNodes;

        void run(void);
        void routeIMUData(const sensor_msgs::Imu::ConstPtr&);
        void routeLogData(const rosgraph_msgs::Log::ConstPtr&);
        void routeMemoryData(const std_msgs::Float64::ConstPtr&);
        void routeBatteryData(const std_msgs::Float64::ConstPtr&);
        void routeProcessorData(const std_msgs::Float64::ConstPtr&);
        void routeLeftCameraImage(const sensor_msgs::ImageConstPtr&);
        void routeRightCameraImage(const sensor_msgs::ImageConstPtr&);
        void routeDisparityMapImage(const stereo_msgs::DisparityImage&);
        void processCameraImage(const sensor_msgs::ImageConstPtr&, QPixmap*);
};

#endif
