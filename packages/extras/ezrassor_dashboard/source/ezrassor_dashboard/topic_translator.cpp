// A ROS node that translates data from ROS topics into data that Qt can work with.
// Inspired by Chris Taliaferro, Samuel Lewis, and Lucas Gonzalez.
// Written by Tiger Sachse and Sean Rapp.

#include "cv_bridge/cv_bridge.h"
#include "fstream"
#include "mutex"
#include "QImage"
#include "QPixmap"
#include "QString"
#include "QStringList"
#include "QThread"
#include "ros/console.h"
#include "ros/package.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64.h"
#include "stereo_msgs/DisparityImage.h"
#include "string"
#include "topic_translator.h"
#include "unordered_set"
#include "vector"

// Initialize the topic translator with a bunch of constants.
TopicTranslator::TopicTranslator(
    int& argumentCount,
    char** argumentVector,
    int queueSize,
    const std::string& packageName,
    const std::string& nodeName,
    const std::string& imuTopic,
    const std::string& logTopic,
    const std::string& memoryUsageTopic,
    const std::string& processorUsageTopic,
    const std::string& batteryRemainingTopic,
    const std::string& leftCameraImageTopic,
    const std::string& rightCameraImageTopic,
    const std::string& disparityMapImageTopic):
        queueSize(queueSize),
        nodeName(nodeName),
        imuTopic(imuTopic),
        logTopic(logTopic),
        memoryUsageTopic(memoryUsageTopic),
        processorUsageTopic(processorUsageTopic),
        batteryRemainingTopic(batteryRemainingTopic),
        leftCameraImageTopic(leftCameraImageTopic),
        rightCameraImageTopic(rightCameraImageTopic),
        disparityMapImageTopic(disparityMapImageTopic) {

    ros::init(argumentCount, argumentVector, nodeName);

    currentNamespace = "/";
    currentNamespaceUpdated = true;

    currentLeftCameraImage = QPixmap();//
    currentRightCameraImage = QPixmap();//
    currentDisparityMapImage = QPixmap();//

    // Read the whitelisted nodes into an unordered set.
    whitelistedNodes = std::unordered_set<std::string>();
    std::ifstream whitelistedNodesFile(
        ros::package::getPath(packageName) + WHITELIST_PATH
    );
    ROS_INFO("Loading node whitelist...");
    if (whitelistedNodesFile.is_open()) {
        std::string line;
        while (getline(whitelistedNodesFile, line)) {
            whitelistedNodes.insert(line);
        }
        whitelistedNodesFile.close();
    }
    else {
        ROS_WARN("Node whitelist not found. Expect log to be blank.");
    }

    ROS_INFO("Dashboard initialized.");
}

// Destroy this translator. Hangs if destroyed right after creation.
TopicTranslator::~TopicTranslator(void) {
    ros::shutdown();
    ros::waitForShutdown();
    wait();
}

// Confirm that the topic translator can talk to ROS Master.
bool TopicTranslator::initialized(void) {
    if (!ros::master::check()) {
        ROS_ERROR("ROS Master cannot be contacted.");

        return false;
    }
    else {
        return true;
    }
}

// Get a list of available namespaces.
void TopicTranslator::discoverNamespaces(void) {

    // Get all node names.
    std::vector<std::string> nodeNames;
    ros::master::getNodes(nodeNames);

    // Build a QStringList of namespaces, then remove duplicates.
    QStringList nodeNamespaces;
    for (std::string nodeName : nodeNames) {
        std::string nodeNamespace = ros::names::parentNamespace(nodeName);
        if (nodeNamespace.empty() or nodeNamespace.back() != '/') {
            nodeNamespace += "/";
        }
        nodeNamespaces.append(QString::fromStdString(nodeNamespace));
    }
    nodeNamespaces.removeDuplicates();

    Q_EMIT namespacesDiscovered(nodeNamespaces);
}

// Update the current namespace.
void TopicTranslator::updateNamespace(const QString& newNamespace) {

    // Ensure that newNamespace isn't empty before proceeding. Some signals
    // trigger this function with empty strings, and those triggers should
    // be ignored.
    if (!newNamespace.isEmpty()) {
        std::lock_guard<std::mutex> lock(currentNamespaceMutex);
        currentNamespace = newNamespace.toUtf8().constData();
        currentNamespaceUpdated = true;
        ROS_INFO_STREAM(
            "Namespace updated to '" << currentNamespace << "'."
        );
    }
}

// Run the translator ROS node.
void TopicTranslator::run(void) {

    // Create the master NodeHandle and the logSubscriber. The logTopic is
    // never namespaced and so it does not need to be inside the main loop.
    // This subscriber will remain in-scope until this function returns.
    ros::NodeHandle nodeHandle;
    ros::Subscriber logSubscriber = nodeHandle.subscribe(
        "/" + logTopic,
        queueSize,
        &TopicTranslator::routeLogData,
        this
    );

    int iterationsPerSecond = 10;
    ros::WallRate rate(iterationsPerSecond);
    while(ros::ok()) {

        // Make a copy of the currentNamespace string locally. This string and
        // its boolean variable are protected with a lock because they reside
        // in the main thread. A bad read could occur if this function (in the
        // ROS thread) attempts to read currentNamespace as the main thread
        // updates it. I make a copy of the string instead of locking the entire
        // subscriber contruction section to reduce the duration of the critical
        // section of this function.
        std::string currentNamespace;
        {
            std::lock_guard<std::mutex> lock(currentNamespaceMutex);
            currentNamespace = this->currentNamespace;
            currentNamespaceUpdated = false;
        }

        ros::Subscriber imuSubscriber = nodeHandle.subscribe(
            currentNamespace + imuTopic,
            queueSize,
            &TopicTranslator::routeIMUData,
            this
        );
        ros::Subscriber processorUsageSubscriber = nodeHandle.subscribe(
            currentNamespace + processorUsageTopic,
            queueSize,
            &TopicTranslator::routeProcessorData,
            this
        );
        ros::Subscriber memoryUsageSubscriber = nodeHandle.subscribe(
            currentNamespace + memoryUsageTopic,
            queueSize,
            &TopicTranslator::routeMemoryData,
            this
        );
        ros::Subscriber batteryRemainingSubscriber = nodeHandle.subscribe(
            currentNamespace + batteryRemainingTopic,
            queueSize,
            &TopicTranslator::routeBatteryData,
            this
        );
        ros::Subscriber leftCameraImageSubscriber = nodeHandle.subscribe(
            currentNamespace + leftCameraImageTopic,
            queueSize,
            &TopicTranslator::routeLeftCameraImage,
            this
        );
        ros::Subscriber rightCameraImageSubscriber = nodeHandle.subscribe(
            currentNamespace + rightCameraImageTopic,
            queueSize,
            &TopicTranslator::routeRightCameraImage,
            this
        );
        ros::Subscriber disparityMapImageSubscriber = nodeHandle.subscribe(
            currentNamespace + disparityMapImageTopic,
            queueSize,
            &TopicTranslator::routeDisparityMapImage,
            this
        );

        Q_EMIT subscribersInitialized();

        // Spin this node, but occasionally check to see if the currentNamespace
        // has been updated. If so, loop back to the beginning and rebuild the
        // subscribers.
        while (ros::ok()) {
            for (int iteration = 1; iteration < iterationsPerSecond; iteration++) {
                ros::spinOnce();
                rate.sleep();
            }
            std::lock_guard<std::mutex> lock(currentNamespaceMutex);
            if (currentNamespaceUpdated) {
                break;
            }
        }
    }
}

// Route incoming IMU data from ROS.
void TopicTranslator::routeIMUData(const sensor_msgs::Imu::ConstPtr& message) {
    Q_EMIT xOrientationReceived(
        QString().setNum(message->orientation.x, 'f', 3)
    );
    Q_EMIT yOrientationReceived(
        QString().setNum(message->orientation.y, 'f', 3)
    );
    Q_EMIT zOrientationReceived(
        QString().setNum(message->orientation.z, 'f', 3)
    );
    Q_EMIT xAngularVelocityReceived(
        QString().setNum(message->angular_velocity.x, 'f', 3)
    );
    Q_EMIT yAngularVelocityReceived(
        QString().setNum(message->angular_velocity.y, 'f', 3)
    );
    Q_EMIT zAngularVelocityReceived(
        QString().setNum(message->angular_velocity.z, 'f', 3)
    );
    Q_EMIT xLinearAccelerationReceived(
        QString().setNum(message->linear_acceleration.x, 'f', 3)
    );
    Q_EMIT yLinearAccelerationReceived(
        QString().setNum(message->linear_acceleration.y, 'f', 3)
    );
    Q_EMIT zLinearAccelerationReceived(
        QString().setNum(message->linear_acceleration.z, 'f', 3)
    );
}

// Route incoming log data from ROS.
void TopicTranslator::routeLogData(const rosgraph_msgs::Log::ConstPtr& message) {

    // Get the node name and node namespace, then confirm we care about its
    // message. parentNamespace() returns the node's namespace without a forward
    // slash suffix, unless of course the namespace is only "/". To properly
    // match with the format of currentNamespace, a forward slash suffix is
    // added to any node namespace that is not the global namespace ("/").
    std::string nodeName = message->name.substr(message->name.rfind('/') + 1);
    std::string nodeNamespace = ros::names::parentNamespace(message->name);
    if (nodeNamespace != "/") {
        nodeNamespace += "/";
    }

    // Filter out messages that are not priority level INFO, that are from nodes
    // not on the whitelist, and/or that are from nodes in the wrong namespace.
    // Apply these filters to all nodes except this node... this node is allowed
    // to publish whatever it wants. :)
    if (nodeName != this->nodeName) {
        if ((int) message->level != LOG_LEVEL_INFO) {
            return;
        }
        if (nodeNamespace.find(currentNamespace) == std::string::npos) {
            return;
        }
        if ((whitelistedNodes.find(nodeName)) == whitelistedNodes.end()) {
            return;
        }
    }

    // Create a QString with the full node name prefixed to the message.
    QString logLine = QString::fromStdString(
        "[" + message->name + "] " + message->msg
    );

    Q_EMIT logDataReceived(logLine);
}

// Route incoming memory data from ROS.
void TopicTranslator::routeMemoryData(
    const std_msgs::Float64::ConstPtr& message) {

    Q_EMIT memoryDataReceived((int) message->data);
}

// Route incoming battery data from ROS.
void TopicTranslator::routeBatteryData(
    const std_msgs::Float64::ConstPtr& message) {

    Q_EMIT batteryDataReceived((int) message->data);
}

// Route incoming processor data from ROS.
void TopicTranslator::routeProcessorData(
    const std_msgs::Float64::ConstPtr& message) {

    Q_EMIT processorDataReceived((int) message->data);
}

// Route incoming left camera images from ROS.
void TopicTranslator::routeLeftCameraImage(
    const sensor_msgs::ImageConstPtr& message) {

    processCameraImage(message, &currentLeftCameraImage);
    Q_EMIT leftCameraImageReceived(currentLeftCameraImage);
}

// Route incoming right camera images from ROS.
void TopicTranslator::routeRightCameraImage(
    const sensor_msgs::ImageConstPtr& message) {

    processCameraImage(message, &currentRightCameraImage);
    Q_EMIT rightCameraImageReceived(currentRightCameraImage);
}

// Route incoming disparity images from ROS. This function is 90% duplicated from
// processCameraImage() because C++ sucks. Someone who is better with C++ than I
// am should combine the two functions (I was salty when I wrote this).
void TopicTranslator::routeDisparityMapImage(
    const stereo_msgs::DisparityImage& message) {

    cv_bridge::CvImagePtr cvImage;
    try {
        cvImage = cv_bridge::toCvCopy(message.image);
    }
    catch (cv_bridge::Exception& cvException) {
        ROS_ERROR(
            "Exception handling disparity feed in cv_bridge: %s",
            cvException.what()
        );

        return;
    }

    cv::Mat monochromeImage = cv::Mat(cvImage->image.size(), CV_8UC1);
    cv::convertScaleAbs(cvImage->image, monochromeImage, 100, 0.0);

    QImage::Format imageFormat;
    if (monochromeImage.channels() == 3) {
        imageFormat = QImage::Format_RGB888;
    }
    else {
        imageFormat = QImage::Format_Grayscale8;
    }

    QImage qtImage(monochromeImage.cols, monochromeImage.rows, imageFormat);
    int lineSize = monochromeImage.cols * monochromeImage.channels();
    for (int row = 0; row < monochromeImage.rows; row++) {
        memcpy(qtImage.scanLine(row), monochromeImage.ptr(row), lineSize);
    }

    if (monochromeImage.channels() == 3) {
        currentDisparityMapImage = QPixmap::fromImage(qtImage.rgbSwapped());
    }
    else {
        currentDisparityMapImage = QPixmap::fromImage(qtImage);
    }

    Q_EMIT disparityMapImageReceived(currentDisparityMapImage);
}

// Transform images from a ROS format to a Qt format using cv_bridge.
void TopicTranslator::processCameraImage(
    const sensor_msgs::ImageConstPtr& message,
    QPixmap* currentCameraImage) {

    cv_bridge::CvImagePtr cvImage;
    try {
        cvImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& cvException) {
        ROS_ERROR(
            "Exception handling camera feed in cv_bridge: %s",
            cvException.what()
        );

        return;
    }

    QImage::Format imageFormat;
    if (cvImage->image.channels() == 3) {
        imageFormat = QImage::Format_RGB888;
    }
    else {
        imageFormat = QImage::Format_RGB32;
    }

    QImage qtImage(cvImage->image.cols, cvImage->image.rows, imageFormat);
    int lineSize = cvImage->image.cols * cvImage->image.channels();
    for (int row = 0; row < cvImage->image.rows; row++) {
        memcpy(qtImage.scanLine(row), cvImage->image.ptr(row), lineSize);
    }

    if (cvImage->image.channels() == 3) {
        *currentCameraImage = QPixmap::fromImage(qtImage.rgbSwapped());
    }
    else {
        *currentCameraImage = QPixmap::fromImage(qtImage);
    }
}
