#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include "../include/ros_gui/qnode.hpp"

QNode::QNode(int argc, char** argv ) : init_argc(argc), init_argv(argv) {}

QNode::~QNode()
{
    if(ros::isStarted())
    {
      // explicitly needed since we use ros::start();	    
      ros::shutdown();
      ros::waitForShutdown();
    }

    for (int process : process_list) {
        if (process) kill(process, SIGKILL);
    }

    system("killall rosmaster & killall gzserver & killall gzclient");

    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"ez_gui");

    if ( ! ros::master::check() )
        return false;

    // Explicitly needed since our nodehandle is going out of scope.
    ros::start(); 
    ros::NodeHandle n;

     Q_EMIT startingRviz();

    // ROS Communications
    log_subscriber = n.subscribe("/ezrassor/status", 1, &QNode::logCallback, this);
    front_image_subscriber = n.subscribe("/ezrassor/front_camera/left/image_raw", 1, &QNode::imageFrontCallback, this);
    back_image_subscriber = n.subscribe("/ezrassor/front_camera/right/image_raw", 1, &QNode::imageBackCallback, this);
    disparity_subscriber = n.subscribe("/ezrassor/front_camera/disparity", 1, &QNode::disparityCallback, this);
    cpu_subscriber = n.subscribe("/ezrassor/cpuUsage", 100, &QNode::cpuCallback, this);
    vm_subscriber = n.subscribe("/ezrassor/virtualMemoryUsage", 100, &QNode::vmCallback, this);
    sm_subscriber = n.subscribe("/ezrassor/swapMemoryUsage", 100, &QNode::smCallback, this);
    disk_subscriber = n.subscribe("/ezrassor/diskUsage", 100, &QNode::diskCallback, this);
    battery_subscriber = n.subscribe("/ezrassor/batteryLeft", 100, &QNode::batteryCallback, this);
    imu_subscriber = n.subscribe("/imu", 1, &QNode::imuLabelsCallback, this);
    start();
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"ez_gui");

    if ( !ros::master::check() )
        return false;

    // explicitly needed since our nodehandle is going out of scope.
    ros::start();
    ros::NodeHandle n;

    Q_EMIT startingRviz();

    // Add your ros communications here.
    log_subscriber = n.subscribe("/ezrassor/status", 1, &QNode::logCallback, this);
    front_image_subscriber = n.subscribe("/ezrassor/front_camera/left/image_raw", 1, &QNode::imageFrontCallback, this);
    back_image_subscriber = n.subscribe("/ezrassor/front_camera/right/image_raw", 1, &QNode::imageBackCallback, this);
    disparity_subscriber = n.subscribe("/ezrassor/front_camera/disparity", 1, &QNode::disparityCallback, this);
    cpu_subscriber = n.subscribe("/ezrassor/cpuUsage", 100, &QNode::cpuCallback, this);
    vm_subscriber = n.subscribe("/ezrassor/virtualMemoryUsage", 100, &QNode::vmCallback, this);
    sm_subscriber = n.subscribe("/ezrassor/swapMemoryUsage", 100, &QNode::smCallback, this);
    disk_subscriber = n.subscribe("/ezrassor/diskUsage", 100, &QNode::diskCallback, this);
    battery_subscriber = n.subscribe("/ezrassor/batteryLeft", 100, &QNode::batteryCallback, this);
    imu_subscriber = n.subscribe("/imu", 1, &QNode::imuLabelsCallback, this);
    start();
    return true;
}

void QNode::run()
{
    ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;

    // used to signal the gui for a shutdown (useful to roslaunch)
    Q_EMIT rosShutdown(); 
}

void QNode::cpuCallback(const std_msgs::Float64& message_holder)
{
    cpu_Progress_Bar = (int) message_holder.data;
    Q_EMIT cpuUpdated();
}

void QNode::vmCallback(const std_msgs::Float64& message_holder)
{
    vm_Progress_Bar = (int) message_holder.data;
    Q_EMIT vmUpdated();
}

void QNode::smCallback(const std_msgs::Float64& message_holder)
{
    sm_Progress_Bar = (int) message_holder.data;
    Q_EMIT smUpdated();
}

void QNode::diskCallback(const std_msgs::Float64& message_holder)
{
    disk_Progress_Bar = (int) message_holder.data;
    Q_EMIT diskUpdated();
}

void QNode::batteryCallback(const std_msgs::Float64& message_holder)
{
    battery_Progress_Bar = (int) message_holder.data;
    Q_EMIT batteryUpdated();
}

void QNode::logCallback(const std_msgs::String& message_holder)
{
    log(Info, message_holder.data);
}

void QNode::imageFrontCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    QImage::Format format=QImage::Format_RGB32;
    int bpp=cv_ptr->image.channels();

    if(bpp==3) format=QImage::Format_RGB888;

    QImage img(cv_ptr->image.cols,cv_ptr->image.rows,format);
    uchar *sptr,*dptr;
    int linesize=cv_ptr->image.cols*bpp;

    for(int y = 0; y < cv_ptr->image.rows; y++)
    {
        sptr=cv_ptr->image.ptr(y);
        dptr=img.scanLine(y);
        memcpy(dptr,sptr,linesize);
    }

    if(bpp==3)
        front_Camera_Pixmap = QPixmap::fromImage(img.rgbSwapped());
    else
        front_Camera_Pixmap = QPixmap::fromImage(img);

    Q_EMIT frontCamUpdated();
}

void QNode::imageBackCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    QImage::Format format=QImage::Format_RGB32;
    int bpp=cv_ptr->image.channels();

    if(bpp==3) format=QImage::Format_RGB888;

    QImage img(cv_ptr->image.cols,cv_ptr->image.rows,format);
    uchar *sptr,*dptr;
    int linesize=cv_ptr->image.cols*bpp;

    for(int y = 0; y < cv_ptr->image.rows; y++)
    {
        sptr=cv_ptr->image.ptr(y);
        dptr=img.scanLine(y);
        memcpy(dptr,sptr,linesize);
    }

    if(bpp==3)
        back_Camera_Pixmap = QPixmap::fromImage(img.rgbSwapped());
    else
        back_Camera_Pixmap = QPixmap::fromImage(img);

    Q_EMIT backCamUpdated();
}

void QNode::disparityCallback(const stereo_msgs::DisparityImage& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg.image);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat float_img = cv_ptr->image;
    cv::Mat mono8_img;

    if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols)
        mono8_img = cv::Mat(float_img.size(), CV_8UC1);

    cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
    QImage::Format format=QImage::Format_Grayscale8;

    int bpp=mono8_img.channels();

    if(bpp==3) format=QImage::Format_RGB888;

    QImage img(mono8_img.cols,mono8_img.rows,format);
    uchar *sptr,*dptr;
    int linesize=mono8_img.cols*bpp;

    for(int y = 0; y < mono8_img.rows; y++)
    {
        sptr=mono8_img.ptr(y);
        dptr=img.scanLine(y);
        memcpy(dptr,sptr,linesize);
    }

    if(bpp==3)
        disparity_Pixmap = QPixmap::fromImage(img.rgbSwapped());
    else
        disparity_Pixmap = QPixmap::fromImage(img);

    Q_EMIT disparityUpdated();
}

void QNode::imuLabelsCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_labels[0] = msg->orientation.x;
    imu_labels[1] = msg->orientation.y;
    imu_labels[2] = msg->orientation.z;

    imu_labels[3] = msg->angular_velocity.x;
    imu_labels[4] = msg->angular_velocity.y;
    imu_labels[5] = msg->angular_velocity.z;

    imu_labels[6] = msg->linear_acceleration.x;
    imu_labels[7] = msg->linear_acceleration.y;
    imu_labels[8] = msg->linear_acceleration.z;

    Q_EMIT imuLabelsUpdated();
}

void QNode::log( const LogLevel &level, std::string msg)
{
    std::stringstream logging_model_msg;

    switch ( level )
    {
        case(Debug) :
        {
            //ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [";
            break;
        }
        case(Info) :
        {
            //ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [";
            break;
        }
        case(Warn) :
        {
            //ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [";
            break;
        }
        case(Error) :
        {
            //ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [";
            break;
        }
        case(Fatal) :
        {
            //ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [";
            break;
        }
    }

    logging_model_msg << std::setprecision(3) << ros::Time::now().toSec() << "]: " << msg;


    logging_model = QString::fromStdString(logging_model_msg.str());
    Q_EMIT loggingUpdated();
}

void QNode::add_executable_package(QString executable, QString package)
{
    executable_package_map[executable] = package;
}

QString QNode::get_executable_package(QString executable)
{
    return executable_package_map[executable];
}

void QNode::addProcess(pid_t process) {
    process_list.push_back(process);
}
