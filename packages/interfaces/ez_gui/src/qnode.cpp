#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <iostream>
#include "../include/ez_gui/qnode.hpp"

using namespace std;

namespace ez_gui {

    QNode::QNode(int argc, char** argv ) : init_argc(argc), init_argv(argv) {}

    QNode::~QNode()
    {
        if(ros::isStarted())
        {
          ros::shutdown(); // explicitly needed since we use ros::start();
          ros::waitForShutdown();
        }
        wait();
    }

    bool QNode::init()
    {
        ros::init(init_argc,init_argv,"ez_gui");

        if ( ! ros::master::check() )
            return false;

        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;

        // Add your ros communications here.
        imu_subscriber = n.subscribe("/imu", 1000, &QNode::imuCallback, this);
        image_subscriber = n.subscribe("/ez_rassor/front_camera/left/image_raw", 1, &QNode::imageFrontCallback, this);
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
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;

        // Add your ros communications here.
        imu_subscriber = n.subscribe("/imu", 1000, &QNode::imuCallback, this);
        image_subscriber = n.subscribe("/ez_rassor/front_camera/left/image_raw", 1, &QNode::imageFrontCallback, this);
        start();
        return true;
    }

    void QNode::run()
    {
        ros::NodeHandle n;
        imu_subscriber = n.subscribe("/imu", 1000, &QNode::imuCallback, this);
        image_subscriber = n.subscribe("/ez_rassor/front_camera/left/image_raw", 1, &QNode::imageFrontCallback, this);

        ros::spin();

        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
    }

    void QNode::imuCallback(const sensor_msgs::Imu& message_holder)
    {
        log(Info, message_holder);
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

        ROS_INFO_STREAM("Successful Image Conversion");

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
    }

    void QNode::log( const LogLevel &level, const sensor_msgs::Imu &msg)//change sensor_msgs::Image for testing
    {
        logging_model.insertRows(logging_model.rowCount(),1);
        std::stringstream logging_model_msg;

        switch ( level )
        {
            case(Debug) :
            {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                break;
            }
            case(Info) :
            {
                ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: \n" << msg;
                break;
            }
            case(Warn) :
            {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
            }
            case(Error) :
            {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                break;
            }
            case(Fatal) :
            {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                break;
            }
        }

        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        Q_EMIT loggingUpdated(); // used to readjust the scrollbar
    }
}  // namespace ez_gui
