#ifndef ez_gui_QNODE_HPP_
#define ez_gui_QNODE_HPP_

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <rqt_image_view/image_view.h>
#endif

#include <QtGui/QMainWindow>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <QThread>
#include <QStringListModel>
#include <QProgressBar>

namespace ez_gui {

    class QNode : public QThread
    {
        Q_OBJECT
        public:
            QNode(int argc, char** argv );
            virtual ~QNode();
            bool init();
            bool init(const std::string &master_url, const std::string &host_url);
            void run();
            void imuCallback(const sensor_msgs::Imu &message_holder);
            void imageFrontCallback(const sensor_msgs::ImageConstPtr &message_holder);
            void imageBackCallback(const sensor_msgs::ImageConstPtr& msg);
            void cpuCallback(const std_msgs::Float64 &message_holder);
            void vmCallback(const std_msgs::Float64 &message_holder);
            void smCallback(const std_msgs::Float64 &message_holder);
            void diskCallback(const std_msgs::Float64 &message_holder);
            void batteryCallback(const std_msgs::Float64 &message_holder);

            /*********************
            ** Logging
            **********************/
            enum LogLevel
            {
                 Debug,
                 Info,
                 Warn,
                 Error,
                 Fatal
            };

            QStringListModel* loggingModel() { return &logging_model; }
            QPixmap *frontCameraPixmap() { return &front_Camera_Pixmap; }
            QPixmap *backCameraPixmap() { return &back_Camera_Pixmap; }
            int *cpuBarUpdate() { return &cpu_Progress_Bar; }
            int *vmBarUpdate() { return &vm_Progress_Bar; }
            int *smBarUpdate() { return &sm_Progress_Bar; }
            int *diskBarUpdate() { return &disk_Progress_Bar; }
            int *batteryBarUpdate() { return &battery_Progress_Bar; }
            void log( const LogLevel &level, const sensor_msgs::Imu &msg);

            int cpu_Progress_Bar;
            int vm_Progress_Bar;
            int sm_Progress_Bar;
            int disk_Progress_Bar;
            int battery_Progress_Bar;
            QPixmap front_Camera_Pixmap;
            QPixmap back_Camera_Pixmap;

        Q_SIGNALS:
            void loggingUpdated();

            void cpuUpdated();
            void vmUpdated();
            void smUpdated();
            void diskUpdated();
            void batteryUpdated();

            void frontCamUpdated();
            void backCamUpdated();

            void rosShutdown();

        private:
            int init_argc;
            char** init_argv;
            ros::Subscriber imu_subscriber;
            ros::Subscriber front_image_subscriber;
            ros::Subscriber back_image_subscriber;
            ros::Subscriber cpu_subscriber;
            ros::Subscriber vm_subscriber;
            ros::Subscriber sm_subscriber;
            ros::Subscriber disk_subscriber;
            ros::Subscriber battery_subscriber;
            QStringListModel logging_model;
    };

}  // namespace ez_gui

#endif /* ez_gui_QNODE_HPP_ */
