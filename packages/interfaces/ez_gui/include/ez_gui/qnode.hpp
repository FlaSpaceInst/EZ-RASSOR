#ifndef ez_gui_QNODE_HPP_
#define ez_gui_QNODE_HPP_

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <image_transport/image_transport.h>
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
            void imuCallback(const sensor_msgs::Imu& message_holder);
            void imageFrontCallback(const sensor_msgs::ImageConstPtr& message_holder);

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
            QPixmap frontCameraPixmap() { return front_Camera_Pixmap; }
            void log( const LogLevel &level, const sensor_msgs::Imu &msg);

        Q_SIGNALS:
            void loggingUpdated();
            void rosShutdown();

        private:
            int init_argc;
            char** init_argv;
            ros::Subscriber imu_subscriber;
            ros::Subscriber image_subscriber;
            QStringListModel logging_model;
            QPixmap front_Camera_Pixmap;
    };

}  // namespace ez_gui

#endif /* ez_gui_QNODE_HPP_ */
