#ifndef ez_gui_MAIN_WINDOW_H
#define ez_gui_MAIN_WINDOW_H

//#include <QtGui/QMainWindow> //qt4/qt5
#include "QMainWindow"
#include "ui_main_window.h"
#include "qnode.hpp"

#include <qfiledialog.h>
#include <qfileinfo.h>
#include <qlistview.h>
#include <qstringlistmodel.h>
#include <QThread>

//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/view_manager.h>

class RvizPlugin: public QObject{
    Q_OBJECT
public:
    RvizPlugin(QVBoxLayout* ui){
        rviz_panel = new rviz::RenderPanel;
        rvizManager =  new rviz::VisualizationManager(rviz_panel);
        //viewManager = new rviz::ViewManager();


        rviz_panel->initialize(rvizManager->getSceneManager(), rvizManager);
        rviz_panel->setBackgroundColor( Ogre::ColourValue(0, 0,0,0.3)); //no use
        ui->addWidget(rviz_panel);

        rvizManager->initialize();
        rvizManager->startUpdate();
        rvizManager->setFixedFrame("camera_init");
        rviz::Display *grid = rvizManager->createDisplay("rviz/Grid","Grid",true);

        viewManager = rvizManager->getViewManager();
        viewManager->setRenderPanel(rviz_panel);
        viewManager->setCurrentViewControllerType("rviz/Orbit");
        viewManager->getCurrent()->subProp("Target Frame")->setValue("/aft_mapped");

        enablePointCloud2("/ez_rassor/front_camera/points2", "FlatColor", "1");
        rviz::Display* frameRgister=enablePointCloud2("/velodyne_cloud_registered", "Intensity", "2");
        enableOdometry("/integrated_to_init", "10000", "Axes");

    }

    void enableOdometry(QString topic, QString keep, QString shape){
        rviz::Display *odometry = rvizManager->createDisplay("rviz/Odometry","Odometry", true);
        odometry->subProp("Topic")->setValue(topic);//"/apollo/sensor/velodyne64/PointCloud2");
        odometry->subProp("Keep")->setValue(keep);
        odometry->subProp("Shape")->setValue(shape);
    }

    rviz::Display* enablePointCloud2(QString topic, QString corlorTransform, QString size){
        rviz::Display *pointCloud = rvizManager->createDisplay("rviz/PointCloud2","PointCloud2", true);
        pointCloud->subProp("Topic")->setValue(topic);//"/apollo/sensor/velodyne64/PointCloud2");
        pointCloud->subProp("Style")->setValue("Points");
        pointCloud->subProp("Size (Pixels)")->setValue(size);
        pointCloud->subProp("Color Transformer")->setValue(corlorTransform);
        pointCloud->subProp("Invert Rainbow")->setValue("true");
        return pointCloud;
    }


    public slots:
        void on_pushButton_clicked(){std::cout<<3<<std::endl;}
        void hhh(){std::cout<<2<<std::endl;}

    private:
        rviz::RenderPanel* rviz_panel;// = new rviz::RenderPanel;
        rviz::VisualizationManager *rvizManager;// =  new rviz::VisualizationManager(pointCloud_panel);
        rviz::ViewManager* viewManager;
};


class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        MainWindow(int argc, char** argv, QWidget *parent = 0);
        ~MainWindow();

        void ReadSettings(); // Load up qt program settings at startup
        void WriteSettings(); // Save qt program settings when closing

        void closeEvent(QCloseEvent *event); // Overloaded function
        void showNoMasterMessage();

    public Q_SLOTS:
        void on_actionAbout_triggered();
        void on_button_connect_clicked(bool check );
        void on_checkbox_use_environment_stateChanged(int state);

        void nodeLaunch();
        void updateLoggingView();
        void updateCPU();
        void updateVM();
        void updateSM();
        void updateDisk();
        void updateBat();
        void updateFrontCamera();
        void updateBackCamera();
        void updateDisparityCamera();
        void startPointCloud();

    private:
        Ui::MainWindowDesign ui;
        QNode qnode;
};

#endif // ros_gui_MAIN_WINDOW_H
