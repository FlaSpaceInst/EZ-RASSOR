//
//

#include <QObject>
#include "rviz_plugin.h"
#include "rviz/display.h"
#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"

RvizPlugin::RvizPlugin(RvizPlugin::Type type, QString topic) {
    renderPanel = new rviz::RenderPanel();
    visualizationManager = new rviz::VisualizationManager(renderPanel);
    renderPanel->initialize(
        visualizationManager->getSceneManager(),
        visualizationManager
    );
    visualizationManager->initialize();
    visualizationManager->startUpdate();

    switch (type) {
        case RvizPlugin::POINT_CLOUD_VIEWER:
            display = visualizationManager->createDisplay(
                "rviz/PointCloud2",
                "PointCloud2",
                true
            );
            display->subProp("Topic")->setValue(topic);
            break;

        case RvizPlugin::IMU_VIEWER:
            display = visualizationManager->createDisplay(
                "rviz_plugin_tutorials/Imu",
                "Imu",
                true
            );
            display->subProp("Topic")->setValue(topic);
            break;

        case RvizPlugin::POSE_VIEWER:
            display = visualizationManager->createDisplay(
                "rviz/RobotModel",
                "RobotModel",
                true
            );
            break;
    }
}

RvizPlugin::~RvizPlugin(void) {
    delete display;
    delete renderPanel;
    delete visualizationManager;
}
