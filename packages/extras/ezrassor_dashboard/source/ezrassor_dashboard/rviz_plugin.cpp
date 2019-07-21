//
//

#include "QObject"
#include "QWidget"
#include "rviz/display.h"
#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz_plugin.h"

RvizPlugin::RvizPlugin(QWidget* parent, RvizPlugin::Type type, QString topic) {
    renderPanel = new rviz::RenderPanel(parent);
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

        case RvizPlugin::ORIENTATION_VIEWER:
            display = visualizationManager->createDisplay(
                "rviz/Imu",
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
