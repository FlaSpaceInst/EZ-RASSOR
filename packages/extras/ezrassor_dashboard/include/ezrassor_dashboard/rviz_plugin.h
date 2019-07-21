//
//

#ifndef RVIZ_PLUGIN_HEADER
#define RVIZ_PLUGIN_HEADER

#include "QObject"
#include "QString"
#include "QWidget"
#include "rviz/display.h"
#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"

class RvizPlugin: public QObject {
    Q_OBJECT

    public:
        enum Type {
            POSE_VIEWER,
            POINT_CLOUD_VIEWER,
            ORIENTATION_VIEWER
        };

        RvizPlugin(QWidget*, RvizPlugin::Type, QString = "");
        ~RvizPlugin(void);

    private:
        rviz::Display* display;
        rviz::RenderPanel* renderPanel;
        rviz::VisualizationManager* visualizationManager;
};

#endif
