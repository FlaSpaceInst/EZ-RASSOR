#ifndef ez_gui_MAIN_WINDOW_H
#define ez_gui_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace ez_gui {

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
            void showButtonTestMessage();

        public Q_SLOTS:
            void on_actionAbout_triggered();
            void on_button_connect_clicked(bool check );
            void on_checkbox_use_environment_stateChanged(int state);

            void updateLoggingView(); // no idea why this can't connect automatically
            void updateCPU();
            void updateVM();
            void updateSM();
            void updateDisk();
            void updateBat();
            void updateFrontCamera();
            void updateBackCamera();

        private:
            Ui::MainWindowDesign ui;
            QNode qnode;
    };

}  // namespace ez_gui

#endif // ez_gui_MAIN_WINDOW_H
