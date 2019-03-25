#include <QtGui>
#include <QMessageBox>
#include <QDebug>
#include <iostream>
#include "../include/ez_gui/main_window.hpp"

namespace ez_gui {

    using namespace Qt;

    MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), qnode(argc,argv)
    {
        ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
        QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

        setFixedSize(980,690);

        ReadSettings();
        setWindowIcon(QIcon(":/images/icon.png"));
        ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
        QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

        /*********************
        ** Logging
        **********************/
        ui.view_logging->setModel(qnode.loggingModel());
        QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

        /*********************
        ** Image Viewing
        **********************/
        QObject::connect(&qnode, SIGNAL(frontCamUpdated()), this, SLOT(updateFrontCamera()));
        QObject::connect(&qnode, SIGNAL(backCamUpdated()), this, SLOT(updateBackCamera()));

        // Swampworks Logo
        int w = ui.swamp->width();
        int h = ui.swamp->height();
        QPixmap swamp = QPixmap(":/images/swamp.png");
        ui.swamp->setPixmap(swamp.scaled(w,h,Qt::KeepAspectRatio));

        /*********************
        ** Usage
        **********************/
        QObject::connect(&qnode, SIGNAL(cpuUpdated()), this, SLOT(updateCPU()));
        QObject::connect(&qnode, SIGNAL(vmUpdated()), this, SLOT(updateVM()));
        QObject::connect(&qnode, SIGNAL(smUpdated()), this, SLOT(updateSM()));
        QObject::connect(&qnode, SIGNAL(diskUpdated()), this, SLOT(updateDisk()));
        QObject::connect(&qnode, SIGNAL(batteryUpdated()), this, SLOT(updateBat()));

        /*********************
        ** Node Launch
        **********************/
        QObject::connect(ui.button_launch, SIGNAL(clicked()), this, SLOT(nodeLaunch()));
        /*********************
        ** Auto Start
        **********************/
        if ( ui.checkbox_remember_settings->isChecked() ) {
            on_button_connect_clicked(true);
        }
    }

    MainWindow::~MainWindow() {}

    void MainWindow::showNoMasterMessage()
    {
        QMessageBox msgBox;
        msgBox.setText("Couldn't find the ros master.");
        msgBox.exec();
        close();
    }


    /*
     * These trigger whenever the button is clicked, regardless of whether it
     * is already checked or not.
     */

    void MainWindow::on_button_connect_clicked(bool check )
    {
        if ( ui.checkbox_use_environment->isChecked() )
        {
            if ( !qnode.init() )
            {
                showNoMasterMessage();
            }
            else
            {
                ui.button_connect->setEnabled(false);
            }
        }
        else
        {
            if ( ! qnode.init(ui.line_edit_master->text().toStdString(),ui.line_edit_host->text().toStdString()) )
            {
                showNoMasterMessage();
            }
            else
            {
                ui.button_connect->setEnabled(false);
                ui.line_edit_master->setReadOnly(true);
                ui.line_edit_host->setReadOnly(true);
            }
        }
    }


    void MainWindow::on_checkbox_use_environment_stateChanged(int state)
    {
        bool enabled;
        if ( state == 0 )
        {
            enabled = true;
        }
        else
        {
            enabled = false;
        }
        ui.line_edit_master->setEnabled(enabled);
        ui.line_edit_host->setEnabled(enabled);
    }

    void MainWindow::nodeLaunch()
    {
        //QStringList run;
        QProcess process;
        //run <<  ui.package_launch->text() << ui.node_launch->text();
        process.execute("rostopic list");
    }

    /**
     * This function is signalled by the underlying model. When the model changes,
     * this will drop the cursor down to the last line in the QListview to ensure
     * the user can always see the latest log message.
     */

    void MainWindow::updateLoggingView()
    {
        ui.view_logging->scrollToBottom();
    }

    void MainWindow::updateCPU()
    {
        ui.cpuBar->setValue(*qnode.cpuBarUpdate());
    }

    void MainWindow::updateVM()
    {
        ui.vmBar->setValue(*qnode.vmBarUpdate());
    }

    void MainWindow::updateSM()
    {
        ui.smBar->setValue(*qnode.smBarUpdate());
    }

    void MainWindow::updateDisk()
    {
        ui.dBar->setValue(*qnode.diskBarUpdate());
    }

    void MainWindow::updateBat()
    {
        ui.batBar->setValue(*qnode.batteryBarUpdate());
    }


    void MainWindow::updateFrontCamera()
    {
        int w = ui.front_camera->width();
        int h = ui.front_camera->height();
        QPixmap front = *qnode.frontCameraPixmap();
        ui.front_camera->setPixmap(front.scaled(w,h,Qt::KeepAspectRatio));
    }

    void MainWindow::updateBackCamera()
    {
        int w = ui.back_camera->width();
        int h = ui.back_camera->height();
        QPixmap back = *qnode.backCameraPixmap();
        ui.back_camera->setPixmap(back.scaled(w,h,Qt::KeepAspectRatio));
    }

    void MainWindow::on_actionAbout_triggered()
    {
        QMessageBox::about(this,
                           tr("About EZ-Rassor"),
                           tr("<p>The EZ-RASSOR (EZ Regolith Advanced Surface Systems Operations Robot) is an inexpensive, autonomous, regolith-mining robot designed to mimic the look and abilities of NASAs RASSOR on a smaller scale. The primary goal of the EZ-RASSOR is to provide a functioning demonstration robot for visitors at the Kennedy Space Center."));
    }

    void MainWindow::ReadSettings()
    {
        QSettings settings("Qt-Ros Package", "ez_gui");
        restoreGeometry(settings.value("geometry").toByteArray());
        restoreState(settings.value("windowState").toByteArray());
        QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
        QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();

        ui.line_edit_master->setText(master_url);
        ui.line_edit_host->setText(host_url);

        bool remember = settings.value("remember_settings", false).toBool();
        ui.checkbox_remember_settings->setChecked(remember);

        bool checked = settings.value("use_environment_variables", false).toBool();
        ui.checkbox_use_environment->setChecked(checked);
        if ( checked )
        {
            ui.line_edit_master->setEnabled(false);
            ui.line_edit_host->setEnabled(false);
        }
    }

    void MainWindow::WriteSettings()
    {
        QSettings settings("Qt-Ros Package", "ez_gui");
        settings.setValue("master_url",ui.line_edit_master->text());
        settings.setValue("host_url",ui.line_edit_host->text());
        settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
        settings.setValue("geometry", saveGeometry());
        settings.setValue("windowState", saveState());
        settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

    }

    void MainWindow::closeEvent(QCloseEvent *event)
    {
        WriteSettings();
        QMainWindow::closeEvent(event);
    }

}  // namespace ez_gui

