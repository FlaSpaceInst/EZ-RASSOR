#include <QtGui>
#include <QProcess>
#include <QMessageBox>
#include <iostream>
#include <cstdlib>
#include <cstddef>
#include <string>
#include <unistd.h>
#include "../include/ros_gui/main_window.hpp"

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    setFixedSize(975,640);

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Rviz
    **********************/
    QObject::connect(&qnode, SIGNAL(startingRviz()), this, SLOT(startRviz()));

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
    QObject::connect(&qnode, SIGNAL(disparityUpdated()), this, SLOT(updateDisparityCamera()));

    /*********************
    ** IMU Labels
    **********************/
    QObject::connect(&qnode, SIGNAL(imuLabelsUpdated()), this, SLOT(updateImuLabels()));

    /*********************
    ** Swampworks Logo
    **********************/
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
    populateLaunchers();
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
}

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

void MainWindow::startRviz()
{
    RvizPlugin point(this->ui.pc_viewer, 1);
    RvizPlugin imu(this->ui.imu_viewer, 2);
    RvizPlugin pose(this->ui.pose_viewer, 3);
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

void MainWindow::updateImuLabels()
{
    ui.orient_x->setText("<b>x:</b> " + QString::number(qnode.imu_labels[0]));
    ui.orient_y->setText("<b>y:</b> " + QString::number(qnode.imu_labels[1]));
    ui.orient_z->setText("<b>z:</b> " + QString::number(qnode.imu_labels[2]));

    ui.ang_x->setText("<b>x:</b> " + QString::number(qnode.imu_labels[3]));
    ui.ang_y->setText("<b>y:</b> " + QString::number(qnode.imu_labels[4]));
    ui.ang_z->setText("<b>z:</b> " + QString::number(qnode.imu_labels[5]));

    ui.lin_x->setText("<b>x:</b> " + QString::number(qnode.imu_labels[6]));
    ui.lin_y->setText("<b>y:</b> " + QString::number(qnode.imu_labels[7]));
    ui.lin_z->setText("<b>z:</b> " + QString::number(qnode.imu_labels[8]));
}

void MainWindow::updateDisparityCamera()
{
    int w = ui.disparity_camera->width();
    int h = ui.disparity_camera->height();
    QPixmap disparity = *qnode.disparityPixmap();
    ui.disparity_camera->setPixmap(disparity.scaled(w,h,Qt::KeepAspectRatio));
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

/* Launches a node selected in the drop-down menu
 */
void MainWindow::nodeLaunch()
{
    QString executable = ui.comboBox->currentText();

    char buffer[128];

    pid_t pid = fork();

    // parent process does this and ends here
    if (pid != 0) {
        qnode.addProcess(pid);
        return;
    }

    // child process ----
    FILE * fprocess = popen(("roslaunch " + qnode.get_executable_package(executable).toStdString() + " " + executable.toStdString()).c_str(), "r");
    
    try {
        while (fgets(buffer, sizeof buffer, fprocess))
            std::cout << buffer << std::endl;
    } catch (...) {
        pclose(fprocess);
    }
}

/* Extract node and package from full path */
std::pair<QString, QString> executable_and_package_from_path(const char *path)
{
    std::stringstream entirePath(path);
    std::string grab;
    std::string package, executable;
    std::size_t lastSlash;
    int dir_index = 0;

    for (int i = 0; i < 3; i++)
        std::getline(entirePath, grab, '/');
    std::getline(entirePath, package, '/');
    
    std::string patharoni = entirePath.str();
    lastSlash = patharoni.find_last_of('/');

    return std::make_pair(QString::fromStdString(patharoni.substr(lastSlash + 1)), QString::fromStdString(package));
}

/* Populates the drop-down with everything that can be launched
 * for launching from the GUI
 */
void MainWindow::populateLaunchers()
{
    char fprocess_output_buffer[128];
    QStringList executable_paths;
    std::pair<QString, QString> executable_package_pair;

    FILE* fprocess = popen("find . -name \"*.launch\"", "r");
    if (!fprocess)
        std::cout << "Couldn't open find fprocess" << std::endl;

    while (fgets(fprocess_output_buffer, sizeof fprocess_output_buffer, fprocess))
    {
        executable_package_pair = executable_and_package_from_path(fprocess_output_buffer);
        qnode.add_executable_package(executable_package_pair.first, executable_package_pair.second);
        executable_paths << executable_package_pair.first;
    }

    pclose(fprocess);
   
    ui.comboBox->addItems(executable_paths);
}
