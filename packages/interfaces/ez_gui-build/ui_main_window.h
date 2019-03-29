/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_image;
    QLabel *front_camera;
    QLabel *back_camera;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *disparity_camera;
    QLabel *label_11;
    QLabel *swamp;
    QWidget *tab_imu;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *imu_viewer;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label_4;
    QLabel *orient_x;
    QLabel *orient_y;
    QLabel *orient_z;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_15;
    QLabel *ang_x;
    QLabel *ang_y;
    QLabel *ang_z;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_17;
    QLabel *lin_x;
    QLabel *lin_y;
    QLabel *lin_z;
    QWidget *tab_usage;
    QProgressBar *cpuBar;
    QProgressBar *vmBar;
    QProgressBar *smBar;
    QProgressBar *dBar;
    QLabel *cpu_label;
    QLabel *vm_label;
    QProgressBar *batBar;
    QLabel *usage_label_3;
    QLabel *usage_label_;
    QLabel *label_8;
    QWidget *tab_pc;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *pc_viewer;
    QWidget *tab;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *pose_viewer;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QGridLayout *gridLayout_2;
    QCheckBox *checkbox_use_environment;
    QLabel *label_12;
    QLineEdit *line_edit_host;
    QPushButton *button_connect;
    QLabel *label_3;
    QComboBox *comboBox;
    QLabel *label_2;
    QLabel *label;
    QPushButton *button_launch;
    QLineEdit *line_edit_master;
    QCheckBox *checkbox_remember_settings;
    QPushButton *quit_button;
    QListView *view_logging;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QStringLiteral("MainWindowDesign"));
        MainWindowDesign->resize(975, 640);
        MainWindowDesign->setMinimumSize(QSize(975, 640));
        MainWindowDesign->setMaximumSize(QSize(975, 640));
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QStringLiteral("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QStringLiteral("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QStringLiteral("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QStringLiteral("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QStringLiteral("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setMaximumSize(QSize(16777215, 600));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_manager->setTabShape(QTabWidget::Rounded);
        tab_image = new QWidget();
        tab_image->setObjectName(QStringLiteral("tab_image"));
        QFont font;
        font.setBold(false);
        font.setWeight(50);
        tab_image->setFont(font);
        front_camera = new QLabel(tab_image);
        front_camera->setObjectName(QStringLiteral("front_camera"));
        front_camera->setGeometry(QRect(10, 50, 281, 211));
        front_camera->setFrameShape(QFrame::StyledPanel);
        back_camera = new QLabel(tab_image);
        back_camera->setObjectName(QStringLiteral("back_camera"));
        back_camera->setGeometry(QRect(300, 50, 281, 211));
        back_camera->setFrameShape(QFrame::StyledPanel);
        back_camera->setFrameShadow(QFrame::Plain);
        label_9 = new QLabel(tab_image);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(10, 10, 171, 31));
        label_10 = new QLabel(tab_image);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(300, 10, 171, 31));
        disparity_camera = new QLabel(tab_image);
        disparity_camera->setObjectName(QStringLiteral("disparity_camera"));
        disparity_camera->setGeometry(QRect(10, 300, 281, 211));
        disparity_camera->setFrameShape(QFrame::StyledPanel);
        label_11 = new QLabel(tab_image);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(10, 270, 181, 31));
        swamp = new QLabel(tab_image);
        swamp->setObjectName(QStringLiteral("swamp"));
        swamp->setGeometry(QRect(300, 310, 281, 211));
        swamp->setFrameShape(QFrame::NoFrame);
        swamp->setFrameShadow(QFrame::Plain);
        tab_manager->addTab(tab_image, QString());
        tab_imu = new QWidget();
        tab_imu->setObjectName(QStringLiteral("tab_imu"));
        verticalLayoutWidget_2 = new QWidget(tab_imu);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(10, 10, 571, 401));
        imu_viewer = new QVBoxLayout(verticalLayoutWidget_2);
        imu_viewer->setObjectName(QStringLiteral("imu_viewer"));
        imu_viewer->setContentsMargins(0, 0, 0, 0);
        layoutWidget = new QWidget(tab_imu);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 415, 571, 121));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        QFont font1;
        font1.setBold(true);
        font1.setUnderline(true);
        font1.setWeight(75);
        label_4->setFont(font1);
        label_4->setLayoutDirection(Qt::LeftToRight);
        label_4->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        verticalLayout->addWidget(label_4);

        orient_x = new QLabel(layoutWidget);
        orient_x->setObjectName(QStringLiteral("orient_x"));

        verticalLayout->addWidget(orient_x);

        orient_y = new QLabel(layoutWidget);
        orient_y->setObjectName(QStringLiteral("orient_y"));

        verticalLayout->addWidget(orient_y);

        orient_z = new QLabel(layoutWidget);
        orient_z->setObjectName(QStringLiteral("orient_z"));

        verticalLayout->addWidget(orient_z);


        horizontalLayout->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        label_15 = new QLabel(layoutWidget);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setFont(font1);
        label_15->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        verticalLayout_2->addWidget(label_15);

        ang_x = new QLabel(layoutWidget);
        ang_x->setObjectName(QStringLiteral("ang_x"));

        verticalLayout_2->addWidget(ang_x);

        ang_y = new QLabel(layoutWidget);
        ang_y->setObjectName(QStringLiteral("ang_y"));

        verticalLayout_2->addWidget(ang_y);

        ang_z = new QLabel(layoutWidget);
        ang_z->setObjectName(QStringLiteral("ang_z"));

        verticalLayout_2->addWidget(ang_z);


        horizontalLayout->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        label_17 = new QLabel(layoutWidget);
        label_17->setObjectName(QStringLiteral("label_17"));
        QFont font2;
        font2.setBold(true);
        font2.setUnderline(true);
        font2.setWeight(75);
        font2.setStrikeOut(false);
        label_17->setFont(font2);
        label_17->setTextFormat(Qt::RichText);
        label_17->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_3->addWidget(label_17);

        lin_x = new QLabel(layoutWidget);
        lin_x->setObjectName(QStringLiteral("lin_x"));

        verticalLayout_3->addWidget(lin_x);

        lin_y = new QLabel(layoutWidget);
        lin_y->setObjectName(QStringLiteral("lin_y"));

        verticalLayout_3->addWidget(lin_y);

        lin_z = new QLabel(layoutWidget);
        lin_z->setObjectName(QStringLiteral("lin_z"));

        verticalLayout_3->addWidget(lin_z);


        horizontalLayout->addLayout(verticalLayout_3);

        tab_manager->addTab(tab_imu, QString());
        tab_usage = new QWidget();
        tab_usage->setObjectName(QStringLiteral("tab_usage"));
        cpuBar = new QProgressBar(tab_usage);
        cpuBar->setObjectName(QStringLiteral("cpuBar"));
        cpuBar->setGeometry(QRect(120, 30, 461, 61));
        cpuBar->setValue(24);
        vmBar = new QProgressBar(tab_usage);
        vmBar->setObjectName(QStringLiteral("vmBar"));
        vmBar->setGeometry(QRect(120, 120, 461, 61));
        vmBar->setValue(24);
        smBar = new QProgressBar(tab_usage);
        smBar->setObjectName(QStringLiteral("smBar"));
        smBar->setGeometry(QRect(120, 210, 461, 61));
        smBar->setValue(24);
        dBar = new QProgressBar(tab_usage);
        dBar->setObjectName(QStringLiteral("dBar"));
        dBar->setGeometry(QRect(120, 300, 461, 61));
        dBar->setValue(24);
        cpu_label = new QLabel(tab_usage);
        cpu_label->setObjectName(QStringLiteral("cpu_label"));
        cpu_label->setGeometry(QRect(30, 30, 81, 71));
        vm_label = new QLabel(tab_usage);
        vm_label->setObjectName(QStringLiteral("vm_label"));
        vm_label->setGeometry(QRect(20, 120, 91, 71));
        batBar = new QProgressBar(tab_usage);
        batBar->setObjectName(QStringLiteral("batBar"));
        batBar->setGeometry(QRect(120, 390, 461, 61));
        batBar->setValue(24);
        usage_label_3 = new QLabel(tab_usage);
        usage_label_3->setObjectName(QStringLiteral("usage_label_3"));
        usage_label_3->setGeometry(QRect(20, 210, 91, 71));
        usage_label_ = new QLabel(tab_usage);
        usage_label_->setObjectName(QStringLiteral("usage_label_"));
        usage_label_->setGeometry(QRect(30, 300, 81, 71));
        label_8 = new QLabel(tab_usage);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(10, 390, 101, 61));
        tab_manager->addTab(tab_usage, QString());
        tab_pc = new QWidget();
        tab_pc->setObjectName(QStringLiteral("tab_pc"));
        verticalLayoutWidget = new QWidget(tab_pc);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 10, 581, 541));
        pc_viewer = new QVBoxLayout(verticalLayoutWidget);
        pc_viewer->setObjectName(QStringLiteral("pc_viewer"));
        pc_viewer->setContentsMargins(0, 0, 0, 0);
        tab_manager->addTab(tab_pc, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        verticalLayoutWidget_3 = new QWidget(tab);
        verticalLayoutWidget_3->setObjectName(QStringLiteral("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(10, 10, 581, 541));
        pose_viewer = new QVBoxLayout(verticalLayoutWidget_3);
        pose_viewer->setObjectName(QStringLiteral("pose_viewer"));
        pose_viewer->setContentsMargins(0, 0, 0, 0);
        tab_manager->addTab(tab, QString());

        hboxLayout->addWidget(tab_manager);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setMinimumSize(QSize(350, 586));
        groupBox->setMaximumSize(QSize(350, 586));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        checkbox_use_environment = new QCheckBox(groupBox);
        checkbox_use_environment->setObjectName(QStringLiteral("checkbox_use_environment"));
        checkbox_use_environment->setLayoutDirection(Qt::RightToLeft);

        gridLayout_2->addWidget(checkbox_use_environment, 4, 0, 1, 1);

        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout_2->addWidget(label_12, 10, 0, 1, 1);

        line_edit_host = new QLineEdit(groupBox);
        line_edit_host->setObjectName(QStringLiteral("line_edit_host"));

        gridLayout_2->addWidget(line_edit_host, 3, 0, 1, 1);

        button_connect = new QPushButton(groupBox);
        button_connect->setObjectName(QStringLiteral("button_connect"));
        button_connect->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(button_connect->sizePolicy().hasHeightForWidth());
        button_connect->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(button_connect, 6, 0, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setFrameShape(QFrame::NoFrame);
        label_3->setFrameShadow(QFrame::Raised);

        gridLayout_2->addWidget(label_3, 7, 0, 1, 1);

        comboBox = new QComboBox(groupBox);
        comboBox->setObjectName(QStringLiteral("comboBox"));

        gridLayout_2->addWidget(comboBox, 8, 0, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setFrameShape(QFrame::NoFrame);
        label_2->setFrameShadow(QFrame::Raised);

        gridLayout_2->addWidget(label_2, 2, 0, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setFrameShape(QFrame::NoFrame);
        label->setFrameShadow(QFrame::Raised);

        gridLayout_2->addWidget(label, 0, 0, 1, 1);

        button_launch = new QPushButton(groupBox);
        button_launch->setObjectName(QStringLiteral("button_launch"));

        gridLayout_2->addWidget(button_launch, 9, 0, 1, 1);

        line_edit_master = new QLineEdit(groupBox);
        line_edit_master->setObjectName(QStringLiteral("line_edit_master"));

        gridLayout_2->addWidget(line_edit_master, 1, 0, 1, 1);

        checkbox_remember_settings = new QCheckBox(groupBox);
        checkbox_remember_settings->setObjectName(QStringLiteral("checkbox_remember_settings"));
        checkbox_remember_settings->setLayoutDirection(Qt::RightToLeft);

        gridLayout_2->addWidget(checkbox_remember_settings, 5, 0, 1, 1);

        quit_button = new QPushButton(groupBox);
        quit_button->setObjectName(QStringLiteral("quit_button"));
        sizePolicy.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(quit_button, 12, 0, 1, 1);

        view_logging = new QListView(groupBox);
        view_logging->setObjectName(QStringLiteral("view_logging"));

        gridLayout_2->addWidget(view_logging, 11, 0, 1, 1);


        gridLayout->addLayout(gridLayout_2, 0, 0, 1, 1);


        hboxLayout->addWidget(groupBox);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 975, 28));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QStringLiteral("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        menubar->addAction(menu_File->menuAction());
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));

        tab_manager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "EZ-RASSOR Monitoring System", 0));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0));
        front_camera->setText(QString());
        back_camera->setText(QString());
        label_9->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-size:16pt; font-weight:600; font-style:italic;\">Left Camera</span></p></body></html>", 0));
        label_10->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-size:16pt; font-weight:600; font-style:italic;\">Right Camera</span></p></body></html>", 0));
        disparity_camera->setText(QString());
        label_11->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-size:16pt; font-weight:600; font-style:italic;\">Disparity Map</span></p></body></html>", 0));
        swamp->setText(QString());
        tab_manager->setTabText(tab_manager->indexOf(tab_image), QApplication::translate("MainWindowDesign", "Video Feed", 0));
        label_4->setText(QApplication::translate("MainWindowDesign", "Orientation", 0));
        orient_x->setText(QApplication::translate("MainWindowDesign", "x: ", 0));
        orient_y->setText(QApplication::translate("MainWindowDesign", "y:", 0));
        orient_z->setText(QApplication::translate("MainWindowDesign", "z:", 0));
        label_15->setText(QApplication::translate("MainWindowDesign", "Angular Velocity", 0));
        ang_x->setText(QApplication::translate("MainWindowDesign", "x:", 0));
        ang_y->setText(QApplication::translate("MainWindowDesign", "y:", 0));
        ang_z->setText(QApplication::translate("MainWindowDesign", "z:", 0));
        label_17->setText(QApplication::translate("MainWindowDesign", "Linear Acceleration", 0));
        lin_x->setText(QApplication::translate("MainWindowDesign", "x:", 0));
        lin_y->setText(QApplication::translate("MainWindowDesign", "y:", 0));
        lin_z->setText(QApplication::translate("MainWindowDesign", "z:", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab_imu), QApplication::translate("MainWindowDesign", "IMU Display", 0));
        cpu_label->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p align=\"right\"><span style=\" font-size:14pt; font-weight:600; font-style:italic;\">CPU </span></p><p align=\"right\"><span style=\" font-size:14pt; font-weight:600; font-style:italic;\"> Usage </span></p></body></html>", 0));
        vm_label->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p align=\"right\"><span style=\" font-size:14pt; font-weight:600; font-style:italic;\">Virtual </span></p><p align=\"right\"><span style=\" font-size:14pt; font-weight:600; font-style:italic;\">Memory </span></p></body></html>", 0));
        usage_label_3->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p align=\"right\"><span style=\" font-size:14pt; font-weight:600; font-style:italic;\">Swap </span></p><p align=\"right\"><span style=\" font-size:14pt; font-weight:600; font-style:italic;\">Memory </span></p></body></html>", 0));
        usage_label_->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p align=\"right\"><span style=\" font-size:14pt; font-weight:600; font-style:italic;\">Disk </span></p><p align=\"right\"><span style=\" font-size:14pt; font-weight:600; font-style:italic;\"> Usage </span></p></body></html>", 0));
        label_8->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p align=\"right\"><span style=\" font-size:14pt; font-weight:600; font-style:italic;\">Battery </span></p></body></html>", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab_usage), QApplication::translate("MainWindowDesign", "Usage", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab_pc), QApplication::translate("MainWindowDesign", "Point Cloud", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab), QApplication::translate("MainWindowDesign", "Pose Viewer", 0));
        groupBox->setTitle(QApplication::translate("MainWindowDesign", "    Ros Master", 0));
        checkbox_use_environment->setText(QApplication::translate("MainWindowDesign", "Use environment variables", 0));
        label_12->setText(QApplication::translate("MainWindowDesign", "EZ-RASSOR Messages", 0));
        line_edit_host->setText(QApplication::translate("MainWindowDesign", "192.168.1.67", 0));
#ifndef QT_NO_TOOLTIP
        button_connect->setToolTip(QApplication::translate("MainWindowDesign", "Set the target to the current joint trajectory state.", 0));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        button_connect->setStatusTip(QApplication::translate("MainWindowDesign", "Clear all waypoints and set the target to the current joint trajectory state.", 0));
#endif // QT_NO_STATUSTIP
        button_connect->setText(QApplication::translate("MainWindowDesign", "Connect", 0));
        label_3->setText(QApplication::translate("MainWindowDesign", "Launcher", 0));
        label_2->setText(QApplication::translate("MainWindowDesign", "Host IP", 0));
        label->setText(QApplication::translate("MainWindowDesign", "Ros Master Url", 0));
        button_launch->setText(QApplication::translate("MainWindowDesign", "Launch", 0));
        line_edit_master->setText(QApplication::translate("MainWindowDesign", "http://192.168.1.2:11311/", 0));
        checkbox_remember_settings->setText(QApplication::translate("MainWindowDesign", "Remember settings on startup", 0));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "Info", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
