/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QTabWidget *tabWidget;
    QWidget *GeneralTab;
    QWidget *StatusTab;
    QLabel *CPUUsageLabel;
    QProgressBar *CPUUsageBar;
    QTextBrowser *LogBrowser;
    QWidget *CameraTab;
    QFrame *RightCameraFrame;
    QLabel *RightCameraLabel;
    QFrame *LeftCameraFrame;
    QLabel *LeftCameraLabel;
    QWidget *MapTab;
    QFrame *PointCloudFrame;
    QFrame *DisparityMapFrame;
    QLabel *PointCloudLabel;
    QLabel *DisparityMapLabel;
    QWidget *OrientationTab;
    QWidget *layoutWidget;
    QHBoxLayout *OrientationDataBoxes;
    QVBoxLayout *OrientationDataBox;
    QLabel *OrientationDataBoxLabel;
    QLabel *OrientationXLabel;
    QLabel *OrientationYLabel;
    QLabel *OrientationZLabel;
    QVBoxLayout *AngularVelocityDataBox;
    QLabel *AngularVelocityDataBoxLabel;
    QLabel *AngularVelocityXLabel;
    QLabel *AngularVelocityYLabel;
    QLabel *AngularVelocityZLabel;
    QVBoxLayout *LinearAccelerationDataBox;
    QLabel *LinearAccelerationDataBoxLabel;
    QLabel *LinearAccelerationXLabel;
    QLabel *LinearAccelerationYLabel;
    QLabel *LinearAccelerationZLabel;
    QFrame *OrientationFrame;
    QWidget *PoseTab;
    QFrame *PoseFrame;

    void setupUi(QWidget *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(810, 630);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(810, 630));
        MainWindow->setMaximumSize(QSize(810, 630));
        tabWidget = new QTabWidget(MainWindow);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(10, 10, 791, 611));
        GeneralTab = new QWidget();
        GeneralTab->setObjectName(QStringLiteral("GeneralTab"));
        tabWidget->addTab(GeneralTab, QString());
        StatusTab = new QWidget();
        StatusTab->setObjectName(QStringLiteral("StatusTab"));
        CPUUsageLabel = new QLabel(StatusTab);
        CPUUsageLabel->setObjectName(QStringLiteral("CPUUsageLabel"));
        CPUUsageLabel->setGeometry(QRect(100, 520, 111, 41));
        QFont font;
        font.setFamily(QStringLiteral("Ubuntu Light"));
        font.setPointSize(14);
        font.setItalic(false);
        CPUUsageLabel->setFont(font);
        CPUUsageBar = new QProgressBar(StatusTab);
        CPUUsageBar->setObjectName(QStringLiteral("CPUUsageBar"));
        CPUUsageBar->setGeometry(QRect(220, 520, 461, 41));
        QFont font1;
        font1.setFamily(QStringLiteral("Ubuntu Light"));
        CPUUsageBar->setFont(font1);
        CPUUsageBar->setValue(42);
        LogBrowser = new QTextBrowser(StatusTab);
        LogBrowser->setObjectName(QStringLiteral("LogBrowser"));
        LogBrowser->setGeometry(QRect(10, 10, 771, 491));
        tabWidget->addTab(StatusTab, QString());
        CameraTab = new QWidget();
        CameraTab->setObjectName(QStringLiteral("CameraTab"));
        RightCameraFrame = new QFrame(CameraTab);
        RightCameraFrame->setObjectName(QStringLiteral("RightCameraFrame"));
        RightCameraFrame->setGeometry(QRect(400, 100, 361, 341));
        RightCameraFrame->setFrameShape(QFrame::StyledPanel);
        RightCameraFrame->setFrameShadow(QFrame::Raised);
        RightCameraLabel = new QLabel(CameraTab);
        RightCameraLabel->setObjectName(QStringLiteral("RightCameraLabel"));
        RightCameraLabel->setGeometry(QRect(510, 450, 121, 41));
        RightCameraLabel->setFont(font);
        LeftCameraFrame = new QFrame(CameraTab);
        LeftCameraFrame->setObjectName(QStringLiteral("LeftCameraFrame"));
        LeftCameraFrame->setGeometry(QRect(20, 100, 361, 341));
        LeftCameraFrame->setFrameShape(QFrame::StyledPanel);
        LeftCameraFrame->setFrameShadow(QFrame::Raised);
        LeftCameraLabel = new QLabel(CameraTab);
        LeftCameraLabel->setObjectName(QStringLiteral("LeftCameraLabel"));
        LeftCameraLabel->setGeometry(QRect(130, 450, 121, 41));
        LeftCameraLabel->setFont(font);
        tabWidget->addTab(CameraTab, QString());
        MapTab = new QWidget();
        MapTab->setObjectName(QStringLiteral("MapTab"));
        PointCloudFrame = new QFrame(MapTab);
        PointCloudFrame->setObjectName(QStringLiteral("PointCloudFrame"));
        PointCloudFrame->setGeometry(QRect(20, 100, 361, 341));
        PointCloudFrame->setFrameShape(QFrame::StyledPanel);
        PointCloudFrame->setFrameShadow(QFrame::Raised);
        DisparityMapFrame = new QFrame(MapTab);
        DisparityMapFrame->setObjectName(QStringLiteral("DisparityMapFrame"));
        DisparityMapFrame->setGeometry(QRect(400, 100, 361, 341));
        DisparityMapFrame->setFrameShape(QFrame::StyledPanel);
        DisparityMapFrame->setFrameShadow(QFrame::Raised);
        PointCloudLabel = new QLabel(MapTab);
        PointCloudLabel->setObjectName(QStringLiteral("PointCloudLabel"));
        PointCloudLabel->setGeometry(QRect(130, 450, 121, 41));
        PointCloudLabel->setFont(font);
        DisparityMapLabel = new QLabel(MapTab);
        DisparityMapLabel->setObjectName(QStringLiteral("DisparityMapLabel"));
        DisparityMapLabel->setGeometry(QRect(510, 450, 121, 41));
        DisparityMapLabel->setFont(font);
        tabWidget->addTab(MapTab, QString());
        OrientationTab = new QWidget();
        OrientationTab->setObjectName(QStringLiteral("OrientationTab"));
        layoutWidget = new QWidget(OrientationTab);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(110, 450, 571, 121));
        OrientationDataBoxes = new QHBoxLayout(layoutWidget);
        OrientationDataBoxes->setObjectName(QStringLiteral("OrientationDataBoxes"));
        OrientationDataBoxes->setContentsMargins(0, 0, 0, 0);
        OrientationDataBox = new QVBoxLayout();
        OrientationDataBox->setObjectName(QStringLiteral("OrientationDataBox"));
        OrientationDataBox->setSizeConstraint(QLayout::SetDefaultConstraint);
        OrientationDataBoxLabel = new QLabel(layoutWidget);
        OrientationDataBoxLabel->setObjectName(QStringLiteral("OrientationDataBoxLabel"));
        QFont font2;
        font2.setFamily(QStringLiteral("Ubuntu Light"));
        font2.setPointSize(13);
        font2.setBold(false);
        font2.setUnderline(false);
        font2.setWeight(50);
        OrientationDataBoxLabel->setFont(font2);
        OrientationDataBoxLabel->setLayoutDirection(Qt::LeftToRight);
        OrientationDataBoxLabel->setAlignment(Qt::AlignCenter);

        OrientationDataBox->addWidget(OrientationDataBoxLabel);

        OrientationXLabel = new QLabel(layoutWidget);
        OrientationXLabel->setObjectName(QStringLiteral("OrientationXLabel"));
        QFont font3;
        font3.setFamily(QStringLiteral("Ubuntu Light"));
        font3.setPointSize(12);
        font3.setKerning(true);
        OrientationXLabel->setFont(font3);

        OrientationDataBox->addWidget(OrientationXLabel);

        OrientationYLabel = new QLabel(layoutWidget);
        OrientationYLabel->setObjectName(QStringLiteral("OrientationYLabel"));
        QFont font4;
        font4.setFamily(QStringLiteral("Ubuntu Light"));
        font4.setPointSize(12);
        OrientationYLabel->setFont(font4);

        OrientationDataBox->addWidget(OrientationYLabel);

        OrientationZLabel = new QLabel(layoutWidget);
        OrientationZLabel->setObjectName(QStringLiteral("OrientationZLabel"));
        OrientationZLabel->setFont(font4);

        OrientationDataBox->addWidget(OrientationZLabel);


        OrientationDataBoxes->addLayout(OrientationDataBox);

        AngularVelocityDataBox = new QVBoxLayout();
        AngularVelocityDataBox->setObjectName(QStringLiteral("AngularVelocityDataBox"));
        AngularVelocityDataBoxLabel = new QLabel(layoutWidget);
        AngularVelocityDataBoxLabel->setObjectName(QStringLiteral("AngularVelocityDataBoxLabel"));
        AngularVelocityDataBoxLabel->setFont(font2);
        AngularVelocityDataBoxLabel->setAlignment(Qt::AlignCenter);

        AngularVelocityDataBox->addWidget(AngularVelocityDataBoxLabel);

        AngularVelocityXLabel = new QLabel(layoutWidget);
        AngularVelocityXLabel->setObjectName(QStringLiteral("AngularVelocityXLabel"));
        AngularVelocityXLabel->setFont(font4);

        AngularVelocityDataBox->addWidget(AngularVelocityXLabel);

        AngularVelocityYLabel = new QLabel(layoutWidget);
        AngularVelocityYLabel->setObjectName(QStringLiteral("AngularVelocityYLabel"));
        AngularVelocityYLabel->setFont(font4);

        AngularVelocityDataBox->addWidget(AngularVelocityYLabel);

        AngularVelocityZLabel = new QLabel(layoutWidget);
        AngularVelocityZLabel->setObjectName(QStringLiteral("AngularVelocityZLabel"));
        AngularVelocityZLabel->setFont(font4);

        AngularVelocityDataBox->addWidget(AngularVelocityZLabel);


        OrientationDataBoxes->addLayout(AngularVelocityDataBox);

        LinearAccelerationDataBox = new QVBoxLayout();
        LinearAccelerationDataBox->setObjectName(QStringLiteral("LinearAccelerationDataBox"));
        LinearAccelerationDataBoxLabel = new QLabel(layoutWidget);
        LinearAccelerationDataBoxLabel->setObjectName(QStringLiteral("LinearAccelerationDataBoxLabel"));
        QFont font5;
        font5.setFamily(QStringLiteral("Ubuntu Light"));
        font5.setPointSize(13);
        font5.setBold(false);
        font5.setUnderline(false);
        font5.setWeight(50);
        font5.setStrikeOut(false);
        LinearAccelerationDataBoxLabel->setFont(font5);
        LinearAccelerationDataBoxLabel->setTextFormat(Qt::RichText);
        LinearAccelerationDataBoxLabel->setAlignment(Qt::AlignCenter);

        LinearAccelerationDataBox->addWidget(LinearAccelerationDataBoxLabel);

        LinearAccelerationXLabel = new QLabel(layoutWidget);
        LinearAccelerationXLabel->setObjectName(QStringLiteral("LinearAccelerationXLabel"));
        LinearAccelerationXLabel->setFont(font4);

        LinearAccelerationDataBox->addWidget(LinearAccelerationXLabel);

        LinearAccelerationYLabel = new QLabel(layoutWidget);
        LinearAccelerationYLabel->setObjectName(QStringLiteral("LinearAccelerationYLabel"));
        LinearAccelerationYLabel->setFont(font4);

        LinearAccelerationDataBox->addWidget(LinearAccelerationYLabel);

        LinearAccelerationZLabel = new QLabel(layoutWidget);
        LinearAccelerationZLabel->setObjectName(QStringLiteral("LinearAccelerationZLabel"));
        LinearAccelerationZLabel->setFont(font4);

        LinearAccelerationDataBox->addWidget(LinearAccelerationZLabel);


        OrientationDataBoxes->addLayout(LinearAccelerationDataBox);

        OrientationFrame = new QFrame(OrientationTab);
        OrientationFrame->setObjectName(QStringLiteral("OrientationFrame"));
        OrientationFrame->setGeometry(QRect(9, 9, 771, 431));
        OrientationFrame->setFrameShape(QFrame::StyledPanel);
        OrientationFrame->setFrameShadow(QFrame::Raised);
        tabWidget->addTab(OrientationTab, QString());
        PoseTab = new QWidget();
        PoseTab->setObjectName(QStringLiteral("PoseTab"));
        PoseFrame = new QFrame(PoseTab);
        PoseFrame->setObjectName(QStringLiteral("PoseFrame"));
        PoseFrame->setGeometry(QRect(9, 9, 771, 561));
        PoseFrame->setFrameShape(QFrame::StyledPanel);
        PoseFrame->setFrameShadow(QFrame::Raised);
        tabWidget->addTab(PoseTab, QString());

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QWidget *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "EZ-RASSOR Dashboard", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(GeneralTab), QApplication::translate("MainWindow", "General", nullptr));
        CPUUsageLabel->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"right\"><span style=\" font-weight:600; font-style:italic;\">CPU Usage </span></p></body></html>", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(StatusTab), QApplication::translate("MainWindow", "Status", nullptr));
        RightCameraLabel->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"right\"><span style=\" font-weight:600; font-style:italic;\">Right Camera </span></p></body></html>", nullptr));
        LeftCameraLabel->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"right\"><span style=\" font-weight:600; font-style:italic;\">Left Camera </span></p></body></html>", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(CameraTab), QApplication::translate("MainWindow", "Cameras", nullptr));
        PointCloudLabel->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"right\"><span style=\" font-weight:600; font-style:italic;\">Point Cloud </span></p></body></html>", nullptr));
        DisparityMapLabel->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"right\"><span style=\" font-weight:600; font-style:italic;\">Disparity Map </span></p></body></html>", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(MapTab), QApplication::translate("MainWindow", "Map", nullptr));
        OrientationDataBoxLabel->setText(QApplication::translate("MainWindow", "Orientation", nullptr));
        OrientationXLabel->setText(QApplication::translate("MainWindow", " X: ", nullptr));
        OrientationYLabel->setText(QApplication::translate("MainWindow", " Y:", nullptr));
        OrientationZLabel->setText(QApplication::translate("MainWindow", " Z:", nullptr));
        AngularVelocityDataBoxLabel->setText(QApplication::translate("MainWindow", "Angular Velocity", nullptr));
        AngularVelocityXLabel->setText(QApplication::translate("MainWindow", " X:", nullptr));
        AngularVelocityYLabel->setText(QApplication::translate("MainWindow", " Y:", nullptr));
        AngularVelocityZLabel->setText(QApplication::translate("MainWindow", " Z:", nullptr));
        LinearAccelerationDataBoxLabel->setText(QApplication::translate("MainWindow", "Linear Acceleration", nullptr));
        LinearAccelerationXLabel->setText(QApplication::translate("MainWindow", " X:", nullptr));
        LinearAccelerationYLabel->setText(QApplication::translate("MainWindow", " Y:", nullptr));
        LinearAccelerationZLabel->setText(QApplication::translate("MainWindow", " Z:", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(OrientationTab), QApplication::translate("MainWindow", "Orientation", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(PoseTab), QApplication::translate("MainWindow", "Pose", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
