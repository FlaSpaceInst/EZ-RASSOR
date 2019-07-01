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
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QTabWidget *tabWidget;
    QWidget *GeneralTab;
    QWidget *CameraTab;
    QWidget *LogTab;
    QWidget *PoseTab;
    QWidget *OrientationTab;

    void setupUi(QWidget *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(800, 600);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(800, 600));
        MainWindow->setMaximumSize(QSize(9001, 9001));
        gridLayoutWidget = new QWidget(MainWindow);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(-1, -1, 801, 601));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        tabWidget = new QTabWidget(gridLayoutWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        GeneralTab = new QWidget();
        GeneralTab->setObjectName(QStringLiteral("GeneralTab"));
        tabWidget->addTab(GeneralTab, QString());
        CameraTab = new QWidget();
        CameraTab->setObjectName(QStringLiteral("CameraTab"));
        tabWidget->addTab(CameraTab, QString());
        LogTab = new QWidget();
        LogTab->setObjectName(QStringLiteral("LogTab"));
        tabWidget->addTab(LogTab, QString());
        PoseTab = new QWidget();
        PoseTab->setObjectName(QStringLiteral("PoseTab"));
        tabWidget->addTab(PoseTab, QString());
        OrientationTab = new QWidget();
        OrientationTab->setObjectName(QStringLiteral("OrientationTab"));
        tabWidget->addTab(OrientationTab, QString());

        gridLayout->addWidget(tabWidget, 0, 0, 1, 1);


        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QWidget *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "EZ-RASSOR Dashboard", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(GeneralTab), QApplication::translate("MainWindow", "General", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(CameraTab), QApplication::translate("MainWindow", "Cameras", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(LogTab), QApplication::translate("MainWindow", "Logs", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(PoseTab), QApplication::translate("MainWindow", "Pose", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(OrientationTab), QApplication::translate("MainWindow", "Orientation", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
