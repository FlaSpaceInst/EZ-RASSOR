/********************************************************************************
** Form generated from reading UI file 'welcome_window.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WELCOME_WINDOW_H
#define UI_WELCOME_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_WelcomeWindow
{
public:
    QLabel *ROSMasterURLLabel;
    QLineEdit *ROSMasterURLLineEdit;
    QPushButton *ConnectButton;

    void setupUi(QWidget *WelcomeWindow)
    {
        if (WelcomeWindow->objectName().isEmpty())
            WelcomeWindow->setObjectName(QStringLiteral("WelcomeWindow"));
        WelcomeWindow->resize(447, 143);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(WelcomeWindow->sizePolicy().hasHeightForWidth());
        WelcomeWindow->setSizePolicy(sizePolicy);
        WelcomeWindow->setMinimumSize(QSize(447, 143));
        WelcomeWindow->setMaximumSize(QSize(447, 143));
        ROSMasterURLLabel = new QLabel(WelcomeWindow);
        ROSMasterURLLabel->setObjectName(QStringLiteral("ROSMasterURLLabel"));
        ROSMasterURLLabel->setGeometry(QRect(30, 30, 121, 17));
        ROSMasterURLLineEdit = new QLineEdit(WelcomeWindow);
        ROSMasterURLLineEdit->setObjectName(QStringLiteral("ROSMasterURLLineEdit"));
        ROSMasterURLLineEdit->setGeometry(QRect(30, 50, 391, 25));
        ROSMasterURLLineEdit->setAutoFillBackground(false);
        ROSMasterURLLineEdit->setMaxLength(100);
        ROSMasterURLLineEdit->setAlignment(Qt::AlignCenter);
        ConnectButton = new QPushButton(WelcomeWindow);
        ConnectButton->setObjectName(QStringLiteral("ConnectButton"));
        ConnectButton->setGeometry(QRect(30, 90, 391, 25));

        retranslateUi(WelcomeWindow);

        QMetaObject::connectSlotsByName(WelcomeWindow);
    } // setupUi

    void retranslateUi(QWidget *WelcomeWindow)
    {
        WelcomeWindow->setWindowTitle(QApplication::translate("WelcomeWindow", "EZ-RASSOR Dashboard", nullptr));
        ROSMasterURLLabel->setText(QApplication::translate("WelcomeWindow", "ROS Master URL", nullptr));
        ROSMasterURLLineEdit->setText(QApplication::translate("WelcomeWindow", "192.168.1.1:13001", nullptr));
        ConnectButton->setText(QApplication::translate("WelcomeWindow", "Connect", nullptr));
    } // retranslateUi

};

namespace Ui {
    class WelcomeWindow: public Ui_WelcomeWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WELCOME_WINDOW_H
