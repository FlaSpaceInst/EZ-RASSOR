#include <QApplication>
#include "main_window.h"
#include "about_window.h"
#include "topic_translator.h"

int main(int argumentCount, char** argumentVector) {
    QApplication application(argumentCount, argumentVector);

    MainWindow mainWindow;
    mainWindow.show();
    AboutWindow aboutWindow;
    aboutWindow.show();
    TopicTranslator topicTranslator(
        "dashboard",
        "cpu_usage",
        "memory_usage",
        "battery_usage",
        100
    );

    //topicTranslator.connectToMaster("whatthefuck");
    //topicTranslator.start();

    application.connect(
        &topicTranslator,
        SIGNAL(processorDataReceived(int)),
        mainWindow.CPUUsageBar,
        SLOT(setValue(int))
    );

    application.connect(
        &mainWindow,
        SIGNAL(connectionRequested(const std::string&)),
        &topicTranslator,
        SLOT(connectToMaster(const std::string&))
    );

    return application.exec();
}
