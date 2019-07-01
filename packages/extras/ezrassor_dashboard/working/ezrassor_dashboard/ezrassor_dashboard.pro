QT += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
TEMPLATE = app
TARGET = ezrassor_dashboard
DEFINES += QT_DEPRECATED_WARNINGS
SOURCES += \
        ezrassor_dashboard.cpp \
        welcome_window.cpp \
        main_window.cpp
HEADERS += \
        welcome_window.h \
        main_window.h
FORMS += \
    welcome_window.ui \
    main_window.ui
