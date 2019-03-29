/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../ez_gui/include/ros_gui/main_window.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_RvizPlugin_t {
    QByteArrayData data[4];
    char stringdata0[38];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RvizPlugin_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RvizPlugin_t qt_meta_stringdata_RvizPlugin = {
    {
QT_MOC_LITERAL(0, 0, 10), // "RvizPlugin"
QT_MOC_LITERAL(1, 11, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 3) // "hhh"

    },
    "RvizPlugin\0on_pushButton_clicked\0\0hhh"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RvizPlugin[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x0a /* Public */,
       3,    0,   25,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RvizPlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RvizPlugin *_t = static_cast<RvizPlugin *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_pushButton_clicked(); break;
        case 1: _t->hhh(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject RvizPlugin::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_RvizPlugin.data,
      qt_meta_data_RvizPlugin,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *RvizPlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RvizPlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_RvizPlugin.stringdata0))
        return static_cast<void*>(const_cast< RvizPlugin*>(this));
    return QObject::qt_metacast(_clname);
}

int RvizPlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[20];
    char stringdata0[295];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 24), // "on_actionAbout_triggered"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 25), // "on_button_connect_clicked"
QT_MOC_LITERAL(4, 63, 5), // "check"
QT_MOC_LITERAL(5, 69, 40), // "on_checkbox_use_environment_s..."
QT_MOC_LITERAL(6, 110, 5), // "state"
QT_MOC_LITERAL(7, 116, 10), // "nodeLaunch"
QT_MOC_LITERAL(8, 127, 17), // "updateLoggingView"
QT_MOC_LITERAL(9, 145, 9), // "updateCPU"
QT_MOC_LITERAL(10, 155, 8), // "updateVM"
QT_MOC_LITERAL(11, 164, 8), // "updateSM"
QT_MOC_LITERAL(12, 173, 10), // "updateDisk"
QT_MOC_LITERAL(13, 184, 9), // "updateBat"
QT_MOC_LITERAL(14, 194, 17), // "updateFrontCamera"
QT_MOC_LITERAL(15, 212, 16), // "updateBackCamera"
QT_MOC_LITERAL(16, 229, 21), // "updateDisparityCamera"
QT_MOC_LITERAL(17, 251, 9), // "startRviz"
QT_MOC_LITERAL(18, 261, 17), // "populateLaunchers"
QT_MOC_LITERAL(19, 279, 15) // "updateImuLabels"

    },
    "MainWindow\0on_actionAbout_triggered\0"
    "\0on_button_connect_clicked\0check\0"
    "on_checkbox_use_environment_stateChanged\0"
    "state\0nodeLaunch\0updateLoggingView\0"
    "updateCPU\0updateVM\0updateSM\0updateDisk\0"
    "updateBat\0updateFrontCamera\0"
    "updateBackCamera\0updateDisparityCamera\0"
    "startRviz\0populateLaunchers\0updateImuLabels"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x0a /* Public */,
       3,    1,   95,    2, 0x0a /* Public */,
       5,    1,   98,    2, 0x0a /* Public */,
       7,    0,  101,    2, 0x0a /* Public */,
       8,    0,  102,    2, 0x0a /* Public */,
       9,    0,  103,    2, 0x0a /* Public */,
      10,    0,  104,    2, 0x0a /* Public */,
      11,    0,  105,    2, 0x0a /* Public */,
      12,    0,  106,    2, 0x0a /* Public */,
      13,    0,  107,    2, 0x0a /* Public */,
      14,    0,  108,    2, 0x0a /* Public */,
      15,    0,  109,    2, 0x0a /* Public */,
      16,    0,  110,    2, 0x0a /* Public */,
      17,    0,  111,    2, 0x0a /* Public */,
      18,    0,  112,    2, 0x0a /* Public */,
      19,    0,  113,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_button_connect_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_checkbox_use_environment_stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->nodeLaunch(); break;
        case 4: _t->updateLoggingView(); break;
        case 5: _t->updateCPU(); break;
        case 6: _t->updateVM(); break;
        case 7: _t->updateSM(); break;
        case 8: _t->updateDisk(); break;
        case 9: _t->updateBat(); break;
        case 10: _t->updateFrontCamera(); break;
        case 11: _t->updateBackCamera(); break;
        case 12: _t->updateDisparityCamera(); break;
        case 13: _t->startRviz(); break;
        case 14: _t->populateLaunchers(); break;
        case 15: _t->updateImuLabels(); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
