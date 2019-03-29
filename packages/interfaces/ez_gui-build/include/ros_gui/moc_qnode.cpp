/****************************************************************************
** Meta object code from reading C++ file 'qnode.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../ez_gui/include/ros_gui/qnode.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_QNode_t {
    QByteArrayData data[14];
    char stringdata0[170];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QNode_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QNode_t qt_meta_stringdata_QNode = {
    {
QT_MOC_LITERAL(0, 0, 5), // "QNode"
QT_MOC_LITERAL(1, 6, 14), // "loggingUpdated"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 10), // "cpuUpdated"
QT_MOC_LITERAL(4, 33, 9), // "vmUpdated"
QT_MOC_LITERAL(5, 43, 9), // "smUpdated"
QT_MOC_LITERAL(6, 53, 11), // "diskUpdated"
QT_MOC_LITERAL(7, 65, 14), // "batteryUpdated"
QT_MOC_LITERAL(8, 80, 15), // "frontCamUpdated"
QT_MOC_LITERAL(9, 96, 14), // "backCamUpdated"
QT_MOC_LITERAL(10, 111, 16), // "disparityUpdated"
QT_MOC_LITERAL(11, 128, 16), // "imuLabelsUpdated"
QT_MOC_LITERAL(12, 145, 12), // "startingRviz"
QT_MOC_LITERAL(13, 158, 11) // "rosShutdown"

    },
    "QNode\0loggingUpdated\0\0cpuUpdated\0"
    "vmUpdated\0smUpdated\0diskUpdated\0"
    "batteryUpdated\0frontCamUpdated\0"
    "backCamUpdated\0disparityUpdated\0"
    "imuLabelsUpdated\0startingRviz\0rosShutdown"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QNode[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      12,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x06 /* Public */,
       3,    0,   75,    2, 0x06 /* Public */,
       4,    0,   76,    2, 0x06 /* Public */,
       5,    0,   77,    2, 0x06 /* Public */,
       6,    0,   78,    2, 0x06 /* Public */,
       7,    0,   79,    2, 0x06 /* Public */,
       8,    0,   80,    2, 0x06 /* Public */,
       9,    0,   81,    2, 0x06 /* Public */,
      10,    0,   82,    2, 0x06 /* Public */,
      11,    0,   83,    2, 0x06 /* Public */,
      12,    0,   84,    2, 0x06 /* Public */,
      13,    0,   85,    2, 0x06 /* Public */,

 // signals: parameters
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

void QNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QNode *_t = static_cast<QNode *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->loggingUpdated(); break;
        case 1: _t->cpuUpdated(); break;
        case 2: _t->vmUpdated(); break;
        case 3: _t->smUpdated(); break;
        case 4: _t->diskUpdated(); break;
        case 5: _t->batteryUpdated(); break;
        case 6: _t->frontCamUpdated(); break;
        case 7: _t->backCamUpdated(); break;
        case 8: _t->disparityUpdated(); break;
        case 9: _t->imuLabelsUpdated(); break;
        case 10: _t->startingRviz(); break;
        case 11: _t->rosShutdown(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::loggingUpdated)) {
                *result = 0;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::cpuUpdated)) {
                *result = 1;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::vmUpdated)) {
                *result = 2;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::smUpdated)) {
                *result = 3;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::diskUpdated)) {
                *result = 4;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::batteryUpdated)) {
                *result = 5;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::frontCamUpdated)) {
                *result = 6;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::backCamUpdated)) {
                *result = 7;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::disparityUpdated)) {
                *result = 8;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::imuLabelsUpdated)) {
                *result = 9;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::startingRviz)) {
                *result = 10;
            }
        }
        {
            typedef void (QNode::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNode::rosShutdown)) {
                *result = 11;
            }
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject QNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_QNode.data,
      qt_meta_data_QNode,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_QNode.stringdata0))
        return static_cast<void*>(const_cast< QNode*>(this));
    return QThread::qt_metacast(_clname);
}

int QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void QNode::loggingUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void QNode::cpuUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void QNode::vmUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void QNode::smUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void QNode::diskUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}

// SIGNAL 5
void QNode::batteryUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 5, Q_NULLPTR);
}

// SIGNAL 6
void QNode::frontCamUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 6, Q_NULLPTR);
}

// SIGNAL 7
void QNode::backCamUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 7, Q_NULLPTR);
}

// SIGNAL 8
void QNode::disparityUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 8, Q_NULLPTR);
}

// SIGNAL 9
void QNode::imuLabelsUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 9, Q_NULLPTR);
}

// SIGNAL 10
void QNode::startingRviz()
{
    QMetaObject::activate(this, &staticMetaObject, 10, Q_NULLPTR);
}

// SIGNAL 11
void QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 11, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
