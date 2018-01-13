#-------------------------------------------------
#
# Project created by QtCreator 2017-09-06T22:27:57
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets charts

TARGET = MapVierwer
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

DEFINES += ZMQ_STATIC

#ZMQ_PATH = "E:/imaginarium/libzmq_4.2.2_visual_2015/"
ZMQ_PATH = "H:/imaginarium/libzmq_4.2.2_visual_2015/"
#ZMQ_PATH = "I:/imaginarium/libzmq_4.2.2_visual_2017/"
VC_VERSION = 140
#VC_VERSION = 141

INCLUDEPATH += $$ZMQ_PATH/include


win32:CONFIG(release, debug|release): LIBS  += -L$$ZMQ_PATH/lib/ -llibzmq-v$$VC_VERSION-mt-s-4_2_2
else:win32:CONFIG(debug, debug|release): LIBS  += -L$$ZMQ_PATH/lib/ -llibzmq-v$$VC_VERSION-mt-sgd-4_2_2
#else:unix: LIBS  += $$OUT_PWD/../../../projects/mylib/libmylib.a

win32: LIBS += -lWs2_32 -lAdvapi32 -lIphlpapi

RC_FILE = myrc.rc

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    ZmqSubscriber.cpp \
    SensorLocItem.cpp \
    MyGraphView.cpp

HEADERS += \
        mainwindow.h \
    ZmqSubscriber.h \
    SensorLocItem.h \
    MyGraphView.h

FORMS += \
        mainwindow.ui
