#-------------------------------------------------
#
# Project created by QtCreator 2016-01-29T14:42:47
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = QuadroTuneGUI
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    reg_map.h

FORMS    += mainwindow.ui
