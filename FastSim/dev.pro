#-------------------------------------------------
#
# Project created by QtCreator 2013-06-20T18:18:14
#
#-------------------------------------------------

QT       += core gui opengl widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = dev
TEMPLATE = app


INCLUDEPATH += /usr/include/qwt
LIBS = -lqwt -lproj -lgsl -lgslcblas -lm -lpthread -lrt -lGL -lGLU -lglut -lcsv_parser -lfftw3


SOURCES += main.cpp \
    interfacing/JoystickController.cc \
    res/LatLongCalculations.cpp \
    res/utm.cc \
    sim/simulator_initialization.cpp \
    sim/simulator_utility.cpp \
    sim/simulator_core.cpp \
    sim/PathTracker.cc \
    sim/WorldModel.cpp \
    res/Comm.cpp \
    res/CommFunctions.cpp \
    res/Def4PlanReference.cpp \
    res/Def4PlanTrajectory.cpp \
    res/Def4PlanPerception.cpp \
    res/Def4Environment.cpp \
    vis/myglwidget.cpp \
    PlannerMonitor_VisualizationFunctions.cpp \
    PlannerMonitor_InteractiveControl.cpp \
    PlannerMonitor_Initialization.cpp \
    PlannerMonitor_Core.cpp \
    PlannerMonitor.cpp \
    PlannerMonitor_LoadScenario.cpp \

HEADERS  += \
    interfacing/JoystickController.h \
    sim/simulator.h \
    sim/PathTracker.h \
    sim/WorldModel.h \
    res/Comm.h \
    res/Def4Environment.h \
    res/Def4PlanReference.h \
    res/Def4PlanTrajectory.h \
    res/Def4PlanPerception.h \
    res/LatLongCalculations.h \
    res/CommFunctions.h \
    res/utm.h \
    vis/myglwidget.h \
    vis/VisObjectRectangle.h \
    vis/VisObjectCircle.h \
    PlannerMonitor.h \

FORMS    += \
    gui/PlannerMonitor.ui
