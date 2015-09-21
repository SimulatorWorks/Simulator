/**
* @file         PlannerMonitor_Initialization.cpp
* @author       Tianyu Gu
* @date         06/10/2013
*/

#include "PlannerMonitor.h"
#include "ui_PlannerMonitor.h"

PlannerMonitor::PlannerMonitor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PlannerMonitor)
{
    ui->setupUi(this);

    ui->plot2Dworld->setAutoReplot(true);
    ui->plot2Dworld->enableAxis(0, true);
    ui->plot2Dworld->enableAxis(2, true);
    ui->plot2Dworld->setAxisTitle(0, "Northing");
    ui->plot2Dworld->setAxisTitle(2, "Easting");

    ui->plotHistoryCurvature->setAutoReplot(true);
    ui->plotHistoryCurvature->enableAxis(0, true);
    ui->plotHistoryCurvature->enableAxis(2, true);
    ui->plotHistoryCurvature->setAxisTitle(0, "Curvature");
    ui->plotHistoryCurvature->setAxisTitle(2, "Time");
    ui->plotHistoryCurvature->setAxisScale(0, -0.2*100, 0.2*100);
    ui->plotHistoryCurvature->setAxisScale(2, -10, 0);

    ui->plotHistorySpeed->setAutoReplot(true);
    ui->plotHistorySpeed->enableAxis(0, true);
    ui->plotHistorySpeed->enableAxis(2, true);
    ui->plotHistorySpeed->setAxisTitle(0, "Speed");
    ui->plotHistorySpeed->setAxisTitle(2, "Time");
    ui->plotHistorySpeed->setAxisScale(0, 0, 30);
    ui->plotHistorySpeed->setAxisScale(2, -10, 0);

    ui->plotSteeringWheel->setAutoReplot();
    ui->plotSteeringWheel->enableAxis(0, false);
    ui->plotSteeringWheel->enableAxis(2, false);



    canvasMagnifier_ = new QwtPlotMagnifier(ui->plot2Dworld->canvas());
    canvasMagnifier_->setMouseButton(Qt::RightButton);
    canvasPanner_ = new QwtPlotPanner(ui->plot2Dworld->canvas());

    canvasZoomScale_ = 1.0;
    canvasVerticalOffset_ = 0;
    canvasHorizontalOffset_ = 0;

    // Initialize server communicator
//    comm_ = new ComServer();

    // Initialize joystick controller for steering wheel
    jsConnected_ = false;
    js_ = new JoystickController();
    if(js_->getJoystickFd() < 0) {
        qDebug("Unable to connect to Joystick\n");
    }
    else {
        qDebug("Connected to Joystick\n");
        jsConnected_ = true;
    }


    // Initilize qt simulator
    sim_ = new simulator();


    // Link Controls
    isSimulatorEnabled_ = false;
    isJoystickOverride_ = false;

    // Display Controls
    isDisplaySim_RoadStructure_ = true;
    isDisplaySim_VehiclePose_ = true;
    isDisplaySim_GroundTruthObjects_ = true;


    // Timer for update displays
    timerUpdateDisplay_ = new QTimer(this);
    timerUpdateDisplay_->setInterval(20);
    connect(timerUpdateDisplay_, SIGNAL(timeout()), this, SLOT(timerUpdateDisplayTimeout()));
    timerUpdateDisplay_->start();


    // Timer for QT simulator
    timerQtSimulator_ = new QTimer(this);
    timerQtSimulator_->setInterval(SIM_CYCLE_TIME_MS);
    connect(timerQtSimulator_, SIGNAL(timeout()), this, SLOT(timerQtSimulatorTimeout()));
    timerQtSimulator_->start();

    // Timer for interfacing with joystick
    timerGetJoystickInput_ = new QTimer(this);
    timerGetJoystickInput_->setInterval(1);
    connect(timerGetJoystickInput_, SIGNAL(timeout()), this, SLOT(timerGetJoystickInputTimeout()));
    timerGetJoystickInput_->start();
}

PlannerMonitor::~PlannerMonitor()
{
    delete ui;
    delete sim_;
}
