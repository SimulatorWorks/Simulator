/**
* @file         PlannerMonitor.h
* @author       Tianyu Gu
* @date         06/10/2013
*/

#ifndef PLANNERMONITOR_H
#define PLANNERMONITOR_H

#include <QMainWindow>
#include <QFile>
#include <QFileDialog>

#include "qwt_symbol.h"
#include "qwt_plot.h"
#include "qwt_plot_curve.h"
#include "qwt_plot_magnifier.h"
#include "qwt_plot_panner.h"

#include "interfacing/JoystickController.h"

#include "sim/simulator.h"

#include "vis/VisObjectCircle.h"
#include "vis/VisObjectRectangle.h"

#define PLAN_CYCLE_TIME_MS  100
#define SIM_CYCLE_TIME_MS  100

namespace Ui {
class PlannerMonitor;
}

class PlannerMonitor : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit PlannerMonitor(QWidget *parent = 0);
    ~PlannerMonitor();
    
private:
    void corePlanningSequence();
    void coreSimulatingSequence( bool isSimIsolated );

private slots:
    // Timer Slots
    void timerUpdateDisplayTimeout();

    void timerQtSimulatorTimeout();

    void timerGetJoystickInputTimeout();

    void on_btnWorldZoomIn_clicked();

    void on_btnWorldZoomOut_clicked();

    void on_btnWorldLeft_clicked();

    void on_btnRight_clicked();

    void on_btnWorldUp_clicked();

    void on_btnWorldDown_clicked();

    void on_btnUpdateFromSimVs_clicked();

    void on_btnSetSimVs_clicked();

    void on_btnSimSwitch_clicked();

    void on_btnToggleDisplaySim_RoadStructure_clicked();

    void on_btnToggleDisplaySim_VehiclePose_clicked();

    void on_btnToggleDisplaySim_GroundTruthObjects_clicked();

    void on_btnJoystickOverride_clicked();

    void on_btnLoadScenarioFromFile_clicked();

private:
    Ui::PlannerMonitor *ui;


private:
    JoystickController* js_;
    bool jsConnected_;

    // Qt simulating,
    simulator* sim_;

    // For visualization
    void setWorldPerspective();
        QwtPlotMagnifier* canvasMagnifier_;
        QwtPlotPanner*    canvasPanner_;
        double canvasZoomScale_;
        int canvasVerticalOffset_;
        int canvasHorizontalOffset_;

        // "draw": a drawing of an object on canvas
        // "curv": a drawing of a curve on canvas
        // "figu": a plot on figures in widget tabs
        // "mark": a mark on figures in widget tabs


    void displaySim_VehState(bool isDisplay);

    void displaySim_RoadStructure(bool isDisplay);

    void displaySim_Objects(bool isDisplay);

    void displayJS_Input();
        QwtPlotCurve curveSteerWheelHorizontalBar_;


private:
    QTimer *timerUpdateDisplay_;
    QTimer *timerQtSimulator_;
    QTimer *timerGetJoystickInput_;


    // true => run qt simulator, retrieve sensing data from qt simulator
    bool isSimulatorEnabled_;
    bool isJoystickOverride_;

    // display controls
    bool isDisplaySim_RoadStructure_;
    bool isDisplaySim_VehiclePose_;
    bool isDisplaySim_GroundTruthObjects_;

    bool trigger0_;
    bool trigger1_;
    bool trigger2_;

    inline double  timeval_as_double(struct timeval* tv) {
        double sec = (double)tv->tv_sec + ((double)tv->tv_usec / 1000000.0);
        return sec;
    }

    inline double  get_sysclock_sec(void) {
        struct timeval tv;
        int ret;
        ret = gettimeofday(&tv, NULL);
        if (ret)    {
            perror("gettimeofday");
            return -1.0;
        }
        else {
            return timeval_as_double(&tv);
        }
    }
};

#endif // PLANNERMONITOR_H
