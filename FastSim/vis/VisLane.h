#ifndef DRAWING_LANE_H
#define DRAWING_LANE_H

#include "qwt_plot.h"
#include "qwt_plot_curve.h"

#include "res/Comm.h"
#include "res/Def4PlanReference.h"

class DrawingLane
{
public:
    DrawingLane()
    {
        laneCL_.reset();
    }


    void attach(QwtPlot * plot)
    {
        curveCL_.attach(plot);
        curveLL_.attach(plot);
        curveRL_.attach(plot);
    }

    void detach()
    {
        curveCL_.detach();
        curveLL_.detach();
        curveRL_.detach();
    }

    void updateLane(LanePiece laneCL) {
        if(laneCL_.numOfPoints != 0) { return; } // If already set one, used to safe redrawing time

        laneCL_ = laneCL;
        updateDrawings();
    }


private:
    void updateDrawings()
    {
        QVector<double> tmpX, tmpY;

        // draw centerline
        tmpX.clear(); tmpY.clear();
        for(int ii=0; ii<laneCL_.numOfPoints; ii++) {
            tmpX.push_back( laneCL_.x[ii] );
            tmpY.push_back( laneCL_.y[ii] );
        }
        curveCL_.setSamples(tmpX, tmpY);
        curveCL_.setPen(QPen(QColor(200,200,200), 1));
        curveCL_.setStyle(QwtPlotCurve::Dots);


        // draw leftline
        tmpX.clear(); tmpY.clear();
        for(int ii=0; ii<laneCL_.numOfPoints; ii++) {
            tmpX.push_back( laneCL_.x[ii] - laneCL_.laneWidth / 2 * sin(laneCL_.theta[ii]) );
            tmpY.push_back( laneCL_.y[ii] + laneCL_.laneWidth / 2 * cos(laneCL_.theta[ii]) );
        }
        curveLL_.setSamples(tmpX, tmpY);
        curveLL_.setPen(QPen(QColor(200,200,200), 3));
        curveLL_.setStyle(QwtPlotCurve::Lines);


        // draw rightline
        tmpX.clear(); tmpY.clear();
        for(int ii=0; ii<laneCL_.numOfPoints; ii++) {
            tmpX.push_back( laneCL_.x[ii] + laneCL_.laneWidth / 2 * sin(laneCL_.theta[ii]) );
            tmpY.push_back( laneCL_.y[ii] - laneCL_.laneWidth / 2 * cos(laneCL_.theta[ii]) );
        }
        curveRL_.setSamples(tmpX, tmpY);
        curveRL_.setPen(QPen(QColor(200,200,200), 3));
        curveRL_.setStyle(QwtPlotCurve::Lines);
    }


    LanePiece laneCL_;

    QwtPlotCurve curveCL_;
    QwtPlotCurve curveLL_;
    QwtPlotCurve curveRL_;

};

#endif // DRAWING_LANE_H
