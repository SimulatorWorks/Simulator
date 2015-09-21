#ifndef VisObjectCircle_H
#define VisObjectCircle_H

#include "qwt_plot.h"
#include "qwt_plot_curve.h"

#include "res/Comm.h"
#include "res/Def4Environment.h"
#include "res/Def4PlanPerception.h"

class VisObjectCircle
{
public:
    void attach(QwtPlot * plot) {
        curveEdge_.attach(plot);
    }

    void detach() {
        curveEdge_.detach();
    }

    void updatePosDim(OBJECT_DYNAMIC_CIRCLE obj) {
        easting_m_ = obj.getPose().x;
        northing_m_ = obj.getPose().y;
        heading_rad_ = obj.getPose().theta;
        radius_m_ = obj.getRadius();

        isGroundTruth_ = true;
        updateDrawings();
    }

    void updatePosDim(PH_OBJECT obj) {
        easting_m_ = obj.pose.x;
        northing_m_ = obj.pose.y;
        heading_rad_ = obj.pose.theta;
        radius_m_ = fmax(obj.width_m, obj.length_m);

        isGroundTruth_ = false;
        updateDrawings();
    }


private:
    void updateDrawings() {
        QVector<double> tmpX, tmpY;

        for(float i=0; i<=2 * M_PI + 0.1 ; i+=0.1) {
            tmpX.push_back(easting_m_ + radius_m_ * sin(i));
            tmpY.push_back(northing_m_ + radius_m_ * cos(i));
        }

        curveEdge_.setSamples(tmpX, tmpY);

        if(isGroundTruth_) {
            curveEdge_.setPen(QPen(QColor(238,232,170), 3));
        }
        else {
            curveEdge_.setPen(QPen(QColor(0,255,0), 3));
        }

        curveEdge_.setStyle(QwtPlotCurve::Lines);
    }

    double easting_m_;
    double northing_m_;
    double heading_rad_;
    double radius_m_;

    bool isGroundTruth_;

    QwtPlotCurve curveEdge_;
};

#endif // PLOTCAR_H
