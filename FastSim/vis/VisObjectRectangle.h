#ifndef VisObjectRectangle_H
#define VisObjectRectangle_H

#include "res/Def4Environment.h"

class VisObjectRectangle
{
public:
    void attach(QwtPlot * plot) {
        curveEdgeFront_.attach(plot);
        curveEdgeBack_.attach(plot);
        curveEdgeLeft_.attach(plot);
        curveEdgeRight_.attach(plot);
    }

    void detach() {
        curveEdgeFront_.detach();
        curveEdgeBack_.detach();
        curveEdgeLeft_.detach();
        curveEdgeRight_.detach();
    }

    void updatePosDim(OBJECT_STATIC* obj) {
        easting_m_ = obj->getPose().x;
        northing_m_ = obj->getPose().y;
        heading_rad_ = obj->getPose().theta;
        length_m_ = obj->getLength();
        width_m_ = obj->getWidth();

        isGroundTruth_ = true;
        updateDrawings();
    }

    void updatePosDim(OBJECT_DYNAMIC_RECTANGLE obj) {
        easting_m_ = obj.getPose().x;
        northing_m_ = obj.getPose().y;
        heading_rad_ = obj.getPose().theta;
        length_m_ = obj.getLength();
        width_m_ = obj.getWidth();

        isGroundTruth_ = true;
        updateDrawings();
    }

    void updatePosDim(PH_OBJECT obj) {
        easting_m_ = obj.pose.x;
        northing_m_ = obj.pose.y;
        heading_rad_ = obj.pose.theta;
        length_m_ = obj.length_m;
        width_m_ = obj.width_m;

        isGroundTruth_ = false;
        updateDrawings();
    }


private:
    void updateDrawings()
    {
        std::pair<double,double> fc, bc;
        std::pair<double,double> fl, fr, bl, br;

        fc.first = easting_m_ + length_m_ / 2 * cos(heading_rad_);
        fc.second = northing_m_ + length_m_ / 2 * sin(heading_rad_);

        bc.first = easting_m_ - length_m_ / 2 * cos(heading_rad_);
        bc.second = northing_m_ - length_m_ / 2 * sin(heading_rad_);

        fl.first = fc.first - width_m_ / 2 * sin(heading_rad_);
        fl.second = fc.second + width_m_ / 2 * cos(heading_rad_);

        fr.first = fc.first + width_m_ / 2 * sin(heading_rad_);
        fr.second = fc.second - width_m_ / 2 * cos(heading_rad_);

        bl.first = bc.first - width_m_ / 2 * sin(heading_rad_);
        bl.second = bc.second + width_m_ / 2 * cos(heading_rad_);

        br.first = bc.first + width_m_ / 2 * sin(heading_rad_);
        br.second = bc.second - width_m_ / 2 * cos(heading_rad_);


        QVector<double> tmpX, tmpY;
        tmpX.push_back(fl.first); tmpX.push_back(fr.first);
        tmpY.push_back(fl.second); tmpY.push_back(fr.second);
        curveEdgeFront_.setSamples(tmpX, tmpY);
        if( isGroundTruth_ ) { curveEdgeFront_.setPen(QPen(QColor(238,232,170), 3)); }
        else { curveEdgeFront_.setPen(QPen(QColor(0,0,255), 3)); }
        curveEdgeFront_.setStyle(QwtPlotCurve::Lines);


        tmpX.clear(); tmpY.clear();
        tmpX.push_back(bl.first); tmpX.push_back(br.first);
        tmpY.push_back(bl.second); tmpY.push_back(br.second);
        curveEdgeBack_.setSamples(tmpX, tmpY);
        if( isGroundTruth_ ) { curveEdgeBack_.setPen(QPen(QColor(238,232,170), 3)); }
        else { curveEdgeBack_.setPen(QPen(QColor(0,0,255), 3)); }
        curveEdgeBack_.setStyle(QwtPlotCurve::Lines);


        tmpX.clear(); tmpY.clear();
        tmpX.push_back(bl.first); tmpX.push_back(fl.first);
        tmpY.push_back(bl.second); tmpY.push_back(fl.second);
        curveEdgeLeft_.setSamples(tmpX, tmpY);
        if( isGroundTruth_ ) { curveEdgeLeft_.setPen(QPen(QColor(238,232,170), 3)); }
        else { curveEdgeLeft_.setPen(QPen(QColor(0,0,255), 3)); }
        curveEdgeLeft_.setStyle(QwtPlotCurve::Lines);


        tmpX.clear(); tmpY.clear();
        tmpX.push_back(br.first); tmpX.push_back(fr.first);
        tmpY.push_back(br.second); tmpY.push_back(fr.second);
        curveEdgeRight_.setSamples(tmpX, tmpY);
        if( isGroundTruth_ ) { curveEdgeRight_.setPen(QPen(QColor(238,232,170), 3)); }
        else { curveEdgeRight_.setPen(QPen(QColor(0,0,255), 3)); }
        curveEdgeRight_.setStyle(QwtPlotCurve::Lines);
    }

    double easting_m_;
    double northing_m_;
    double heading_rad_;
    double length_m_;
    double width_m_;

    bool isGroundTruth_;

    QwtPlotCurve curveEdgeFront_;
    QwtPlotCurve curveEdgeBack_;
    QwtPlotCurve curveEdgeLeft_;
    QwtPlotCurve curveEdgeRight_;

};

#endif // PLOTCAR_H
