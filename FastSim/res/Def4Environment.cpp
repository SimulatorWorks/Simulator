/**
---------------------------------------------------------------------------
Copyright  (c) 2010-2013 Carnegie Mellon University,
All rights reserved.

This source code was developed in part with support from sponsors including
General Motors, the National Science Foundation and the US Department of
Transportation.

Use of this software is strictly governed by agreements between Carnegie
Mellon University and various sponsors.

Users of this software must also be fully aware and agree that Carnegie
Mellon does not guarantee the correct functioning of this software in any
system. Carnegie Mellon or any of its affiliates will not be liable for
any damage and/or penalties resulting from the use of this software.
Any user of this software takes complete responsibility for the use of any
software or design.
---------------------------------------------------------------------------
* @file         Def4Simulation.cpp
* @author       Tianyu Gu
* @date         06/13/2013
*/

#include "Def4Environment.h"

void LanePiece::reset() {
#ifdef FLAG_IN_TRUCS
    waypoints.clear();
#else
    waypointsLL.clear();
    waypointsNE.clear();
#endif
    s.clear();
    x.clear();
    y.clear();
    theta.clear();
    k.clear();
    dk.clear();

    numOfPoints = 0;

    isValid = false;
}

bool LanePiece::generateReference( vector<pair<double, double> > waypointsLL,
                                   vector<pair<double, double> > waypointsNE,
                                   double laneWidth_m,
                                   double speedLimit_mps )
{
    this->reset();

    this->waypointsLL = waypointsLL;
    this->waypointsNE = waypointsNE;

    // Interpolate at every interpGap_m
    vector<double>  wps, wpx, wpy;

    for(vector<pair<double, double> >::size_type ii=0; ii<waypointsNE.size(); ii++) {
        wpx.push_back( waypointsNE.at(ii).first );
        wpy.push_back( waypointsNE.at(ii).second );
    }
    wps.push_back( 0.0 );
    for(vector<pair<double, double> >::size_type ii=1; ii<waypointsNE.size(); ii++) {
        double distIncrement = hypot( wpx.at(ii) - wpx.at(ii-1),
                                      wpy.at(ii) - wpy.at(ii-1) );
        wps.push_back( wps.back() + distIncrement );
    }

    int numOfStation = wps.size();

    if(numOfStation == 0) { return false; }

    double s[numOfStation];
    double x[numOfStation];
    double y[numOfStation];
    for (int i = 0; i < numOfStation; i++) {
        s[i] = wps.at(i);
        x[i] = wpx.at(i);
        y[i] = wpy.at(i);
    }

    gsl_interp_accel *acc;
    gsl_spline *Xspline;
    gsl_spline *Yspline;

    if(numOfStation >= 4) {
        acc         = gsl_interp_accel_alloc ();
        Xspline     = gsl_spline_alloc (gsl_interp_cspline, numOfStation);
        Yspline     = gsl_spline_alloc (gsl_interp_cspline, numOfStation);

        gsl_spline_init (Xspline, s, x, numOfStation);
        gsl_spline_init (Yspline, s, y, numOfStation);

        for (double ss = s[0]; ss <= s[numOfStation-1] ; ss += CENTERLINE_RESOLUTION) {
            this->s.push_back( ss );
            this->x.push_back( gsl_spline_eval (Xspline, ss, acc) );
            this->y.push_back( gsl_spline_eval (Yspline, ss, acc) );
            this->theta.push_back( 0.0 );
            this->k.push_back( 0.0 );
            this->dk.push_back( 0.0 );
        }
        this->numOfPoints = this->s.size();

        this->type = NORMAL_LANE;
        this->laneWidth = laneWidth_m;
        this->speedLimit = speedLimit_mps;

        this->interpretHeadingCurvature();
    }
    else if (numOfStation >= 2) {
        acc         = gsl_interp_accel_alloc ();
        Xspline     = gsl_spline_alloc (gsl_interp_linear, numOfStation);
        Yspline     = gsl_spline_alloc (gsl_interp_linear, numOfStation);

        gsl_spline_init (Xspline, s, x, numOfStation);
        gsl_spline_init (Yspline, s, y, numOfStation);

        for (double ss = 0; ss <= s[numOfStation-1] ; ss += CENTERLINE_RESOLUTION) {
            this->s.push_back( ss );
            this->x.push_back( gsl_spline_eval (Xspline, ss, acc) );
            this->y.push_back( gsl_spline_eval (Yspline, ss, acc) );
            this->theta.push_back( 0.0 );
            this->k.push_back( 0.0 );
        }
        this->numOfPoints = this->s.size();

        this->type = NORMAL_LANE;
        this->laneWidth = laneWidth_m;
        this->speedLimit = speedLimit_mps;

        this->interpretHeadingCurvature();
    }
    else if (numOfStation == 1) {
        this->s.push_back( 0.0 );
        this->x.push_back( x[0] );
        this->y.push_back( y[0] );
        this->theta.push_back( 0.0 );
        this->k.push_back( 0.0 );

        this->numOfPoints = this->s.size();

        this->type = NORMAL_LANE;
        this->laneWidth = laneWidth_m;
        this->speedLimit = speedLimit_mps;
    }

    this->isValid = true;

    gsl_spline_free (Xspline);
    gsl_spline_free (Yspline);
    gsl_interp_accel_free (acc);

    return true;
}

int LanePiece::getNearestIndexBySpatialState(StateSpatial spatial)
{
    double minDist = GSL_POSINF;
    int minId = -1;
    for(int i=0; i < numOfPoints; i++) {
        double dist = hypot(spatial.x - x[i], spatial.y - y[i]);
        if(dist < minDist) {
            minDist = dist;
            minId = i;
        }
    }

    return minId;
}

int LanePiece::getNearestIndexByCoordinates(double x, double y)
{
    double minDist = GSL_POSINF;
    int minId = -1;
    for(int i=0; i < numOfPoints; i++) {
        double dist = hypot(x - this->x[i], y - this->y[i]);
        if(dist < minDist) {
            minDist = dist;
            minId = i;
        }
    }

    return minId;
}

double LanePiece::getAbsDistanceByCoordinates(double x, double y)
{
    double minDist = GSL_POSINF;
    for(int i=0; i<numOfPoints; i++) {
        double dist = hypot( this->x[i]-x, this->y[i]-y );
        if( dist < minDist ) {
            minDist = dist;
        }
    }

    return minDist;
}

double LanePiece::getSignedDistanceByCoordinates(double x, double y)
{
    float minDist1 = GSL_POSINF;
    int s_Id1 = -1;
    float minDist2 = GSL_POSINF;
    int s_Id2 = -1;


    for(vector<double>::size_type j = 0; j < this->x.size(); j++ ) {
        double dist = hypot(x - this->x[j], y - this->y[j]);
        if(minDist1 > dist) {
            minDist2 = minDist1;
            minDist1 = dist;
            s_Id2 = s_Id1;
            s_Id1 = j;
        }
        else if(minDist2 > dist) {
            minDist2 = dist;
            s_Id2 = j;
        }
    }

    double angle1 = atan2(y - this->y[s_Id1], x - this->x[s_Id1]);
    double angle2;

    if(s_Id1 >= numOfPoints - 1) {
        //If the nearest is the last point in ci_Route ...
        angle2 = atan2(this->y[s_Id1] - this->y[s_Id1-1],
                this->x[s_Id1] - this->x[s_Id1-1]);
    }
    else {
        angle2 = atan2(this->y[s_Id1+1] - this->y[s_Id1],
                this->x[s_Id1+1] - this->x[s_Id1]);
    }

    // Calculate the real dist
    double a = hypot(this->y[s_Id1] - y,
                     this->x[s_Id1] - x);
    double b = hypot(this->y[s_Id2] - y,
                     this->x[s_Id2] - x);
    double c = hypot(this->y[s_Id2] - this->y[s_Id1],
                     this->x[s_Id2] - this->x[s_Id1]);
    double p = (a+b+c)/2;
    double area = sqrt(p*(p-a)*(p-b)*(p-c));

    double minDist = 2*area/c;

    double dist;
    if (angle2 >= 0) {
        if (angle1 >= 0) { // if both in "top half" of atan2 plot
            if((angle1 - angle2) >= 0) {
                //left side
                dist = minDist;
            }
            else {
                // right side
                dist = -minDist;
            }
        }
        if (angle1 < 0) { // angle2 in top half, angle1 in bottom half
            if(angle1 >= angle2- M_PI) {
                // right side
                dist = -minDist;
            }
            else {
                //left side
                dist = minDist;
            }
        }
    }
    else {
        if (angle1 >= 0) { // angle2 in bottom half, angle1 in top half
            if (angle1 <= (angle2 + M_PI)) {
                //left side
                dist = minDist;
            }
            else {
                // right side
                dist = -minDist;
            }
        }
        if (angle1 < 0) { // angle1 and angle2 both in bottom half
            if (angle1 >= angle2) {
                //left side
                dist = minDist;
            }
            else {
                // right side
                dist = -minDist;
            }
        }
    }

    return dist;
}


bool LanePiece::interpretHeadingCurvature()
{
    // Recalculate theta, k and dk
    if(this->numOfPoints < NUM_FIT_FRONT + NUM_FIT_BACK + 1) {
        int i = 0;
        for( i = 0; i < this->numOfPoints; i++) {
            if(i == this->numOfPoints - 1) {
                this->theta[i] = this->theta[i-1];
                this->k[i] = 0;
                break;
            }
            this->theta[i] = atan2(this->y[i+1] - this->y[i], this->x[i+1] - this->x[i]);
            this->k[i] = 0;
        }
    }
    else {
        for(int i = 0; i < this->numOfPoints; i++) {
            int id0 = (i-NUM_FIT_FRONT >= 0) ? (i-NUM_FIT_FRONT):(0);
            int id1 = (i+NUM_FIT_BACK < this->numOfPoints) ? (i+NUM_FIT_BACK):(this->numOfPoints-1);
            double si, xi, yi, /*ei,*/ chisq;

            gsl_matrix *S, *cov;
            gsl_vector *x, *y, *w, *cx, *cy;

            int n = id1-id0+1;
            S = gsl_matrix_alloc (n, 3);
            x = gsl_vector_alloc (n);
            y = gsl_vector_alloc (n);
            w = gsl_vector_alloc (n);

            cx = gsl_vector_alloc (3);
            cy = gsl_vector_alloc (3);
            cov = gsl_matrix_alloc (3, 3);

            for (int j = 0; j < n; j++)
            {
                si = this->s[id0+j];
                xi = this->x[id0+j];
                yi = this->y[id0+j];

                gsl_matrix_set (S, j, 0, 1.0);
                gsl_matrix_set (S, j, 1, si);
                gsl_matrix_set (S, j, 2, si*si);

                gsl_vector_set (x, j, xi);
                gsl_vector_set (y, j, yi);
                gsl_vector_set (w, j, 1.0);
            }

            gsl_multifit_linear_workspace * work = gsl_multifit_linear_alloc (n, 3);
            gsl_multifit_wlinear (S, w, x, cx, cov, &chisq, work);
            gsl_multifit_linear_free (work);

            work = gsl_multifit_linear_alloc (n, 3);
            gsl_multifit_wlinear (S, w, y, cy, cov, &chisq, work);
            gsl_multifit_linear_free (work);

#define Cx(i) (gsl_vector_get(cx,(i)))
#define Cy(i) (gsl_vector_get(cy,(i)))

            double s_ = this->s[i];
            //        double x_ = Cx(2) * s_ * s_ + Cx(1) * s_ + Cx(0);
            double xd_ = 2 * Cx(2) * s_ + Cx(1);
            double xdd_ = 2 * Cx(2);
            //        double y_ = Cy(2) * s_ * s_ + Cy(1) * s_ + Cy(0);
            double yd_ = 2 * Cy(2) * s_ + Cy(1);
            double ydd_ = 2 * Cy(2);

            this->theta[i] = atan2(yd_, xd_);
            this->k[i] = (xd_ * ydd_ - yd_ * xdd_)
                    / ( sqrt( (xd_*xd_ + yd_*yd_)*(xd_*xd_ + yd_*yd_)*(xd_*xd_ + yd_*yd_) ) );

            gsl_matrix_free (S);
            gsl_vector_free (x);
            gsl_vector_free (y);
            gsl_vector_free (w);

            gsl_vector_free (cx);
            gsl_vector_free (cy);
            gsl_matrix_free (cov);
        }
    }

    return true;
}


void LanePiece::attach(QwtPlot * plot) {
    curveCL_.attach(plot);
    curveLL_.attach(plot);
    curveRL_.attach(plot);
}

void LanePiece::detach() {
    curveCL_.detach();
    curveLL_.detach();
    curveRL_.detach();
}

void LanePiece::updateDrawings()
{
    QVector<double> tmpX, tmpY;

    // draw centerline
    tmpX.clear(); tmpY.clear();
    for(int ii=0; ii<numOfPoints; ii++) {
        tmpX.push_back( x[ii] );
        tmpY.push_back( y[ii] );
    }
    curveCL_.setSamples(tmpX, tmpY);
    curveCL_.setPen(QPen(QColor(200,200,200), 1));
    curveCL_.setStyle(QwtPlotCurve::Dots);


    // draw leftline
    tmpX.clear(); tmpY.clear();
    for(int ii=0; ii<numOfPoints; ii++) {
        tmpX.push_back( x[ii] - laneWidth / 2 * sin(theta[ii]) );
        tmpY.push_back( y[ii] + laneWidth / 2 * cos(theta[ii]) );
    }
    curveLL_.setSamples(tmpX, tmpY);
    curveLL_.setPen(QPen(QColor(200,200,200), 3));
    curveLL_.setStyle(QwtPlotCurve::Lines);


    // draw rightline
    tmpX.clear(); tmpY.clear();
    for(int ii=0; ii<numOfPoints; ii++) {
        tmpX.push_back( x[ii] + laneWidth / 2 * sin(theta[ii]) );
        tmpY.push_back( y[ii] - laneWidth / 2 * cos(theta[ii]) );
    }
    curveRL_.setSamples(tmpX, tmpY);
    curveRL_.setPen(QPen(QColor(200,200,200), 3));
    curveRL_.setStyle(QwtPlotCurve::Lines);
}


//int RoadGroundTruth::getIndexOfNearestLaneInOneGoalSegment(int goalSegmentId, double startX, double startY)
//{
//    int numOfLanes = this->goalSegments[goalSegmentId].lanePieces.size();

//    double minDist[numOfLanes];
//    int minIdx[numOfLanes];
//    for(int i=0; i<numOfLanes; i++)
//    {
//        minDist[i] = GSL_POSINF;
//        minIdx[i] = -1;
//    }

//    for(int i=0; i<numOfLanes; i++)
//    {
//        for(int j=0; j < this->goalSegments[goalSegmentId].lanePieces[i].numOfPoints; j++)
//        {
//            double dist = hypot(startX - this->goalSegments[goalSegmentId].lanePieces[i].x[j],
//                                startY - this->goalSegments[goalSegmentId].lanePieces[i].y[j]);
//            if(dist < minDist[i])
//            {
//                minDist[i] = dist;
//                minIdx[i] = j;
//            }
//        }
//    }

//    // find the index of start lane in this->goalSegments[goalSegmentId].laneIds
//    double minDistToLane = GSL_POSINF;
//    int startLaneIdx = -1;
//    for(int i=0; i < numOfLanes; i++)
//    {
//        if(minDist[i] < minDistToLane)
//        {
//            minDistToLane = minDist[i];
//            startLaneIdx = i;
//        }
//    }

//    return startLaneIdx;
//}


OBJECT::OBJECT() {
    name.clear();
    pose.reset();
    speed_mps = 0.0;
    acc_mpss = 0.0;
    isSimTriggered = NULL;
}

void OBJECT_STATIC::attach(QwtPlot * plot) {
    curveEdgeFront_.attach(plot);
    curveEdgeBack_.attach(plot);
    curveEdgeLeft_.attach(plot);
    curveEdgeRight_.attach(plot);
}

void OBJECT_STATIC::detach() {
    curveEdgeFront_.detach();
    curveEdgeBack_.detach();
    curveEdgeLeft_.detach();
    curveEdgeRight_.detach();
}

void OBJECT_STATIC::updateDrawings()
{
    std::pair<double,double> fc, bc;
    std::pair<double,double> fl, fr, bl, br;

    fc.first = pose.x + length_m / 2 * cos(pose.theta);
    fc.second = pose.y + length_m / 2 * sin(pose.theta);

    bc.first = pose.x - length_m / 2 * cos(pose.theta);
    bc.second = pose.y - length_m / 2 * sin(pose.theta);

    fl.first = fc.first - width_m / 2 * sin(pose.theta);
    fl.second = fc.second + width_m / 2 * cos(pose.theta);

    fr.first = fc.first + width_m / 2 * sin(pose.theta);
    fr.second = fc.second - width_m / 2 * cos(pose.theta);

    bl.first = bc.first - width_m / 2 * sin(pose.theta);
    bl.second = bc.second + width_m / 2 * cos(pose.theta);

    br.first = bc.first + width_m / 2 * sin(pose.theta);
    br.second = bc.second - width_m / 2 * cos(pose.theta);


    QVector<double> tmpX, tmpY;
    tmpX.push_back(fl.first); tmpX.push_back(fr.first);
    tmpY.push_back(fl.second); tmpY.push_back(fr.second);
    curveEdgeFront_.setSamples(tmpX, tmpY);
    curveEdgeFront_.setPen(QPen(QColor(238,232,170), 3));
    curveEdgeFront_.setStyle(QwtPlotCurve::Lines);


    tmpX.clear(); tmpY.clear();
    tmpX.push_back(bl.first); tmpX.push_back(br.first);
    tmpY.push_back(bl.second); tmpY.push_back(br.second);
    curveEdgeBack_.setSamples(tmpX, tmpY);
    curveEdgeBack_.setPen(QPen(QColor(238,232,170), 3));
    curveEdgeBack_.setStyle(QwtPlotCurve::Lines);


    tmpX.clear(); tmpY.clear();
    tmpX.push_back(bl.first); tmpX.push_back(fl.first);
    tmpY.push_back(bl.second); tmpY.push_back(fl.second);
    curveEdgeLeft_.setSamples(tmpX, tmpY);
    curveEdgeLeft_.setPen(QPen(QColor(238,232,170), 3));
    curveEdgeLeft_.setStyle(QwtPlotCurve::Lines);


    tmpX.clear(); tmpY.clear();
    tmpX.push_back(br.first); tmpX.push_back(fr.first);
    tmpY.push_back(br.second); tmpY.push_back(fr.second);
    curveEdgeRight_.setSamples(tmpX, tmpY);
    curveEdgeRight_.setPen(QPen(QColor(238,232,170), 3));
    curveEdgeRight_.setStyle(QwtPlotCurve::Lines);
}

std::pair<double, double> OBJECT_DYNAMIC_CIRCLE::getFuturePosition(double time) {
    double movedDist = speed_mps * time;

    std::pair<double, double> pos;

    pos.first = pose.x + movedDist * cos(pose.theta);
    pos.second = pose.y + movedDist * sin(pose.theta);

    return pos;
}

void OBJECT_DYNAMIC_CIRCLE::stepSim_linear(double time) {

    if( isSimTriggered == NULL ||
            (isSimTriggered != NULL && *isSimTriggered == true ) ) {
        double movedDist = speed_mps * time;
        pose.x = pose.x + movedDist * cos(pose.theta);
        pose.y = pose.y + movedDist * sin(pose.theta);
    }
}

void OBJECT_DYNAMIC_CIRCLE::attach(QwtPlot * plot) {
    curveEdge_.attach(plot);
}

void OBJECT_DYNAMIC_CIRCLE::detach() {
    curveEdge_.detach();
}

void OBJECT_DYNAMIC_CIRCLE::updateDrawings() {
    QVector<double> tmpX, tmpY;

    for(float i=0; i<=2 * M_PI + 0.1 ; i+=0.1) {
        tmpX.push_back(pose.x + radius_m * sin(i));
        tmpY.push_back(pose.y + radius_m * cos(i));
    }

    curveEdge_.setSamples(tmpX, tmpY);
    curveEdge_.setPen(QPen(QColor(238,232,170), 3));
    curveEdge_.setStyle(QwtPlotCurve::Lines);
}


OBJECT_DYNAMIC_RECTANGLE::OBJECT_DYNAMIC_RECTANGLE() {
    hasPath = false;
    path.clear();
    isSimTriggered = NULL;
    isSwitchTrackingLaneTriggered = NULL;

    laneTracking = NULL;
    laneToTrack  = NULL;
}

void OBJECT_DYNAMIC_RECTANGLE::attach(QwtPlot * plot) {
    curveEdgeFront_.attach(plot);
    curveEdgeBack_.attach(plot);
    curveEdgeLeft_.attach(plot);
    curveEdgeRight_.attach(plot);
}

void OBJECT_DYNAMIC_RECTANGLE::detach() {
    curveEdgeFront_.detach();
    curveEdgeBack_.detach();
    curveEdgeLeft_.detach();
    curveEdgeRight_.detach();
}

void OBJECT_DYNAMIC_RECTANGLE::updateDrawings()
{
    std::pair<double,double> fc, bc;
    std::pair<double,double> fl, fr, bl, br;

    fc.first = pose.x + length_m / 2 * cos(pose.theta);
    fc.second = pose.y + length_m / 2 * sin(pose.theta);

    bc.first = pose.x - length_m / 2 * cos(pose.theta);
    bc.second = pose.y - length_m / 2 * sin(pose.theta);

    fl.first = fc.first - width_m / 2 * sin(pose.theta);
    fl.second = fc.second + width_m / 2 * cos(pose.theta);

    fr.first = fc.first + width_m / 2 * sin(pose.theta);
    fr.second = fc.second - width_m / 2 * cos(pose.theta);

    bl.first = bc.first - width_m / 2 * sin(pose.theta);
    bl.second = bc.second + width_m / 2 * cos(pose.theta);

    br.first = bc.first + width_m / 2 * sin(pose.theta);
    br.second = bc.second - width_m / 2 * cos(pose.theta);


    QVector<double> tmpX, tmpY;
    tmpX.push_back(fl.first); tmpX.push_back(fr.first);
    tmpY.push_back(fl.second); tmpY.push_back(fr.second);
    curveEdgeFront_.setSamples(tmpX, tmpY);
    curveEdgeFront_.setPen(QPen(QColor(238,232,170), 3));
    curveEdgeFront_.setStyle(QwtPlotCurve::Lines);


    tmpX.clear(); tmpY.clear();
    tmpX.push_back(bl.first); tmpX.push_back(br.first);
    tmpY.push_back(bl.second); tmpY.push_back(br.second);
    curveEdgeBack_.setSamples(tmpX, tmpY);
    curveEdgeBack_.setPen(QPen(QColor(238,232,170), 3));
    curveEdgeBack_.setStyle(QwtPlotCurve::Lines);


    tmpX.clear(); tmpY.clear();
    tmpX.push_back(bl.first); tmpX.push_back(fl.first);
    tmpY.push_back(bl.second); tmpY.push_back(fl.second);
    curveEdgeLeft_.setSamples(tmpX, tmpY);
    curveEdgeLeft_.setPen(QPen(QColor(238,232,170), 3));
    curveEdgeLeft_.setStyle(QwtPlotCurve::Lines);


    tmpX.clear(); tmpY.clear();
    tmpX.push_back(br.first); tmpX.push_back(fr.first);
    tmpY.push_back(br.second); tmpY.push_back(fr.second);
    curveEdgeRight_.setSamples(tmpX, tmpY);
    curveEdgeRight_.setPen(QPen(QColor(238,232,170), 3));
    curveEdgeRight_.setStyle(QwtPlotCurve::Lines);
}

StateSpatial OBJECT_DYNAMIC_RECTANGLE::predictPosHeading_Linear(double time)
{
    double movedDist = speed_mps * time;

    StateSpatial posture;

    posture.x = pose.x + movedDist * cos(pose.theta);
    posture.y = pose.y + movedDist * sin(pose.theta);
    posture.theta = pose.theta;
    posture.k = 0;
    posture.dk = 0;

    return posture;
}

void OBJECT_DYNAMIC_RECTANGLE::checkSwitchLaneTrigger()
{
    if( ( isSwitchTrackingLaneTriggered != NULL && *isSwitchTrackingLaneTriggered == true ) ) {
        laneTracking = laneToTrack;
    }
}

void OBJECT_DYNAMIC_RECTANGLE::stepSim_linear(double time)
{
    if( isSimTriggered == NULL ||
            ( isSimTriggered != NULL && *isSimTriggered == true ) ) {

        double movedDist = speed_mps * time;
        pose.x = pose.x + movedDist * cos(pose.theta);
        pose.y = pose.y + movedDist * sin(pose.theta);

        hasPath = false;
    }
}


void OBJECT_DYNAMIC_RECTANGLE::stepSim_tracking(double time)
{
    if( isSimTriggered == NULL ||
            (isSimTriggered != NULL && *isSimTriggered == true ) ) {

        if ( laneTracking!= NULL && laneTracking->numOfPoints == 0) return;

        // Generate just one path and one speed profile
        StateSpatial s0;
        s0.x = pose.x;
        s0.y = pose.y;
        s0.theta = pose.theta;
        s0.k = 0.0;

        int startId = laneTracking->getNearestIndexBySpatialState( s0 );
        int lookaheadId = fmin(laneTracking->numOfPoints - 1, startId + 50);

        StateSpatial sf;
        sf.x = laneTracking->x[lookaheadId];
        sf.y = laneTracking->y[lookaheadId];
        sf.theta = laneTracking->theta[lookaheadId];
        sf.k = 0.0;

        PathParameterArc pathPar;
        pathPar.findPathParameter(s0, sf, 3);
        path = pathPar.plotPath(100);
        hasPath = true;

        double a = acc_mpss;
        double v0 = speed_mps;
        double vf = fmin(fmax(v0 + a * time, 0.0), laneTracking->speedLimit);

        double movedDist = (v0 + vf) / 2 * time;
        int projId = path.getProjIdxByS(movedDist);

        pose.x = path.x[projId];
        pose.y = path.y[projId];
        pose.theta = path.theta[projId];
        speed_mps = vf;
        acc_mpss = a;
    }
}

void OBJECT_DYNAMIC_RECTANGLE::stepSim_tracking_dk(double time, vector<OBJECT_DYNAMIC_RECTANGLE*> objs, HostVehicle* hv)
{
    if( isSimTriggered == NULL ||
            (isSimTriggered != NULL && *isSimTriggered == true ) ) {

        if ( laneTracking!= NULL && laneTracking->numOfPoints == 0) return;

        double dkGapTime_s = 2.0;
        double maxDist2ConsiderDK_m = 100.0;

        // Generate just one path and one speed profile
        int startId = laneTracking->getNearestIndexByCoordinates( pose.x, pose.y );
        int lookaheadId = fmin(laneTracking->numOfPoints - 1, startId + 50);

        StateSpatial s0;
        s0.x = pose.x;
        s0.y = pose.y;
        s0.theta = pose.theta;
        s0.k = 0.0;

        StateSpatial sf;
        sf.x = laneTracking->x[lookaheadId];
        sf.y = laneTracking->y[lookaheadId];
        sf.theta = laneTracking->theta[lookaheadId];
        sf.k = 0.0;

        PathParameterArc pathPar;
        pathPar.findPathParameter(s0, sf, 3);
        path = pathPar.plotPath(100);
        hasPath = true;

        // retrieve a list of objects from that is associated with a lane, find the one to distance keeping for
        bool isDK2Obj = false;
        OBJECT_DYNAMIC_RECTANGLE* objDK = NULL;
        double dist2ObjDK = GSL_POSINF;
        for(int i=0; i<objs.size(); i++) {
            OBJECT_DYNAMIC_RECTANGLE* obj = objs[i];
            if( obj->name != this->name && obj->laneTracking == this->laneTracking ) {

                int objId = laneTracking->getNearestIndexByCoordinates( obj->getPose().x, obj->getPose().y );
                double dist_m = laneTracking->s[objId] - laneTracking->s[startId];

                if( dist_m > 0 && dist_m <= maxDist2ConsiderDK_m ) {
                    if( dist_m < dist2ObjDK ) {
                        isDK2Obj = true;
                        dist2ObjDK = dist_m;
                        objDK = obj;
                    }
                }
            }
        }

        // associate a lane for the vehicle state
        bool isDK2HV = false;
        double dist2HvDK = GSL_POSINF;
        double latDistToLaneTracking_m = laneTracking->getAbsDistanceByCoordinates(hv->getVehicleState().spatial.x, hv->getVehicleState().spatial.y);
        if( latDistToLaneTracking_m <= laneTracking->laneWidth / 2 ) {
            int hvId = laneTracking->getNearestIndexByCoordinates( hv->getVehicleState().spatial.x, hv->getVehicleState().spatial.y );
            double dist_m = laneTracking->s[hvId] - laneTracking->s[startId];

            if( dist_m > 0 && dist_m <= maxDist2ConsiderDK_m ) {
                if( dist_m < dist2ObjDK ) {
                    isDK2HV = true;
                    isDK2Obj = false;
                    dist2HvDK = dist_m;
                }
            }
        }

        // calculate acceleration
        double a = acc_mpss;

        if( isDK2Obj ) {
            double dist2ObjDK_target = speed_mps * dkGapTime_s;

            double epsilon_dot = objDK->speed_mps - this->speed_mps;
            double delta = dist2ObjDK_target - dist2ObjDK;
            double lamda = 1.0;

            a = - 1 / dkGapTime_s * ( epsilon_dot + lamda*delta );
        }
        else if( isDK2HV ) {
            double dist2HvDK_target = speed_mps * dkGapTime_s;

            double epsilon_dot = hv->getVehicleState().temporal.v - this->speed_mps;
            double delta = dist2HvDK_target - dist2HvDK;
            double lamda = 1.0;

            a = - 1 / dkGapTime_s * ( epsilon_dot + lamda*delta );
        }

        double v0 = speed_mps;
        double vf = fmin(fmax(v0 + a * time, 0.0), laneTracking->speedLimit);

        double movedDist = (v0 + vf) / 2 * time;
        int projId = path.getProjIdxByS(movedDist);

        pose.x= path.x[projId];
        pose.y = path.y[projId];
        pose.theta = path.theta[projId];
        speed_mps = vf;
        acc_mpss = a;
    }
}



// unit construction
HostVehicle::HostVehicle()
{
    vs_.reset();
}

// destruction, nothing to see here
HostVehicle::~HostVehicle()
{
}

void HostVehicle::forceSetVehicleState(double easting, double northing, double heading, double curv, double speed, double acc)
{
    vs_.spatial.x = easting;
    vs_.spatial.y = northing;
    vs_.spatial.theta = heading;
    vs_.spatial.k = curv;

    vs_.temporal.v = speed;
    vs_.temporal.a = acc;
}


void HostVehicle::setCommand(double speed, double curvature)
{
    m_speed_cmd_ = speed;
    m_curvature_cmd_ = curvature;


    //In this model they are the same;
    m_speed_ = m_speed_cmd_;
    m_curvature_ = m_curvature_cmd_;
}


void HostVehicle::stepSim(double dt)
{
    // Get last state
    double x = vs_.spatial.x;
    double y = vs_.spatial.y;
    double theta = vs_.spatial.theta;

    // Cal increment
    double arc = m_speed_ * dt;
    if (arc == 0) { return; }


    if( fabs(m_curvature_) <= 1e-6 ) {
        x = x + arc * cos(theta);
        y = y + arc * sin(theta);
    }
    else {
        double radius = - 1 / m_curvature_;
        double sgn = 1;
        if(radius<0) { sgn = -1; }

        radius = fabs(radius);
        double cx = x + radius * cos(theta - sgn * M_PI / 2);
        double cy = y + radius * sin(theta - sgn * M_PI / 2);

        double ctheta = theta + sgn * M_PI / 2;
        double phi = arc / (sgn * radius);

        x = cx + radius * cos(ctheta - phi);
        y = cy + radius * sin(ctheta - phi);
        theta = (ctheta - phi) - sgn * M_PI / 2;

        double count = ceil(theta/(2*M_PI) - 0.5);
        theta = theta - count * 2 * M_PI;
    }

    // Update state
    vs_.spatial.x = x;
    vs_.spatial.y = y;
    vs_.spatial.theta = theta;
    vs_.spatial.k = m_curvature_;
    vs_.temporal.v = m_speed_;
    vs_.temporal.a = 0.0;

    isAutonomous_ = true;
}


void HostVehicle::attach(QwtPlot * plot)
{
    curveEdgeFront_.attach(plot);
    curveEdgeBack_.attach(plot);
    curveEdgeLeft_.attach(plot);
    curveEdgeRight_.attach(plot);

    curveAxleFront_.attach(plot);
    curveAxleRear_.attach(plot);

    curveWheelFL_.attach(plot);
    curveWheelFR_.attach(plot);
    curveWheelBL_.attach(plot);
    curveWheelBR_.attach(plot);
}

void HostVehicle::detach()
{
    curveEdgeFront_.detach();
    curveEdgeBack_.detach();
    curveEdgeLeft_.detach();
    curveEdgeRight_.detach();

    curveAxleFront_.detach();
    curveAxleRear_.detach();

    curveWheelFL_.detach();
    curveWheelFR_.detach();
    curveWheelBL_.detach();
    curveWheelBR_.detach();
}

void HostVehicle::updateDrawings()
{
    double theta = vs_.spatial.theta;

    std::pair<double,double> cg;
    cg.first = vs_.spatial.x + VEH_WHEELBASE_B * cos(theta);
    cg.second = vs_.spatial.y + VEH_WHEELBASE_B * sin(theta);

    std::pair<double,double> fc, bc;
    std::pair<double,double> fl, fr, bl, br;

    fc.first = cg.first + VEH_LF * cos(theta);
    fc.second = cg.second + VEH_LF * sin(theta);

    bc.first = cg.first - VEH_LB * cos(theta);
    bc.second = cg.second - VEH_LB * sin(theta);

    fl.first = fc.first - VEH_WIDTH / 2 * sin(theta);
    fl.second = fc.second + VEH_WIDTH / 2 * cos(theta);

    fr.first = fc.first + VEH_WIDTH / 2 * sin(theta);
    fr.second = fc.second - VEH_WIDTH / 2 * cos(theta);

    bl.first = bc.first - VEH_WIDTH / 2 * sin(theta);
    bl.second = bc.second + VEH_WIDTH / 2 * cos(theta);

    br.first = bc.first + VEH_WIDTH / 2 * sin(theta);
    br.second = bc.second - VEH_WIDTH / 2 * cos(theta);


    QVector<double> tmpX, tmpY;
    tmpX.push_back(fl.first); tmpX.push_back(fr.first);
    tmpY.push_back(fl.second); tmpY.push_back(fr.second);
    curveEdgeFront_.setSamples(tmpX, tmpY);

    tmpX.clear(); tmpY.clear();
    tmpX.push_back(bl.first); tmpX.push_back(br.first);
    tmpY.push_back(bl.second); tmpY.push_back(br.second);
    curveEdgeBack_.setSamples(tmpX, tmpY);

    tmpX.clear(); tmpY.clear();
    tmpX.push_back(bl.first); tmpX.push_back(fl.first);
    tmpY.push_back(bl.second); tmpY.push_back(fl.second);
    curveEdgeLeft_.setSamples(tmpX, tmpY);

    tmpX.clear(); tmpY.clear();
    tmpX.push_back(br.first); tmpX.push_back(fr.first);
    tmpY.push_back(br.second); tmpY.push_back(fr.second);
    curveEdgeRight_.setSamples(tmpX, tmpY);



    std::pair<double,double> frontAxleCenter, backAxleCenter;
    std::pair<double,double> frontAxleLeft, frontAxleRight, backAxleLeft, backAxleRight;
    frontAxleCenter.first = cg.first + VEH_WHEELBASE_F * cos(theta);
    frontAxleCenter.second = cg.second + VEH_WHEELBASE_F * sin(theta);

    backAxleCenter.first = cg.first - VEH_WHEELBASE_B * cos(theta);
    backAxleCenter.second = cg.second - VEH_WHEELBASE_B * sin(theta);

    frontAxleLeft.first = frontAxleCenter.first - VEH_WIDTH_WHEELS / 2 * sin(theta);
    frontAxleLeft.second = frontAxleCenter.second + VEH_WIDTH_WHEELS / 2 * cos(theta);

    frontAxleRight.first = frontAxleCenter.first + VEH_WIDTH_WHEELS / 2 * sin(theta);
    frontAxleRight.second = frontAxleCenter.second - VEH_WIDTH_WHEELS / 2 * cos(theta);

    backAxleLeft.first = backAxleCenter.first - VEH_WIDTH_WHEELS / 2 * sin(theta);
    backAxleLeft.second = backAxleCenter.second + VEH_WIDTH_WHEELS / 2 * cos(theta);

    backAxleRight.first = backAxleCenter.first + VEH_WIDTH_WHEELS / 2 * sin(theta);
    backAxleRight.second = backAxleCenter.second - VEH_WIDTH_WHEELS / 2 * cos(theta);

    tmpX.clear(); tmpY.clear();
    tmpX.push_back(frontAxleLeft.first); tmpX.push_back(frontAxleRight.first);
    tmpY.push_back(frontAxleLeft.second); tmpY.push_back(frontAxleRight.second);
    curveAxleFront_.setSamples(tmpX, tmpY);

    tmpX.clear(); tmpY.clear();
    tmpX.push_back(backAxleLeft.first); tmpX.push_back(backAxleRight.first);
    tmpY.push_back(backAxleLeft.second); tmpY.push_back(backAxleRight.second);
    curveAxleRear_.setSamples(tmpX, tmpY);


    double absSteerAngle = theta + atan(vs_.spatial.k * VEH_WHEELBASE);

    std::pair<double,double> frontLeftWheelF, frontLeftWheelB, frontRightWheelF, frontRightWheelB;
    frontLeftWheelF.first = frontAxleLeft.first + VEH_WHEEL_RADIUS * cos(absSteerAngle);
    frontLeftWheelF.second = frontAxleLeft.second + VEH_WHEEL_RADIUS * sin(absSteerAngle);

    frontLeftWheelB.first = frontAxleLeft.first - VEH_WHEEL_RADIUS * cos(absSteerAngle);
    frontLeftWheelB.second = frontAxleLeft.second - VEH_WHEEL_RADIUS * sin(absSteerAngle);

    frontRightWheelF.first = frontAxleRight.first + VEH_WHEEL_RADIUS * cos(absSteerAngle);
    frontRightWheelF.second = frontAxleRight.second + VEH_WHEEL_RADIUS * sin(absSteerAngle);

    frontRightWheelB.first = frontAxleRight.first - VEH_WHEEL_RADIUS * cos(absSteerAngle);
    frontRightWheelB.second = frontAxleRight.second - VEH_WHEEL_RADIUS * sin(absSteerAngle);


    tmpX.clear(); tmpY.clear();
    tmpX.push_back(frontLeftWheelF.first); tmpX.push_back(frontLeftWheelB.first);
    tmpY.push_back(frontLeftWheelF.second); tmpY.push_back(frontLeftWheelB.second);
    curveWheelFL_.setSamples(tmpX, tmpY);

    tmpX.clear(); tmpY.clear();
    tmpX.push_back(frontRightWheelF.first); tmpX.push_back(frontRightWheelB.first);
    tmpY.push_back(frontRightWheelF.second); tmpY.push_back(frontRightWheelB.second);
    curveWheelFR_.setSamples(tmpX, tmpY);



    std::pair<double,double> backLeftWheelF, backLeftWheelB, backRightWheelF, backRightWheelB;
    backLeftWheelF.first = backAxleLeft.first + VEH_WHEEL_RADIUS * cos(theta);
    backLeftWheelF.second = backAxleLeft.second + VEH_WHEEL_RADIUS * sin(theta);

    backLeftWheelB.first = backAxleLeft.first - VEH_WHEEL_RADIUS * cos(theta);
    backLeftWheelB.second = backAxleLeft.second - VEH_WHEEL_RADIUS * sin(theta);

    backRightWheelF.first = backAxleRight.first + VEH_WHEEL_RADIUS * cos(theta);
    backRightWheelF.second = backAxleRight.second + VEH_WHEEL_RADIUS * sin(theta);

    backRightWheelB.first = backAxleRight.first - VEH_WHEEL_RADIUS * cos(theta);
    backRightWheelB.second = backAxleRight.second - VEH_WHEEL_RADIUS * sin(theta);

    tmpX.clear(); tmpY.clear();
    tmpX.push_back(backLeftWheelF.first); tmpX.push_back(backLeftWheelB.first);
    tmpY.push_back(backLeftWheelF.second); tmpY.push_back(backLeftWheelB.second);
    curveWheelBL_.setSamples(tmpX, tmpY);

    tmpX.clear(); tmpY.clear();
    tmpX.push_back(backRightWheelF.first); tmpX.push_back(backRightWheelB.first);
    tmpY.push_back(backRightWheelF.second); tmpY.push_back(backRightWheelB.second);
    curveWheelBR_.setSamples(tmpX, tmpY);
}
