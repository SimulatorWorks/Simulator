#include "Def4PlanReference.h"

ReferenceRaw::ReferenceRaw()
{
    reset();
}

void ReferenceRaw::reset()
{
    this->rB.clear();
    this->lB.clear();

    this->x.clear();
    this->y.clear();
    this->theta.clear();
    this->k.clear();
    this->s.clear();

    this->v_L.clear();

    this->segmentId.clear();

    this->type.clear();

    this->numOfPoints = 0;
}

Reference::Reference()
{
    this->reset();
}

void Reference::reset()
{
    this->rB.clear();
    this->lB.clear();

    this->s.clear();
    this->x.clear();
    this->y.clear();
    this->theta.clear();
    this->k.clear();

    this->vLim.clear();
    this->vMax.clear();
    this->v.clear();
    this->tEst.clear();

    this->type.clear();

    this->segmentId.clear();
    this->optFlag.clear();

    this->numOfPoints = 0;

    this->blocked =  false;
}

void Reference::push(double rB,
                     double lB,
                     double s,
                     double x,
                     double y,
                     double theta,
                     double k,
                     double vLim,
                     double vMax,
                     double v,
                     double tEst,
                     ReferenceType type,
                     int segmentId,
                     int optFlag )
{
    this->rB.push_back(rB);
    this->lB.push_back(lB);

    this->s.push_back(s);
    this->x.push_back(x);
    this->y.push_back(y);
    this->theta.push_back(theta);
    this->k.push_back(k);

    this->vLim.push_back(vLim);
    this->vMax.push_back(vMax);
    this->v.push_back(v);
    this->tEst.push_back(tEst);

    this->type.push_back(type);
    this->segmentId.push_back(segmentId);

    this->optFlag.push_back(optFlag);

    this->numOfPoints++;
}


//int Reference::getNearestIndexBySpatialState(StateSpatial spatial)
//{
//    double minDist = GSL_POSINF;
//    int minId = -1;
//    for(int i=0; i < numOfPoints; i++) {
//        double dist = hypot(spatial.x - this->x[i], spatial.y - this->y[i]);
//        if(dist < minDist) {
//            minDist = dist;
//            minId = i;
//        }
//    }

//    return minId;
//}

int Reference::getNearestIndexByTime(double time)
{
    double minDT = GSL_POSINF;
    int minId = -1;

    for(int ii=0; ii < numOfPoints; ii++) {
        double dt = fabs(time - this->tEst[ii]);
        if(dt < minDT) {
            minDT = dt;
            minId = ii;
        }
    }

    return minId;
}

int Reference::getNearestIndexByStation( double station )
{
    double minDS = GSL_POSINF;
    int minId = -1;

    for(int ii=0; ii < this->numOfPoints; ii++) {
        double ds = fabs(station - this->s[ii]);
        if(ds < minDS) {
            minDS = ds;
            minId = ii;
        }
    }

    return minId;
}

double Reference::getStationByCoordinates( double x, double y )
{
    double minDist = GSL_POSINF;
    int minId = -1;
    for(int i=0; i < this->numOfPoints; i++) {
        double dist = hypot(x - this->x[i], y - this->y[i]);
        if(dist < minDist) {
            minDist = dist;
            minId = i;
        }
    }

    return this->s[minId];
}

double Reference::getStationByCoordinates( double x, double y, int idx0, int idxf )
{
    double minDist = GSL_POSINF;
    int minId = idx0;
    for(int i=idx0; i <= idxf; i++) {
        double dist = hypot(x - this->x[i], y - this->y[i]);
        if(dist < minDist) {
            minDist = dist;
            minId = i;
        }
    }

    return this->s[minId];
}

double Reference::getSignedDistanceByCoordinates( double x, double y )
{
    float minDist1 = GSL_POSINF;
    int s_Id1 = -1;
    float minDist2 = GSL_POSINF;
    int s_Id2 = -1;

    for(unsigned int j = 0; j < this->x.size(); j++ ) {
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

    if(s_Id1 >= this->numOfPoints - 1) {
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

int Reference::getNearestIndexByCoordinates( double x, double y )
{
    double minDist = GSL_POSINF;
    int minId = -1;
    for(int i=0; i < this->numOfPoints; i++) {
        double dist = hypot(x - this->x[i], y - this->y[i]);
        if(dist < minDist) {
            minDist = dist;
            minId = i;
        }
    }

    return minId;
}

double Reference::getAbsDistanceByCoordinates( double x, double y )
{
    double minDist = GSL_POSINF;
    for(int ii=0; ii<numOfPoints; ii++) {
        double dist = hypot(x - this->x[ii], y - this->y[ii]);
        if(dist < minDist) {
            minDist = dist;
        }
    }

    return minDist;
}

bool Reference::recalculateStationHeadingCurvature()
{
    if (this->numOfPoints <= 1) { return false; }

    // Recalculate s
    for(int i = 0; i < this->numOfPoints ; i++) {
        if(i == 0) { this->s[0] = 0; }
        else {
            double ds = hypot(this->x[i] - this->x[i-1], this->y[i] - this->y[i-1]);
            this->s[i] = this->s[i-1] + ds;
        }
    }

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

            for (int j = 0; j < n; j++) {
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

            gsl_matrix_free(S);
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

//bool Reference::adjustMaximumVelocity(double currV)
//{
//    if(this->numOfPoints > 0)
//    {
//        std::vector<double>& x = this->x;
//        std::vector<double>& y = this->y;
//        std::vector<double>& k = this->k;
//        std::vector<double>& v = this->v;
//        std::vector<ReferenceType>& type = this->type;

//        v.resize(this->numOfPoints);

//        // Cap velocity
//        for(int i=0; i< this->numOfPoints ; i++)
//        {
//            if( fabs(k[i]) < 1e-3 ) { v[i] = v[i]; }
//            else
//            {
//                double vel = 0.0;
//                if (type[i] == InLane) { vel = sqrt( maxLatAcc_InLane / fabs(k[i]) ); }
//                else if(type[i] == LaneChange) { vel = sqrt( maxLatAcc_ChangeLane / fabs(k[i]) ); }
//                else if(type[i] == VirtualLaneMild) { vel = sqrt( maxLatAcc_VirtualLaneMild / fabs(k[i]) ); }
//                else if(type[i] == VirtualLaneTight) { vel = sqrt( maxLatAcc_VirtualLaneTight / fabs(k[i]) ); }

//                if(vel > v[i]) { v[i] = v[i]; }
//                else { v[i] = vel; }
//            }
//        }

//        std::vector<double> d(this->numOfPoints-1);

//        for(uint i=0; i<d.size(); i++)
//        {
//            d[i]=hypot(x[i]-x[i+1], y[i]-y[i+1]);
//        }


//        double approxRemainingDistance = this->s.back() - this->s.front();
//        double adjustedLonDec = currV * currV / (2 * approxRemainingDistance);


//        double totalGap = Post_Gap + Pre_Gap;
//        if(approxRemainingDistance < (totalGap - 1) * DP_LonInterval)
//        {
//            v.back() = 0.0;
//            // Reaching to stop, get the minimum one between casual stop & emergent stop!
//            limit_acc_dec(v, d, maxLonAcc, std::min(std::max(adjustedLonDec, maxLonDec), maxLonDec_Emergent));
//        }
//        else
//        {
//            limit_acc_dec(v, d, maxLonAcc, maxLonDec);
//        }


//        return true;
//    }
//    else { return false; }
//}

void Reference::recalculateStation()
{
    if(numOfPoints <= 0) { return; }
    s[0] = 0.0;
    for(int ii=1; ii<numOfPoints; ii++) {
        s[ii] = s[ii-1] + hypot(x[ii]-x[ii-1], y[ii]-y[ii-1]);
    }
}

bool Reference::recalculateEstimatedTime()
{
    this->tEst.resize(this->numOfPoints);

    if(this->numOfPoints <= 0) { return false; }

    int stopIdx = this->numOfPoints;

    this->tEst[0] = 0;
    for(int ii = 1; ii < this->numOfPoints; ii++) {
        if(this->v[ii] + this->v[ii-1] <= 1e-1) {
            stopIdx = ii;
            break;
        }
        else {
            this->tEst[ii] = this->tEst[ii-1] + 2 * ( this->s[ii] - this->s[ii-1] ) / ( this->v[ii] + this->v[ii-1] );
        }
    }

    for(int ii = stopIdx; ii < this->numOfPoints; ii++) {
        this->tEst[ii] = GSL_POSINF;
    }

    return true;
}


void Reference::copyFrom(Reference& from)
{
    this->reset();

    for(int ii=0; ii<from.numOfPoints; ii++) {
        rB.push_back( from.rB.at(ii) );
        lB.push_back( from.lB.at(ii) );

        s.push_back( from.s.at(ii) );
        x.push_back( from.x.at(ii) );
        y.push_back( from.y.at(ii) );
        theta.push_back( from.theta.at(ii) );
        k.push_back( from.k.at(ii) );

        vLim.push_back( from.vLim.at(ii) );
        vMax.push_back( from.vMax.at(ii) );
        v.push_back( from.v.at(ii) );
        tEst.push_back( from.tEst.at(ii) );

        type.push_back( from.type.at(ii) );

        segmentId.push_back( from.segmentId.at(ii) );
    }
    numOfPoints = s.size();

    blocked = from.blocked;
}


void Reference::copyFrom(LanePiece& lane) {
    reset();
    for(int ii=0; ii<lane.numOfPoints; ii++) {
        rB.push_back( - lane.laneWidth / 2 );
        lB.push_back(   lane.laneWidth / 2 );

        s.push_back( lane.s[ii] );
        x.push_back( lane.x[ii] );
        y.push_back( lane.y[ii] );
        theta.push_back( lane.theta[ii] );
        k.push_back( lane.k[ii] );

        vLim.push_back( lane.speedLimit );
        vMax.push_back( lane.speedLimit );
        v.push_back( lane.speedLimit );
        tEst.push_back( 0.0 );

        type.push_back( InLane );

        segmentId.push_back( 0 );
        optFlag.push_back( 0 );
    }
    numOfPoints = lane.s.size();
}


//void Reference::limit_acc_dec(std::vector<double>& v, std::vector<double>& d, double maxAcc, double maxDec)
//{
//    for(uint i=0; i<v.size()-1; i++)
//    {
//        int curr = i;
//        int next = i+1;

//        if(v[next] > v[curr])
//        {
//            double t = (-v[curr] + sqrt(v[curr]*v[curr] + 2 * maxAcc * d[i])) / maxAcc;
//            double maxv = v[curr] + maxAcc * t;
//            if(v[next] > maxv) { v[next] = maxv; }
//        }
//    }

//    for(int i=v.size()-1; i>0; i--)
//    {
//        int curr = i;
//        int next = i-1;
//        double t = (-v[curr] + sqrt(v[curr]*v[curr] + 2 * maxDec * d[i-1])) / maxDec;
//        double maxv = v[curr] + maxDec * t;
//        if(v[next] > maxv) { v[next] = maxv; }
//    }
//}




























bool REFERENCE_TRAIN::projectPlanOntoDemoCL(Reference& cl, Reference& ref)
{
    // For each point on the demo find the nearest idx on the ref input, and set
    xProj.resize( numOfPoints );
    yProj.resize( numOfPoints );
    kProj.resize( numOfPoints );
    vProj.resize( numOfPoints );


    double sRefCoarse[ref.numOfPoints];
    double xRefCoarse[ref.numOfPoints];
    double yRefCoarse[ref.numOfPoints];
    double kRefCoarse[ref.numOfPoints];
    double vRefCoarse[ref.numOfPoints];

    for(int i=0; i<ref.numOfPoints; i++)
    {
        sRefCoarse[i] = ref.s[i];
        xRefCoarse[i] = ref.x[i];
        yRefCoarse[i] = ref.y[i];
        kRefCoarse[i] = ref.k[i];
        vRefCoarse[i] = ref.v[i];
    }

    gsl_interp_accel *acc = gsl_interp_accel_alloc ();
    gsl_spline *Xspline = gsl_spline_alloc (gsl_interp_cspline, ref.numOfPoints);
    gsl_spline *Yspline = gsl_spline_alloc (gsl_interp_cspline, ref.numOfPoints);
    gsl_spline *Kspline = gsl_spline_alloc (gsl_interp_cspline, ref.numOfPoints);
    gsl_spline *Vspline = gsl_spline_alloc (gsl_interp_cspline, ref.numOfPoints);

    // Interpolate with cubic spline if the # of points is >= 4
    gsl_spline_init (Xspline, sRefCoarse, xRefCoarse, ref.numOfPoints);
    gsl_spline_init (Yspline, sRefCoarse, yRefCoarse, ref.numOfPoints);
    gsl_spline_init (Kspline, sRefCoarse, kRefCoarse, ref.numOfPoints);
    gsl_spline_init (Vspline, sRefCoarse, vRefCoarse, ref.numOfPoints);

    vector<double> sRefFine;
    vector<double> xRefFine;
    vector<double> yRefFine;
    vector<double> kRefFine;
    vector<double> vRefFine;

    // Interpolated splines
    int s0 = ref.s.front();
    int sf = ref.s.back();
    for (double ss = s0; ss <= sf; ss += 0.01) {
        sRefFine.push_back( ss );
        xRefFine.push_back( gsl_spline_eval (Xspline, ss, acc) );
        yRefFine.push_back( gsl_spline_eval (Yspline, ss, acc) );
        kRefFine.push_back( gsl_spline_eval (Kspline, ss, acc) );
        vRefFine.push_back( gsl_spline_eval (Vspline, ss, acc) );
    }

    gsl_spline_free (Xspline);
    gsl_spline_free (Yspline);
    gsl_interp_accel_free (acc);


    for(int i=0; i<numOfPoints; i++) {
        double minDist = GSL_POSINF;
        int minId = -1;
        for(vector<double>::size_type k=0; k < sRefFine.size(); k++) {
            double dist = hypot(xRefFine[k] - this->x[i], yRefFine[k] - this->y[i]);
            if(dist < minDist) {
                minDist = dist;
                minId = k;
            }
        }

        xProj[i] = xRefFine[minId];
        yProj[i] = yRefFine[minId];
        kProj[i] = kRefFine[minId];
        vProj[i] = vRefFine[minId];
    }

    // Project onto centerline
//    double maxAbsL = 0.0;
//    double lDemo[numOfPoints];
//    for(int i=0; i<numOfPoints; i++) {
//        lDemo[i] = cl.getSignedDistanceByCoordinates( this->x[i], this->y[i] );

//        maxAbsL = fmax(maxAbsL, fabs(lDemo[i]));
//    }

//    double lProj[numOfPoints];
//    for(int i=0; i<numOfPoints; i++) {
//        lProj[i] = cl.getSignedDistanceByCoordinates( xProj[i], yProj[i] );
//    }

    cumPathError = 0.0;
    cumSpeedError = 0.0;
//    for(int i=0; i<numOfPoints; i++)
//    {
//        cumPathError += fabs(lDemo[i]) / maxAbsL * (lDemo[i]-lProj[i])*(lDemo[i]-lProj[i]);
//        cumSpeedError += (v[i]-vProj[i]) * (v[i]-vProj[i]);
//    }

    for(int i=0; i<numOfPoints; i++)
    {
        cumPathError += (x[i]-xProj[i])*(x[i]-xProj[i]) + (y[i]-yProj[i])*(y[i]-yProj[i]);
        cumSpeedError += (v[i]-vProj[i]) * (v[i]-vProj[i]);
    }

    cumPathError = sqrt(fabs(cumPathError / numOfPoints));
    cumSpeedError = sqrt(fabs(cumSpeedError / numOfPoints));

}

bool REFERENCE_TRAIN::projectPlanOntoDemo(Reference& ref)
{
    // For each point on the demo find the nearest idx on the ref input, and set
    xProj.resize( numOfPoints );
    yProj.resize( numOfPoints );
    kProj.resize( numOfPoints );
    vProj.resize( numOfPoints );


    double sRefCoarse[ref.numOfPoints];
    double xRefCoarse[ref.numOfPoints];
    double yRefCoarse[ref.numOfPoints];
    double kRefCoarse[ref.numOfPoints];
    double vRefCoarse[ref.numOfPoints];

    for(int i=0; i<ref.numOfPoints; i++)
    {
        sRefCoarse[i] = ref.s[i];
        xRefCoarse[i] = ref.x[i];
        yRefCoarse[i] = ref.y[i];
        kRefCoarse[i] = ref.k[i];
        vRefCoarse[i] = ref.v[i];
    }

    gsl_interp_accel *acc = gsl_interp_accel_alloc ();
    gsl_spline *Xspline = gsl_spline_alloc (gsl_interp_cspline, ref.numOfPoints);
    gsl_spline *Yspline = gsl_spline_alloc (gsl_interp_cspline, ref.numOfPoints);
    gsl_spline *Kspline = gsl_spline_alloc (gsl_interp_cspline, ref.numOfPoints);
    gsl_spline *Vspline = gsl_spline_alloc (gsl_interp_cspline, ref.numOfPoints);

    // Interpolate with cubic spline if the # of points is >= 4
    gsl_spline_init (Xspline, sRefCoarse, xRefCoarse, ref.numOfPoints);
    gsl_spline_init (Yspline, sRefCoarse, yRefCoarse, ref.numOfPoints);
    gsl_spline_init (Kspline, sRefCoarse, kRefCoarse, ref.numOfPoints);
    gsl_spline_init (Vspline, sRefCoarse, vRefCoarse, ref.numOfPoints);

    vector<double> sRefFine;
    vector<double> xRefFine;
    vector<double> yRefFine;
    vector<double> kRefFine;
    vector<double> vRefFine;

    // Interpolated splines
    int s0 = ref.s.front();
    int sf = ref.s.back();
    for (double ss = s0; ss <= sf; ss += 0.01) {
        sRefFine.push_back( ss );
        xRefFine.push_back( gsl_spline_eval (Xspline, ss, acc) );
        yRefFine.push_back( gsl_spline_eval (Yspline, ss, acc) );
        kRefFine.push_back( gsl_spline_eval (Kspline, ss, acc) );
        vRefFine.push_back( gsl_spline_eval (Vspline, ss, acc) );
    }

    gsl_spline_free (Xspline);
    gsl_spline_free (Yspline);
    gsl_interp_accel_free (acc);


    for(int i=0; i<numOfPoints; i++) {
        double minDist = GSL_POSINF;
        int minId = -1;
        for(vector<double>::size_type k=0; k < sRefFine.size(); k++) {
            double dist = hypot(xRefFine[k] - this->x[i], yRefFine[k] - this->y[i]);
            if(dist < minDist) {
                minDist = dist;
                minId = k;
            }
        }

        xProj[i] = xRefFine[minId];
        yProj[i] = yRefFine[minId];
        kProj[i] = kRefFine[minId];
        vProj[i] = vRefFine[minId];
    }


    cumPathError = 0;
    cumSpeedError = 0;

    for(int i=0; i<numOfPoints; i++)
    {
        cumPathError += (x[i]-xProj[i])*(x[i]-xProj[i]) + (y[i]-yProj[i])*(y[i]-yProj[i]);
        cumSpeedError += (v[i]-vProj[i]) * (v[i]-vProj[i]);

//        cumPathError += hypot( (x[i]-xProj[i]), (y[i]-yProj[i]) );
//        cumSpeedError += fabs(v[i]-vProj[i]);
    }


    return true;
}

void REFERENCE_UNFILTERED::recalculateStation()
{
    numOfPoints = this->s.size();
    if(numOfPoints <= 0) { return; }

    s[0] = 0.0;
    for(int ii=1; ii<numOfPoints; ii++)
    {
        s[ii] = s[ii-1] + hypot(x[ii]-x[ii-1], y[ii]-y[ii-1]);
    }
}

void REFERENCE_UNFILTERED::conditioning()
{
    for(vector<double>::size_type k = 0; k < s.size(); k++) {
        if(k == 0) { s[k] = 0.0; }
        else {
            double dist = hypot(x[k] - x[k-1], y[k] - y[k-1]);
            s[k] = s[k-1] + dist;
        }
    }

    std::vector<double> sNew, xNew, yNew, thetaNew, vNew;
    for(vector<double>::size_type k = 0; k < s.size(); k++) {
        if(k == 0 || s[k] - s[k-1] > 0.05) {
            sNew.push_back( s[k] );
            xNew.push_back( x[k] );
            yNew.push_back( y[k] );
            thetaNew.push_back( theta[k] );
            vNew.push_back( v[k] );
        }
    }

    this->s = sNew;
    this->x = xNew;
    this->y = yNew;
    this->theta = thetaNew;
    this->v = vNew;

    this->numOfPoints = s.size();
}

void REFERENCE_UNFILTERED::reinterpolate(double ds)
{
    std::vector<double> S; S.clear();
    std::vector<double> X; X.clear();
    std::vector<double> Y; Y.clear();
    std::vector<double> H; H.clear();
    std::vector<double> V; V.clear();

    int numOfPoints2Interp = this->numOfPoints;

    if(numOfPoints2Interp >= 2) {
        double s[numOfPoints2Interp];
        double x[numOfPoints2Interp];
        double y[numOfPoints2Interp];
        double v[numOfPoints2Interp];

        for (int i = 0; i < numOfPoints2Interp; i++) {
            s[i] = this->s[i];
            x[i] = this->x[i];
            y[i] = this->y[i];
            v[i] = this->v[i];
        }

        gsl_interp_accel *acc = gsl_interp_accel_alloc ();
        gsl_spline *Xspline;
        gsl_spline *Yspline;
        gsl_spline *Vspline;

        // Interpolate with cubic spline if the # of points is >= 4
        if(numOfPoints2Interp >= 4) {
            Xspline = gsl_spline_alloc (gsl_interp_cspline, numOfPoints2Interp);
            Yspline = gsl_spline_alloc (gsl_interp_cspline, numOfPoints2Interp);
            Vspline = gsl_spline_alloc (gsl_interp_cspline, numOfPoints2Interp);
        }
        else {
            Xspline = gsl_spline_alloc (gsl_interp_linear, numOfPoints2Interp);
            Yspline = gsl_spline_alloc (gsl_interp_linear, numOfPoints2Interp);
            Vspline = gsl_spline_alloc (gsl_interp_linear, numOfPoints2Interp);
        }

        gsl_spline_init (Xspline, s, x, numOfPoints2Interp);
        gsl_spline_init (Yspline, s, y, numOfPoints2Interp);
        gsl_spline_init (Vspline, s, v, numOfPoints2Interp);

        // Interpolated splines
        for (double ss = 0.0; ss <= s[numOfPoints2Interp-1] ; ss+=ds) {
            S.push_back( ss );
            X.push_back( gsl_spline_eval (Xspline, ss, acc) );
            Y.push_back( gsl_spline_eval (Yspline, ss, acc) );
            H.push_back( 0.0 );
            V.push_back( gsl_spline_eval (Vspline, ss, acc) );
        }

        double numOfPointsNew = S.size();
        if( numOfPointsNew >= 2 ) {
            for(int i=0; i<numOfPointsNew-1; i++) {
                H[i] = atan2( Y[i+1]-Y[i], X[i+1]-X[i] );
            }
            H[numOfPointsNew-1] = H[numOfPointsNew-2];
        }

        gsl_spline_free (Xspline);
        gsl_spline_free (Yspline);
        gsl_spline_free (Vspline);
        gsl_interp_accel_free (acc);
    }
    else {return;}

    this->s = S;
    this->x = X;
    this->y = Y;
    this->theta = H;
    this->v = V;

    this->numOfPoints = this->s.size();
}

int REFERENCE_UNFILTERED::getNearestIndexByCoordinates(double x, double y)
{
    double minDist = GSL_POSINF;
    int minId = -1;
    for(int i=0; i < numOfPoints; i++)
    {
        double dist = hypot(x - this->x[i], y - this->y[i]);
        if(dist < minDist)
        {
            minDist = dist;
            minId = i;
        }
    }

    return minId;
}


double REFERENCE_UNFILTERED::getSignedDistanceByCoordinates(double x, double y)
{
    float minDist1 = GSL_POSINF;
    int s_Id1 = -1;
    float minDist2 = GSL_POSINF;
    int s_Id2 = -1;


    for(unsigned int j = 0; j < this->x.size(); j++ )
    {
        double dist = hypot(x - this->x[j], y - this->y[j]);
        if(minDist1 > dist)
        {
            minDist2 = minDist1;
            minDist1 = dist;
            s_Id2 = s_Id1;
            s_Id1 = j;
        }
        else if(minDist2 > dist)
        {
            minDist2 = dist;
            s_Id2 = j;
        }
    }

    double angle1 = atan2(y - this->y[s_Id1], x - this->x[s_Id1]);
    double angle2;

    if(s_Id1 >= numOfPoints - 1)
    {
        //If the nearest is the last point in ci_Route ...
        angle2 = atan2(this->y[s_Id1] - this->y[s_Id1-1],
                this->x[s_Id1] - this->x[s_Id1-1]);
    }
    else
    {
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
    if (angle2 >= 0)
    {
        if (angle1 >= 0)
        { // if both in "top half" of atan2 plot
            if((angle1 - angle2) >= 0)
            {
                //left side
                dist = minDist;
            }
            else
            {
                // right side
                dist = -minDist;
            }
        }
        if (angle1 < 0)
        { // angle2 in top half, angle1 in bottom half
            if(angle1 >= angle2- M_PI)
            {
                // right side
                dist = -minDist;
            }
            else
            {
                //left side
                dist = minDist;
            }
        }
    }
    else
    {
        if (angle1 >= 0)
        { // angle2 in bottom half, angle1 in top half
            if (angle1 <= (angle2 + M_PI))
            {
                //left side
                dist = minDist;
            }
            else
            {
                // right side
                dist = -minDist;
            }
        }
        if (angle1 < 0)
        { // angle1 and angle2 both in bottom half
            if (angle1 >= angle2)
            {
                //left side
                dist = minDist;
            }
            else
            {
                // right side
                dist = -minDist;
            }
        }
    }

    return dist;
}
