#include "CommFunctions.h"

/**
 * @brief Interpolate <arg, par> at every step
 * @param arg
 * @param par
 * @param step
 * @param argInterp
 * @param parInterp
 * @return
 */
bool plan_interpolate( const vector<double> arg, const vector<double> par, const vector<double>& argInterp,
                       vector<double>& parInterp  )
{
    if( arg.size() != par.size() || arg.size() == 0 )
        return false;

    int Num = arg.size();

    double S[Num];
    double X[Num];
    for(int i=0; i<Num; i++) {
        S[i] = arg[i];
        X[i] = par[i];
    }

    gsl_interp_accel *acc = gsl_interp_accel_alloc ();
    gsl_spline* spline;
    if( Num >= 4 ) {
        spline = gsl_spline_alloc (gsl_interp_cspline, Num);
    } else if( Num >= 2 ) {
        spline = gsl_spline_alloc (gsl_interp_linear, Num);
    }
    else {
        parInterp = par;
        return true;
    }

    gsl_spline_init (spline, S, X, Num);

    std::vector<double> ss = argInterp;
    std::vector<double> xx;
    // Interpolated splines
    for (int i=0; i<ss.size(); i++) {
        xx.push_back( gsl_spline_eval (spline, ss[i], acc) );
    }
    gsl_spline_free (spline);
    gsl_interp_accel_free (acc);

    parInterp = xx;

    return true;
}

/**
 * @brief plan_calcHeading calculate the heading with simple differencing
 * @param xx
 * @param yy
 * @param hh
 * @return
 */
bool plan_calcHeading(const vector<double> xx, const vector<double> yy,
                      vector<double>& hh )
{
    if( xx.size() != yy.size() || xx.empty() || yy.empty() ) return false;

    int num = xx.size();

    std::vector<double> heading(num, 0.0);
    if( num >= 2 ) {
        for(int i=1; i<num; i++) {
            heading[i] = atan2( yy[i]-yy[i-1], xx[i]-xx[i-1] );
        }
        heading[0] = heading[1];
    }

    hh = heading;

    return true;
}


/**
 * @brief find the curvature (k) of each point (s,x,y) on a given trace
 * @param s
 * @param x
 * @param y
 * @param h
 * @param k
 */
bool plan_fitCurvature(const vector<double> ss, const vector<double> xx, const vector<double> yy,
                       vector<double>& kk )
{
    if( ss.size() != xx.size() || xx.size() != yy.size() || ss.size() != yy.size() || ss.size() == 0)
        return false;

    int sideNUM = 3;

    int numOfPoints = ss.size();
    kk.resize( numOfPoints );

    if( numOfPoints >= 2*sideNUM-1 ) {
        for(int i = 0; i < numOfPoints; i++)
        {
            int id0 = (i-sideNUM >= 0) ? (i-sideNUM):(0);
            int id1 = (i+sideNUM <= numOfPoints-1) ? (i+sideNUM):(numOfPoints-1);

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
                si = ss[id0+j];
                xi = xx[id0+j];
                yi = yy[id0+j];

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

            double s_ = ss[i];
            double x_ = Cx(2) * s_ * s_ + Cx(1) * s_ + Cx(0);
            double xd_ = 2 * Cx(2) * s_ + Cx(1);
            double xdd_ = 2 * Cx(2);
            double y_ = Cy(2) * s_ * s_ + Cy(1) * s_ + Cy(0);
            double yd_ = 2 * Cy(2) * s_ + Cy(1);
            double ydd_ = 2 * Cy(2);

            kk[i] = (xd_ * ydd_ - yd_ * xdd_) / ( sqrt( (xd_*xd_ + yd_*yd_)*(xd_*xd_ + yd_*yd_)*(xd_*xd_ + yd_*yd_) ) );

            gsl_matrix_free (S);
            gsl_vector_free (x);
            gsl_vector_free (y);
            gsl_vector_free (w);

            gsl_vector_free (cx);
            gsl_vector_free (cy);
            gsl_matrix_free (cov);
        }
    }
    else {
        for(int i = 0; i < numOfPoints; i++) {
            kk[i] = 0.0;
        }
    }

    return true;
}


/**
 * @brief calculate time tt from (ss, vv) from 0 second assuming piecewise constant acceleration
 * @param ss
 * @param vv
 * @param tt
 * @return
 */
bool plan_calculateTime(const vector<double> ss, const vector<double> vv,
                        vector<double>& tt)
{
    if( ss.size() != vv.size() || ss.size() == 0 )
        return false;

    int numOfPoints = ss.size();

    tt.resize( numOfPoints );

    tt[0] = 0;
    for(int i=1; i < numOfPoints; i++) {
        tt[i] = tt[i-1] + 2 * ( ss[i] - ss[i-1] ) / ( vv[i] + vv[i-1] );
    }

    return true;
}

/**
 * @brief Cap the input angle in radian from -pi to pi
 * @param rot
 * @return
 */
double plan_capAngleNegPiToPi(double rot)
{
    while(rot < -M_PI) rot += 2*M_PI;
    while(rot >= M_PI) rot -= 2*M_PI;

    return rot;
}

/**
 * @brief Cap the input angle in radian from 0 to 2pi
 * @param rot
 * @return
 */
double plan_capAngleZeroTo2Pi(double rot) {
    while(rot < 0.0)        { rot += 2*M_PI; }
    while(rot >= 2*M_PI)    { rot -= 2*M_PI; }

    return rot;
}



void LatLon2NE( double lat, double lon, double *pE, double *pN )
{
    if(FLAG_USE_UTMWGS84) {
        LatLonToUtmWGS84(lat, lon, *pN, *pE, 17);
    }
    else {
        // When a new map is used, consider change the lat/lon reference point
        // And change the initial veh state in simulator
        ConvertLatLonToNE(LAT_REF, LON_REF, lat, lon, pE, pN);
    }
}


void NE2LatLon( double E, double N, double *pLat, double *pLon)
{
    if(FLAG_USE_UTMWGS84) {
        UtmToLatLonWGS84(*pLat, *pLon, N, E, 17);
    }
    else {
        // When a new map is used, consider change the lat/lon reference point
        // And change the initial veh state in simulator
        ConvertNEToLatLon(E, N, LAT_REF, LON_REF, pLat, pLon);
    }
}


double normal_dist(std::pair<double, double> P1, std::pair<double, double> P2, std::pair<double, double> P)
{
    return fabs((P2.first-P1.first)*(P1.second-P.second)-(P1.first-P.first)*(P2.second-P1.second)) / sqrt((P2.first-P1.first)*(P2.first-P1.first) + (P2.second-P1.second)*(P2.second-P1.second));
}

std::vector<int> adaptive_sample(std::vector<double> xc, std::vector<double> yc, std::vector<double> sc, int numOfPoints, double max_gap, double max_dist)
{
    // Customized D-P to filter peudo-piecewise-linear reference
    std::vector<int> stack;

    int lastId = 0;
    std::pair<double, double> lastPt;
    lastPt.first = xc[lastId]; lastPt.second = yc[lastId];

    stack.push_back(lastId);

    for(int currId = 0; currId < numOfPoints; currId++) {
        std::pair<double, double> currPt;
        currPt.first = xc[currId]; currPt.second = yc[currId];

        double maxDist = 0.0;
        for(int tmpId = lastId; tmpId <= currId; tmpId++) {
            std::pair<double, double> tmpPt;
            tmpPt.first = xc[tmpId]; tmpPt.second = yc[tmpId];

            double dist = normal_dist(lastPt, currPt, tmpPt);

            if(dist > maxDist) {
                maxDist = dist;
            }
        }

        if( sc[currId] - sc[lastId] >= max_gap || maxDist >= max_dist ) {
            stack.push_back(currId);
            lastId = currId;
            lastPt.first = xc[lastId]; lastPt.second = yc[lastId];
        }
    }

    if(stack.back() != numOfPoints-1) {
        stack.push_back(numOfPoints - 1);
    }

    return stack;
}

std::vector<int> compress_Dauglas_Peuker(std::vector<double>& X, std::vector<double>& Y, int numOfPoints)
{
    std::vector<int> stack;
    std::vector<int> list_pts;

    int idx_first = 0;
    std::pair<double, double> point_first;
    point_first.first = X.at(idx_first); point_first.second = Y.at(idx_first);

    int idx_last = numOfPoints - 1;
    std::pair<double, double> point_last;
    point_last.first = X.at(idx_last); point_last.second = Y.at(idx_last);

    list_pts.push_back( idx_first );

    stack.push_back(idx_last);

    while( !stack.empty() ) {
        int idx_dmax = -1;
        double dmax = 0;

        for(int idx = idx_first + 1; idx < idx_last; ++idx) {
            std::pair<double, double> point_tmp;
            point_tmp.first = X.at(idx); point_tmp.second = Y.at(idx);

            double d = normal_dist(point_first, point_last, point_tmp);

            if(d>dmax) {
                dmax = d;
                idx_dmax = idx;
            }
        }

        #define MAX_LINEAR_TOLERATION   0.2
        if(dmax >= MAX_LINEAR_TOLERATION) {
            idx_last = idx_dmax;
            point_last.first = X.at(idx_last); point_last.second = Y.at(idx_last);

            stack.push_back(idx_dmax);
        }
        else {
            list_pts.push_back(idx_last);

            idx_first = stack.back();
            point_first.first = X.at(idx_first); point_first.second = Y.at(idx_first);

            stack.pop_back();

            if(!stack.empty()) {
                idx_last = stack.back();
                point_last.first = X.at(idx_last); point_last.second = Y.at(idx_last);
            }
        }
    }

    return list_pts;
}

Fitting::Fitting()
{
}

int Fitting::f(const gsl_vector * x, gsl_vector * f)
{
    double p0 = gsl_vector_get (x, 0);
    double p1 = gsl_vector_get (x, 1);
    double p2 = gsl_vector_get (x, 2);
    double p3 = gsl_vector_get (x, 3);
    double p4 = gsl_vector_get (x, 4);
    double p5 = gsl_vector_get (x, 5);

    for (int i = 0; i < N_; i++)
    {
        /* Model Yi = p0 + p1*t + p2*t^2 + p3*t^3 + p4*t^4 + p5*t^5  */
        double t = tTraj_[i];
        double Yi = p0 + p1*t + p2*t*t + p3*t*t*t + p4*t*t*t*t + p5*t*t*t*t*t;
        gsl_vector_set (f, i, (Yi - kTraj_[i]));
    }

    return GSL_SUCCESS;
}


int Fitting::df (const gsl_vector * x,  gsl_matrix * J)
{
    for (int i = 0; i < N_; i++)
    {
        /* Jacobian matrix J(i,j) = dfi / dxj, */
        /* where fi = (Yi - yi),      */
        /*       Yi = p0 + p1*t + p2*t^2 + p3*t^3 + p4*t^4 + p5*t^5  */
        /* and the xj are the parameters (p0, p1, p2, p3, p4, p5) */
        double t = tTraj_[i];
        double dYi_dp0 = 1;
        double dYi_dp1 = t;
        double dYi_dp2 = t*t;
        double dYi_dp3 = t*t*t;
        double dYi_dp4 = t*t*t*t;
        double dYi_dp5 = t*t*t*t*t;

        gsl_matrix_set (J, i, 0, dYi_dp0);
        gsl_matrix_set (J, i, 1, dYi_dp1);
        gsl_matrix_set (J, i, 2, dYi_dp2);
        gsl_matrix_set (J, i, 3, dYi_dp3);
        gsl_matrix_set (J, i, 4, dYi_dp4);
        gsl_matrix_set (J, i, 5, dYi_dp5);
    }
    return GSL_SUCCESS;
}


int Fitting::fdf (const gsl_vector * x, gsl_vector * f, gsl_matrix * J)
{
    this->f(x, f);
    this->df(x, J);

    return GSL_SUCCESS;
}

int Fitting::sf(const gsl_vector * x, void *params, gsl_vector * f)
{
    Fitting *foo = (Fitting *)params;

    return foo->f(x, f);
}

int Fitting::sdf(const gsl_vector * x, void *params, gsl_matrix * J)
{
    Fitting *foo = (Fitting *)params;

    return foo->df(x, J);
}

int Fitting::sfdf(const gsl_vector * x, void *params, gsl_vector * f, gsl_matrix * J)
{
    Fitting *foo = (Fitting *)params;

    return foo->fdf(x, f, J);
}

void Fitting::findPathParameter(std::vector<double> t_traj, std::vector<double> k_traj,
                                std::vector<double>& P)
{
    N_ = t_traj.size();
    tTraj_ = t_traj;
    kTraj_ = k_traj;


    const size_t n = N_;
    const size_t p = 6;


    int status;


    double x_init[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    gsl_vector_view x = gsl_vector_view_array (x_init, p);

    gsl_rng_env_setup();

    const gsl_rng_type * type = gsl_rng_default;
    gsl_rng* r = gsl_rng_alloc (type);

    gsl_multifit_function_fdf f;
    f.f = &Fitting::sf;
    f.df = &Fitting::sdf;
    f.fdf = &Fitting::sfdf;
    f.n = n;
    f.p = p;
    f.params = this;

    gsl_matrix *covar = gsl_matrix_alloc (p, p);

    const gsl_multifit_fdfsolver_type* T = gsl_multifit_fdfsolver_lmsder;
    gsl_multifit_fdfsolver* s = gsl_multifit_fdfsolver_alloc (T, n, p);
    gsl_multifit_fdfsolver_set (s, &f, &x.vector);


    unsigned int iter = 0;
    do
    {
        iter++;
        status = gsl_multifit_fdfsolver_iterate (s);

//        printf ("status = %s\n", gsl_strerror (status));

        if (status) { break; }

        status = gsl_multifit_test_delta (s->dx, s->x, 1e-4, 1e-4);
    }
    while (status == GSL_CONTINUE && iter < 500);

    gsl_multifit_covar (s->J, 0.0, covar);

#define FIT(i) gsl_vector_get(s->x, i)
//#define ERR(i) sqrt(gsl_matrix_get(covar,i,i))

    P.resize(6);
    P[0] = FIT(0);
    P[1] = FIT(1);
    P[2] = FIT(2);
    P[3] = FIT(3);
    P[4] = FIT(4);
    P[5] = FIT(5);

    gsl_multifit_fdfsolver_free (s);
    gsl_matrix_free (covar);
    gsl_rng_free (r);
}


SVDWeighted::SVDWeighted()
    :xData_(NULL),
    covData_(NULL),
    yData_(NULL),
    wData_(NULL),
    cData_(NULL),
    workSpace_(NULL),
    maxSamples_(0),
    maxCoefficients_(0)
{
}

SVDWeighted::~SVDWeighted()
{
    deallocateData();
    deallocateWorkspace();
}

bool SVDWeighted::deallocateData()
{
    if(xData_ != NULL)
    {
        gsl_matrix_free(xData_);
        xData_ = NULL;
    }
    if(covData_ != NULL)
    {
        gsl_matrix_free(covData_);
        covData_ = NULL;
    }
    if(yData_ != NULL)
    {
        gsl_vector_free(yData_);
        yData_ = NULL;
    }
    if(wData_ != NULL)
    {
        gsl_vector_free(wData_);
        wData_ = NULL;
    }
    if(cData_ != NULL)
    {
        gsl_vector_free(cData_);
        cData_ = NULL;
    }
    return true;
}

bool SVDWeighted::deallocateWorkspace()
{
    if(workSpace_ != NULL)
    {
        gsl_multifit_linear_free(workSpace_);
        workSpace_ = NULL;
    }
    return true;
}

bool SVDWeighted::allocateData(const int & numberSamples, const int & numberCoefficients)
{
    bool status(true);
    xData_ = gsl_matrix_alloc(numberSamples, numberCoefficients);
    covData_ = gsl_matrix_alloc(numberCoefficients, numberCoefficients);
    yData_ = gsl_vector_alloc(numberSamples);
    wData_ = gsl_vector_alloc(numberSamples);
    cData_ = gsl_vector_alloc(numberCoefficients);
    status = xData_ != NULL && covData_ != NULL && yData_ != NULL && wData_ != NULL && cData_ != NULL;
    return status;
}

bool SVDWeighted::checkMemoryAllocation(const int & numberSamples, const int & numberCoefficients)
{
    bool status(false);

    if(numberSamples > maxSamples_ || numberCoefficients > maxCoefficients_)
    {
        deallocateWorkspace();
        if(numberSamples > maxSamples_)
        {
            maxSamples_ = numberSamples;
        }
        if(numberCoefficients > maxCoefficients_)
        {
            maxCoefficients_ = numberCoefficients;
        }
        workSpace_ = gsl_multifit_linear_alloc(maxSamples_, maxCoefficients_);
        status = (workSpace_ != NULL) && allocateData(maxSamples_, maxCoefficients_);
    }
    return status;
}

/**
 * @brief Performs a least-squares fit on the set of x and y values, using the weights w, to match a function a given
 * order.
 *
 * @return true for success, false for failure.
 *
 */
bool SVDWeighted::fitWeightedValues(const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & w, const unsigned int & order, std::vector<double> & coeffs, double & chiSquared)
{
    const unsigned int numberSamples(x.size());
    const unsigned int numberCoefficients(order + 1);
    coeffs.resize(numberCoefficients);
    bool status(false);

    status = this->checkMemoryAllocation(numberSamples, numberCoefficients);
    if(status)
    {
        for(unsigned int ii(0); ii < numberSamples; ++ii)
        {
            gsl_matrix_set(xData_, ii, 0, 1.0);
            for(unsigned int jj(1); jj <= order; ++jj)
            {
                gsl_matrix_set(xData_, ii, jj, pow(x[ii],jj));
            }

            gsl_vector_set(yData_, ii, y[ii]);
            gsl_vector_set(wData_, ii, w[ii]);
        }

        gsl_multifit_wlinear(xData_, wData_, yData_, cData_, covData_, &chiSquared, workSpace_);

        for(unsigned int ii(0); ii < numberCoefficients; ++ii)
        {
            coeffs[ii] = gsl_vector_get(cData_, ii);
        }
        status = true;
    }
    return status;
}

/**
 * @brief Performs a least-squares fit on the set of x and y values, fitting to a polynomial of the given order.
 *
 * @return true for success, false for failure.
 */
bool SVDWeighted::fitValues(const std::vector<double> & x, const std::vector<double> & y, const unsigned int & order, std::vector<double> & coeffs, double & chiSquared)
{
    bool status(false);
    const unsigned int numberSamples(x.size());
    vector<double> weights(numberSamples, 1.0);

    status = this->fitWeightedValues(x, y, weights, order, coeffs, chiSquared);
    return status;
}
