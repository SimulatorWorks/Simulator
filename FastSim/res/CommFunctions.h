#ifndef COMMFUNCTIONS_H
#define COMMFUNCTIONS_H

#include "Comm.h"

using namespace std;

bool plan_interpolate( const vector<double> arg, const vector<double> par, const vector<double>& argInterp,
                       vector<double>& parInterp );

bool plan_calcHeading(const vector<double> xx, const vector<double> yy,
                      vector<double>& hh );

bool plan_fitCurvature(const vector<double> ss, const vector<double> xx, const vector<double> yy,
                       vector<double>& kk );

bool plan_calculateTime(const vector<double> ss, const vector<double> vv,
                        vector<double>& tt);

double plan_capAngleNegPiToPi(double rot);
double plan_capAngleZeroTo2Pi(double rot);


double normal_dist(std::pair<double, double> P1, std::pair<double, double> P2, std::pair<double, double> P);
std::vector<int> adaptive_sample(std::vector<double> xc, std::vector<double> yc, std::vector<double> sc, int numOfPoints, double max_gap, double max_dist);
std::vector<int> compress_Dauglas_Peuker(std::vector<double>& X, std::vector<double>& Y, int numOfPoints);

void LatLon2NE( double lat, double lon, double *pE, double *pN );
void NE2LatLon( double E, double N, double *pLat, double *pLon);

class Fitting
{
public:
    Fitting();

    void findPathParameter(std::vector<double> t_traj, std::vector<double> k_traj,
                           std::vector<double>& P);

    std::vector<std::pair<double, double> > tkPairs;

    std::pair<double, double> getPoint(const uint ii, const gsl_vector * const x);

    int f(const gsl_vector * x, gsl_vector * f);
    int df(const gsl_vector * x,  gsl_matrix * J);
    int fdf(const gsl_vector * x, gsl_vector * f, gsl_matrix * J);

    static int sf(const gsl_vector * x, void *params, gsl_vector * f);
    static int sdf(const gsl_vector * x, void *params, gsl_matrix * J);
    static int sfdf(const gsl_vector * x, void *params, gsl_vector * f, gsl_matrix * J);

private:
    int N_;
    std::vector<double> tTraj_;
    std::vector<double> kTraj_;

};

/**
 * @brief Class for performing least-squares fit of data using SVD, with the option of weighting input points.
 *
 * This class provides a nice C++ wrapper for the GSL's weighted SVD calls. Since this is not included in the GSLMM
 * wrapper according to the documention, this class provides that interface.
 */
class SVDWeighted
{
    public:
        SVDWeighted();
        ~SVDWeighted();

        bool fitWeightedValues(const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & w, const unsigned int & order, std::vector<double> & coeffs, double & chiSquared);
        bool fitValues(const std::vector<double> & x, const std::vector<double> & y, const unsigned int & order, std::vector<double> & coeffs, double & chiSquared);

    protected:
        gsl_matrix * xData_, *covData_;
        gsl_vector * yData_, *wData_, *cData_;
        gsl_multifit_linear_workspace * workSpace_;
        int maxSamples_, maxCoefficients_;

        bool deallocateData();
        bool allocateData(const int & numberSamples, const int & numberCoefficients);
        bool deallocateWorkspace();
        bool checkMemoryAllocation(const int & numberSamples, const int & numberCoefficients);
};


#endif // COMMFUNCTIONS_H
