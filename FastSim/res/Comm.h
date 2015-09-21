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
* @file         Def4Common.h
* @author       Tianyu Gu
* @date         06/10/2013
*/
#ifndef _Def4Common_
#define _Def4Common_

//#define FLAG_IN_TRUCS
//#define COMPILATION_FOR_GMACS

#include <string>
#include <vector>
#include <deque>

#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <iomanip>

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <pwd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <omp.h>
#include <fftw3.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_fit.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_errno.h>

#include <QtCore>
#include <QVector>
#include <QPair>
#include <QQueue>
#include <QDebug>

#include "qwt_plot.h"
#include "qwt_plot_curve.h"

#include "utm.h"
#include "LatLongCalculations.h"

#ifdef FLAG_IN_TRUCS
    #include <boost/serialization/vector.hpp>
    #include <boost/date_time/posix_time/posix_time.hpp>

    #include <task/Task.h>

    #include <MotionCommands/MotionCommands.h>
    #include <RoadWorldModel/RoadWorldModel.h>
    #include <RoadWorldModel/RoadWorldModelUpdater.h>
    #include <RoadWorldModel/BlockedLinks.h>
    #include <interfaces/RoadWorldModel/Input/Abstract.h>
    #include <interfaces/RoadBlockageSet/Output/Abstract.h>

    #include <CONFIG_GPU.h>

    #include <utility>
    #include <log/Logger.h>
#endif


#ifdef FLAG_IN_TRUCS
    #define log_debug1(...) logger_.log_debug( (ostringstream() << "\t" << __VA_ARGS__).str() )
    #define log_info1(...)  logger_.log_info( (ostringstream() << "\t" << __VA_ARGS__).str() )
    #define log_warn1(...)  logger_.log_warn( (ostringstream() << "\t" << __VA_ARGS__).str() )
    #define log_error1(...) logger_.log_error( (ostringstream() << "\t" << __VA_ARGS__).str() )

    #define log_debug2(...) logger_.log_debug( (ostringstream() << "\t\t" << __VA_ARGS__).str() )
    #define log_info2(...)  logger_.log_info( (ostringstream() << "\t\t" << __VA_ARGS__).str() )
    #define log_warn2(...)  logger_.log_warn( (ostringstream() << "\t\t" << __VA_ARGS__).str() )
    #define log_error2(...) logger_.log_error( (ostringstream() << "\t\t" << __VA_ARGS__).str() )
#else
    #define log_debug1(...) std::cout << "LOG_DBUG: \t" << __VA_ARGS__ << std::endl
    #define log_info1(...)  std::cout << "LOG_INFO: \t" << __VA_ARGS__ << std::endl
    #define log_warn1(...)  std::cout << "LOG_WARN: \t" << __VA_ARGS__ << std::endl
    #define log_error1(...) std::cout << "LOG_ERRO: \t" << __VA_ARGS__ << std::endl

    #define log_debug2(...) std::cout << "LOG_DBUG: \t\t" << __VA_ARGS__ << std::endl
    #define log_info2(...)  std::cout << "LOG_INFO: \t\t" << __VA_ARGS__ << std::endl
    #define log_warn2(...)  std::cout << "LOG_WARN: \t\t" << __VA_ARGS__ << std::endl
    #define log_error2(...) std::cout << "LOG_ERRO: \t\t" << __VA_ARGS__ << std::endl
#endif

#ifdef FLAG_IN_TRUCS
    #define GET_DIR_CONFIG string(getenv("XXXXXX"))
#else
    #ifdef COMPILATION_FOR_GMACS
        #define GET_DIR_CONFIG string("/home/gmacs/traj-planner/trajplanner/bin/CONFIG/")
    #else
        #define GET_DIR_CONFIG string(getenv("DIR_MY_SIM_CONFIG"))
    #endif
#endif


using namespace std;

// Coordinate Transformation Method
#define FLAG_USE_UTMWGS84   true

#define LAT_REF 42.518074
#define LON_REF -83.044961


#define NM_CELL_SIZE                            0.2                 /* width of map cells (in meters) */
#define NM_CELL_SIZE_INV                        5.0                 // 1/NM_CELL_SIZE


#ifndef DEG2RAD
#define DEG2RAD               0.017453292519943295474
#endif

#ifndef RAD2DEG
#define RAD2DEG               57.295779513082322865
#endif

#define G       9.8
#define INF     1E20


#define NUM_FIT_FRONT 5
#define NUM_FIT_BACK 5


// Data structure
enum ColorEnum {CL_RED, CL_ORANGE, CL_YELLOW, CL_GREEN, CL_BLUE, CL_PURPUL, CL_WHITE};

struct GenericPoint
{
    GenericPoint() {}
    GenericPoint(double x0, double y0, ColorEnum color0, double radius0) { x = x0; y= y0; color=color0; radius = radius0; }

    double x;
    double y;

    ColorEnum color;
    double radius;

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version){
        ar & x;
        ar & y;
        ar & color;
        ar & radius;
    }
#endif
};

struct GenericPoint3D
{
    GenericPoint3D() {}
    GenericPoint3D(double x0, double y0, double z0) { x = x0; y = y0; z = z0; }

    double x;
    double y;
    double z;

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version){
        ar & x;
        ar & y;
        ar & z;
    }
#endif
};

struct GenericPose
{
    GenericPose() {}
    GenericPose(double x0, double y0, double h0) { x = x0; y= y0; theta=h0; }
    void reset() { x = 0.0; y = 0.0; theta = 0.0; }

    double x;
    double y;
    double theta;

    ColorEnum color;
    double radius;

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version){
        ar & x;
        ar & y;
        ar & theta;
        ar & color;
        ar & radius;
    }
#endif
};


struct StateSpatial
{
    double x;
    double y;
    double theta;
    double k;
    double dk;

    void reset() {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        k = 0.0;
        dk = 0.0;
    }

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & x;
        ar & y;
        ar & theta;
        ar & k;
        ar & dk;
    }
#endif
};


struct StateTemporal
{
    double v;
    double a;

    void reset() {
        v = 0.0;
        a = 0.0;
    }

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & v;
        ar & a;
    }
#endif
};


struct State
{
    StateSpatial  spatial;
    StateTemporal temporal;

    void reset() {
        spatial.reset();
        temporal.reset();
    }

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & spatial;
        ar & temporal;
    }
#endif
};

struct MODEL_VEH_DYNAMICS
{
    double L;          // wheelBase,                                                m
    double LB1;        // the length between the front axle and the front bumper,   m
    double LB2;        // the length between the rear axle and the rear bumper,     m
    double L1;         // the length between the centroid and the front axle,       m
    double L2;         // the length between the centroid and the rear axle,        m
    double Width;      // the width of the car,                                     m

    State vehState;

    void getVehCorners(double& lf_x, double& lf_y,
                       double& rf_x, double& rf_y,
                       double& lb_x, double& lb_y,
                       double& rb_x, double& rb_y) {
        double x = vehState.spatial.x;
        double y = vehState.spatial.y;
        double theta = vehState.spatial.theta;

        double F = L + LB1;
        double R = LB2;
        double HW = Width / 2;

        lf_x = x + F * cos(theta) - HW * sin(theta);
        lf_y = y + F * sin(theta) + HW * cos(theta);

        rf_x = x + F * cos(theta) + HW * sin(theta);
        rf_y = y + F * sin(theta) - HW * cos(theta);

        rb_x = x - R * cos(theta) + HW * sin(theta);
        rb_y = y - R * sin(theta) - HW * cos(theta);

        lb_x = x - R * cos(theta) - HW * sin(theta);
        lb_y = y - R * sin(theta) + HW * cos(theta);
    }

    MODEL_VEH_DYNAMICS() {
        this->L = 2.8067;                       // wheelBase,                        m
        this->LB1 = 1.0;                        // the length between the front axle and the front bumper, m
        this->LB2 = 1.0;                        // the length between the rear axle and the rear bumper, m
        this->L1 = 1.2916;                      // the length between the centroid and the front axle, m
        this->L2 = this->L-this->L1;                // the length between the centroid and the rear axle,     m
        this->Width = 2.0;                      // the width of the car,                            m
    }

    void stepSim(double cmdK, double cmdV, double dt) {
        // X(1): x
        // X(2): y
        // X(3): theta
        // X(4): v
        double Gss = 1.0;

        double X = vehState.spatial.x;
        double Y = vehState.spatial.y;
        double H = vehState.spatial.theta;
        double K = vehState.spatial.k;
        double V = vehState.temporal.v;

        double dX = V * cos(H);
        double dY = V * sin(H);
        double dH = Gss * V * K;

        X = X + dX * dt;
        Y = Y + dY * dt;
        H = H + dH * dt;
        K = cmdK;
        V = cmdV;

        vehState.spatial.x = X;
        vehState.spatial.y = Y;
        vehState.spatial.theta = H;
        vehState.spatial.k = K;
        vehState.temporal.v = V;
        vehState.temporal.a = 0;
    }

    void setVehicleState(State vs0) {
        this->vehState = vs0;
    }

};

struct Vehicle_State{
    double northing;			// state position (units: meters)
    double easting;			// state position (units: meters)
    double heading;			// state heading 3 o'clock CCW (units: radians)
};

typedef struct VehicleMeasurements{
  double length;
  double width;
} VehicleMeasurements;

typedef struct CCPoint{
  double x;
  double y;
} CCPoint;

enum corners{FL, FR, RL, RR};
typedef struct VehicleShape{
  GenericPoint corner[4];
} VehicleShape;





#endif
