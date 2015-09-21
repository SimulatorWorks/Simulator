/**
* @file         Def4PlanTrajectory.h
* @author       Tianyu Gu
* @date         03/17/2013
*
* @attention    Copyright (C) 2013
* @attention    Carnegie Mellon University
* @attention    All rights reserved
*
*/

#ifndef _Def4PlanTrajectory_
#define _Def4PlanTrajectory_

#include "Comm.h"
#include "CommFunctions.h"

#define POINTNUM_INTERNAL 200
#define POINTNUM 50


struct Path
{
    std::vector<double> s;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> k;
    std::vector<double> dk;
    std::vector<double> ddk;
    int numOfPoints;
    bool isValid;

    void clear();
    void resize(int pointNum);

    int getProjIdxByS(double s);
};

struct Velocity
{
    std::vector<double> t;
    std::vector<double> s;
    std::vector<double> v;
    std::vector<double> a;
    int numOfPoints;
    bool isValid;

    void clear();
    void resize(int pointNum);
};

// PATH_PARAMETER_ARC2
// Initial: (s0.x, s0.y, s0.theta, s0.k)
// Model: k(s) = p0 + p1*s + p2*s^2
// Known parameter: p0 = s0.k;
// Unknown parameter: sf, p1, p2
// Constraints: sf.x, sf.y, sf.theta

// PATH_PARAMETER_ARC3
// Initial: (s0.x, s0.y, s0.theta, s0.k)
// Model: k(s) = p0 + p1*s + p2*s^2 + p3*s^3
// Known parameter: p0 = s0.k;
// Unknown parameter: sf, p1, p2, p3
// Constraints: sf.x, sf.y, sf.theta, sf.k


// PATH_PARAMETER_ARC3_RELAX
// Initial: (s0.x, s0.y, s0.theta, s0.k, s0.dk)
// Model: k(s) = p0 + p1*s + p2*s^2 + p3*s^3
// Known parameter: p0 = s0.k, p0 = s0.dk;
// Unknown parameter: sf, p2, p3
// Constraints: sf.x, sf.y, sf.theta



// PATH_PARAMETER_ARC4
// Initial: (s0.x, s0.y, s0.theta, s0.k, s0.dk)
// Model: k(s) = p0 + p1*s + p2*s^2 + p3*s^3 + p4*s^4
// Known parameter: p0 = s0.k   p1 = s0.dk
// Unknown parameter: sf, p2, p3, p4
// Constraints: sf.x, sf.y, sf.theta, sf.k



struct PATH_PARAMETER_XY5
{
//    PATH_PARAMETER_XY5();

    StateSpatial s0;
    StateSpatial sf;
    double p[6];
    double length;
    bool isValid;


    void reset();

    std::pair<double,double> getXYByS(double s);
    double getBiasThetaByS(double s);
    double getCurvatureByS(double s);
    double getDCurvatureByS(double s);
    double getDDCurvatureByS(double s);

    void findPathParameter(StateSpatial startState, StateSpatial endState);

    Path plotPath(int pointNum);


    // Only for this particular path primitive
    double xf;
    Path helpingPath; // Built when parameter founds

    int getIdOnHelpingPathByS(double s);


#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & s0;
        ar & sf;
        ar & p[0] & p[1] & p[2] & p[3] & p[4] & p[5];
        ar & length;
        ar & isValid;
    }
#endif
};


struct PathParameterArc
{
    StateSpatial s0;
    StateSpatial sf;

    double p[6]; //w.r.t arc-length
    double length;

    int order;
    bool isValid;

    void reset();

    void findPathParameter(StateSpatial startState, StateSpatial endState, int order);
        void initParameter(StateSpatial startState, StateSpatial transformedState);
        StateSpatial calculateState();
        void stepOptimize(StateSpatial tempX, StateSpatial transformEndState);
            void calculateJacobi(double *jacobi);
        bool isTermination(StateSpatial currTransformedEndState, StateSpatial goalTransformedEndState);

        std::pair<double,double> getXYByS(double s);
        std::pair<double,double> getXYByS_Cheap(double s);
        double getBiasThetaByS(double s);
        double getCurvatureByS(double s);
        double getDCurvatureByS(double s);
        double getDDCurvatureByS(double s);


    Path plotPath(int pointNum);

    Path helpingPath; // Built when parameter founds
    int getIdOnHelpingPathByS(double s);

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & s0;
        ar & sf;
        ar & p[0] & p[1] & p[2] & p[3] & p[4] & p[5];
        ar & length;
        ar & order;
        ar & isValid;
    }
#endif
};

//struct PATH_PARAMETER_TIME
//{
//    StateSpatial s0;

//    double p[6]; //w.r.t time
//    double duration;

//    int order;
//    bool isValid;

//    void reset();

//    std::pair<double,double> getXYByT(double t);
//    double getBiasThetaByT(double t);
//    double getCurvatureByT(double t);
//    double getDCurvatureByT(double t);
//    double getDDCurvatureByT(double t);


//    PATH plotPath(int pointNum);

//#ifdef FLAG_IN_TRUCS
//    template <class Archive>
//    void serialize(Archive &ar, unsigned int version)
//    {
//        ar & s0;
//        ar & p[0] & p[1] & p[2] & p[3] & p[4] & p[5];
//        ar & duration;
//        ar & order;
//        ar & isValid;
//    }
//#endif
//};


struct VelocityParameterTime
{
    StateTemporal s0;
    StateTemporal sf;

    double p[5]; //w.r.t time
    double duration;

    int order;
    bool isValid;

    void reset();

    void findVelocityParameter(StateTemporal startState, StateTemporal endState, PathParameterArc* pathPar, int order);
    void findVelocityParameter(StateTemporal startState, double acc, PathParameterArc* pathPar);

    double getSByT(double t);
    double getVByT(double t);
    double getAByT(double t);

    Velocity plotVelocity(int pointNum);


#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & s0;
        ar & sf;
        ar & p[0] & p[1] & p[2] & p[3] & p[4];
        ar & duration;
        ar & order;
        ar & isValid;
    }
#endif
};


#define PHI_LIMIT   GSL_POSINF
#define DPHI_LIMIT  GSL_POSINF
#define DDPHI_LIMIT GSL_POSINF

struct Trajectory
{
    // Should NOT have timeOffset, since it will explode the number of trajectory.
    // Should generate trajectory as base, then use pointers to trajectory and timeOffset pairs to describe a full trajectory
//    double timeOffset_s;

    PathParameterArc pathPar;
    VelocityParameterTime velPar;

    double timeDuration_s;
    std::vector<double> t;

    std::vector<double> s_t;
    std::vector<double> v_t;
    std::vector<double> a_t;

    std::vector<double> x_t;
    std::vector<double> y_t;
    std::vector<double> theta_t;
    std::vector<double> k_t;
    std::vector<double> dk_t;
    int numOfPoints;

    double cost;
    bool isValid;


    void reset();
    void clearPoints(int num);

    void generateOneTrajectory(State startState, State endState, int pathOrder, int velOrder, int pointNum);
    void generateOneTrajectoryConstAcc(State startState, State endState, double acc, int pathOrder, int pointNum);
    void generateOneTrajectoryConstDT(State startState, State endState, int pathOrder, int velOrder, double dt);
    void generateOneTrajectoryStraightLine(State startState, State endState, int pointNum);

    void setTrajectoryParameters(PathParameterArc pathPar, VelocityParameterTime velPar);

    void evaluateTrajectoryGivenPointNum(int pointNum);
    void evaluateTrajectoryGivenDeltaTime(double dt);

    int getProjIdxByT(double t);
    int getProjIdxByS(double s);
    int getProjIdxByXY(double x, double y);

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & pathPar & velPar;
    }
#endif

};


/* Trajectory in Control Space */
struct CTRLTRAJ
{
    CTRLTRAJ()
    {
        numOfPoints = 0;
    }

    State s0;
    double dkdt;
    double dvdt;
    double totalT;

    std::vector<double> t;

    std::vector<double> s_t;
    std::vector<double> v_t;

    std::vector<double> x_t;
    std::vector<double> y_t;
    std::vector<double> theta_t;
    std::vector<double> k_t;
    int numOfPoints;

    void reset();
    void clearPoints(int num);
    void generateOneTrajectory(State startState, double dkdt, double dvdt, double totalT);
        double thetaFunctionByT(double k0, double dkdt, double tt);
        double curvatureFunctionByT(double k0, double dkdt, double tt);

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & s0 & dkdt & dvdt & totalT;
//        ar & t & s_t & v_t & a_t & x_t & y_t & theta_t & k_t & numOfPoints;
        ar & x_t & y_t;
        ar & numOfPoints;
    }
#endif
};


struct TRAJSENTDOWN
{
    PathParameterArc pathParameter;
    VelocityParameterTime velocityParameter;

    // Discretized
    std::vector<double> S;
    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> H;
    std::vector<double> K;
    std::vector<double> DK;
    std::vector<double> V;

    int numOfPoints;

    // Misc
    bool isValid;

    void reset();
};

#define TRAJPLAN_TRAJECTORY_DT  0.1
struct trajplan_trajectory_plus
{
    trajplan_trajectory_plus() { this->reset(); }

    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> H;
    std::vector<double> K;
    std::vector<double> DK;
    std::vector<double> V;

    int numOfPoints;

    void reset();

    double getProjectedSignedDistanceByCoordinates(double x, double y);
    State getProjectedTrajectoryStateByCoordinates(double x, double y);
};


#endif // _Def4PlanTrajectory_
