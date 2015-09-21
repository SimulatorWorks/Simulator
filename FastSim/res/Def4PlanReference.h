/**
* @file         Def4PlanReference.h
* @author       Tianyu Gu
* @date         03/17/2013
*
* @attention    Copyright (C) 2013
* @attention    Carnegie Mellon University
* @attention    All rights reserved
*
*/

#ifndef _Def4PlanReference_
#define _Def4PlanReference_

#include "Comm.h"
#include "Def4PlanPerception.h"
#include "Def4Environment.h"

#define REFERENCE_RESOLUTION    0.5

using namespace std;

/************************************* Traffic-free Reference Planning ****************************************/

enum ReferenceType { InLane, InLane_TakeOver, LaneChange, LaneChange_LastMinute, LaneChange_Abort, VirtualLaneMild, VirtualLaneTight };

struct ReferenceRaw
{
    ReferenceRaw();
    void reset();
    // Data
    std::vector<double> rB;
    std::vector<double> lB;
    std::vector<double> s;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> k;
    std::vector<double> v_L; // To-do: change to std::Vector, just use v_L
    std::vector<ReferenceType> type;
    std::vector<int> segmentId;
    int currSegId;
    int numOfPoints;

#ifdef FLAG_IN_TRUCS
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & rB & lB;
        ar & s & x & y & theta & k;
        ar & v_L;
        ar & type;
        ar & segmentId;
        ar & currSegId;
        ar & numOfPoints;
    }
#endif
};

struct Reference
{
    Reference();
    void reset();
    void push(double rB,
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
              int optFlag );
    // Data
    std::vector<double> rB;
    std::vector<double> lB;
    std::vector<double> s;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> k;
    std::vector<double> vLim; // speed limit of the segment
    std::vector<double> vMax; // max speed, implied by traffic-free plan mostly to deal with stopping and speed limit change, etc.
    std::vector<double> v;    // speed plan latest...
    std::vector<double> tEst; // calculated based on vPll
    std::vector<ReferenceType> type;
    std::vector<int> segmentId;
    std::vector<int> optFlag; // 0: no opt; 1: potential opt; 2: get opt
    int currSegId;
    int numOfPoints;

    bool blocked;

    // Self-maintenance
    void recalculateStation();
    bool recalculateStationHeadingCurvature();
    bool recalculateEstimatedTime();

    int getNearestIndexByCoordinates( double x, double y );
    int getNearestIndexByStation( double station );
    int getNearestIndexByTime(double time);

    double getStationByCoordinates( double x, double y );
    double getStationByCoordinates( double x, double y, int idx0, int idxf );

    double getSignedDistanceByCoordinates( double x, double y );
    double getAbsDistanceByCoordinates( double x, double y );


    void copyFrom(Reference& from);
    void copyFrom(LanePiece& lane);


#ifdef FLAG_IN_TRUCS
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned int version){
        ar & rB & lB;
        ar & s & x & y & theta & k;
        ar & vMax & vLim & v;
        ar & tEst;
        ar & type;
        ar & segmentId;
        ar & optFlag;
        ar & currSegId;
        ar & numOfPoints;

        ar & blocked;
    }
#endif
};


struct RefHistory
{
    RefHistory() {
        reset();
    }
    // Data
    void reset() {
        s.clear(); x.clear(); y.clear(); theta.clear(); k.clear(); v.clear(); t.clear();
        numOfPoints = 0;
    }
    std::vector<double> s;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> k;
    std::vector<double> v;
    std::vector<double> t;
    int numOfPoints;

#ifdef FLAG_IN_TRUCS
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned int version){
        ar & s & x & y & theta & k;
        ar & v & t;
        ar & numOfPoints;
    }
#endif
};


//************************************************************************************

struct REFERENCE_UNFILTERED
{
    std::vector<double> s;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> v;

    int numOfPoints;

    void reset()
    {
        s.clear();
        x.clear();
        y.clear();
        theta.clear();
        v.clear();

        numOfPoints = 0;
    }

    int getNearestIndexByCoordinates(double x, double y);
    double getSignedDistanceByCoordinates(double x, double y);
    void recalculateStation();

    void conditioning();
    void reinterpolate(double ds);

#ifdef FLAG_IN_TRUCS
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & s & x & y & theta;
        ar & v;
        ar & numOfPoints;
    }
#endif
};



/************************************* Learning for Planners ****************************************/
struct REFERENCE_TRAIN
{
    REFERENCE_TRAIN() {
        numOfPoints = 0;
    }

    // Data
    std::vector<double> s;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> k;
    std::vector<double> v;


    std::vector<double> xProj;
    std::vector<double> yProj;
    std::vector<double> kProj;
    std::vector<double> vProj;

    int numOfPoints;

    double cumPathError;
    double cumSpeedError;

    void reset()
    {
        s.clear();
        x.clear();
        y.clear();
        theta.clear();
        k.clear();
        v.clear();

        xProj.clear();
        yProj.clear();
        kProj.clear();
        vProj.clear();

        numOfPoints = 0;
    }

    bool projectPlanOntoDemo(Reference& ref);

    bool projectPlanOntoDemoCL(Reference& cl, Reference& ref);

private:

#ifdef FLAG_IN_TRUCS
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned int version) {
        ar & s & x & y & theta & k & v;
        ar & xProj & yProj & kProj & vProj;
        ar & numOfPoints;

        ar & cumPathError & cumSpeedError;
    }
#endif
};


#endif // _Def4PlanReference_
