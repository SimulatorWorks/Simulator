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
* @file         Def4Simulation.h
* @author       Tianyu Gu
* @date         06/13/2013
*/

#ifndef _Def4Simulation_
#define _Def4Simulation_

#include "Comm.h"
#include "Def4PlanTrajectory.h"

#define CENTERLINE_RESOLUTION   0.5

typedef enum {XML_Unknown, XML_VehicleState, XML_Lane, XML_Object} XML_ElementType;
typedef enum {XML_Obj_Static, XML_Obj_Car, XML_Obj_Bicyclist, XML_Obj_Pedestrian} XML_ObjectType;

struct VehicleState_Def
{
    double Latitude;
    double Longitude;
    double Heading;
    double Curvature;
    double Speed;
    double Acceleration;
};

struct Lane_Def
{
    string name;
    int numOfWaypoints;
    vector<pair<double, double> > waypoints;
    double width_m;
    double speedLim_mps;
};

struct Object_Def
{
    Object_Def() {
        triggerId_Sim = -1;
        triggerId_SwitchLane = -1;
    }

    string name;
    XML_ObjectType type;
    double length_m;
    double width_m;
    double radius_m;

    double Latitude;
    double Longitude;
    double Heading;
    double Speed;

    int triggerId_Sim;

    string laneTracking;
    string laneToTrack;
    int triggerId_SwitchLane;
};


// Vehicle Dimensions
#define VEH_LF  3.0
#define VEH_LB  2.0
#define VEH_LENGTH  (VEH_LF + VEH_LB)
#define VEH_WIDTH  2.0
#define VEH_WHEELBASE_F 2.0
#define VEH_WHEELBASE_B 1.5
#define VEH_REAR_AXLE_TO_TAIL   (VEH_LB - VEH_WHEELBASE_B)
#define VEH_WHEELBASE (VEH_WHEELBASE_F + VEH_WHEELBASE_B)
#define VEH_WIDTH_WHEELS 1.8
#define VEH_WHEEL_RADIUS 0.5


// For scrollingByteMap based collision checkers, need to expand ego vehicle instead of expanding obstacle
#define WIDTH_EXTENSION                         0.0
#define LENGTH_EXTENSION                        0.0
#define VEHICLE_WIDTH                           (1.91 + WIDTH_EXTENSION)
#define REAR_AXLE_TO_TAIL                       0.988
#define REAR_AXLE_TO_FRONT_BUMPER_LENGTH        (3.846 + LENGTH_EXTENSION)
#define VEHICLE_LENGTH                          (REAR_AXLE_TO_TAIL + REAR_AXLE_TO_FRONT_BUMPER_LENGTH)
#define SAFETY_SCALE                            1.0
//#define EVASIVE_SCALE                           1.2


class HostVehicle
{
    State  vs_;

    double m_speed_cmd_;
    double m_curvature_cmd_;
    double m_speed_;
    double m_curvature_;

    bool isAutonomous_;

public:
    HostVehicle();
    virtual ~HostVehicle();


    void forceSetVehicleState(double easting, double northing, double heading, double curv, double speed, double acc);
    inline void setAutonomous(bool isAutonomous) { isAutonomous_ = isAutonomous; }

    void setCommand(double speed, double curvature);
    void stepSim(double dt);

    inline bool getIsAutonomous() { return isAutonomous_; }
    inline State getVehicleState() { return vs_; }

    // visulization
private:
    QwtPlotCurve curveEdgeFront_;
    QwtPlotCurve curveEdgeBack_;
    QwtPlotCurve curveEdgeLeft_;
    QwtPlotCurve curveEdgeRight_;

    QwtPlotCurve curveAxleFront_;
    QwtPlotCurve curveAxleRear_;

    QwtPlotCurve curveWheelFL_;
    QwtPlotCurve curveWheelFR_;
    QwtPlotCurve curveWheelBL_;
    QwtPlotCurve curveWheelBR_;

public:
    void attach(QwtPlot * plot);
    void detach();
    void updateDrawings();
};


enum LanePieceType { NORMAL_LANE, VIRTUAL_LANE };
class LanePiece
{
public:
#ifdef FLAG_IN_TRUCS
    QVector<const roadWorldModel::Waypoint*> waypoints;
#else
    vector<pair<double,double> > waypointsLL;
    vector<pair<double,double> > waypointsNE;
#endif
    vector<double> s;
    vector<double> x;
    vector<double> y;
    vector<double> theta;
    vector<double> k;
    vector<double> dk;
    int numOfPoints;

    LanePieceType type;
    double laneWidth;
    double speedLimit;

    bool isValid;

public:
    void reset();

    bool generateReference( vector<pair<double, double> > waypointsLL, vector<pair<double, double> > waypointsNE, double laneWidth_m, double speedLimit_mps );
    bool interpretHeadingCurvature();

    int getNearestIndexBySpatialState(StateSpatial spatial);
    int getNearestIndexByCoordinates(double x, double y);
    double getAbsDistanceByCoordinates(double x, double y);
    double getSignedDistanceByCoordinates(double x, double y);

#ifndef FLAG_IN_TRUCS
private:
    QwtPlotCurve curveCL_;
    QwtPlotCurve curveLL_;
    QwtPlotCurve curveRL_;
public:
    void attach(QwtPlot * plot);
    void detach();
    void updateDrawings();
#endif
};

//struct GoalSegment
//{
//#ifdef FLAG_IN_TRUCS
//    MotionGoal goal;
//    QVector<roadWorldModel::WorldId> laneIds;
//    const roadWorldModel::Waypoint* entryWaypoint;
//    const roadWorldModel::Waypoint* exitWaypoint;
//#endif

//    vector<LanePiece> lanePieces;
//    bool hasLaneChange;
//    bool hasStopLine;

//    void reset() {
//#ifdef FLAG_IN_TRUCS
//        laneIds.clear();
//#endif
//        lanePieces.clear();
//    }
//};

//struct RoadGroundTruth
//{
//    vector<GoalSegment> goalSegments;
//    uint indexOfCurrSeg;

//    int getIndexOfNearestLaneInOneGoalSegment(int goalSegmentId, double startX, double startY);

//    void reset() {
//        goalSegments.clear();
//    }
//};


class OBJECT {
protected:
    string name;

    GenericPose pose;
    double speed_mps;
    double acc_mpss;
    bool* isSimTriggered;

public:
    // Initialize
    OBJECT();

    // Info set
    inline void setName(string s) { this->name = s; }
    inline void setPose(double x, double y, double theta) {
        this->pose.x = x;
        this->pose.y = y;
        this->pose.theta = theta;
    }
    inline void setSpeed(double v) { this->speed_mps = v; }
    inline void setAcc(double a) { this->acc_mpss = a; }
    inline void setIsSimTriggered(bool* trigger) { this->isSimTriggered = trigger; }

    // Info retrieve
    inline GenericPose getPose() { return this->pose; }
    inline double getSpeed() { return this->speed_mps; }
    inline double getAcc() { return this->acc_mpss; }

};


class OBJECT_STATIC : public OBJECT
{
    // shape
    double length_m;       // length of bounding box, along direction of heading
    double width_m;        // width of bounding box, perpendicular to direction of heading
public:
    // Initialize
    OBJECT_STATIC():OBJECT() {}

    // Info set
    inline void setLength(double l) { this->length_m = l; }
    inline void setWidth(double w) { this->width_m = w; }

    // Info retrieve
    inline double getLength() { return this->length_m; }
    inline double getWidth() { return this->width_m; }

    //visualization
private:
    QwtPlotCurve curveEdgeFront_;
    QwtPlotCurve curveEdgeBack_;
    QwtPlotCurve curveEdgeLeft_;
    QwtPlotCurve curveEdgeRight_;
public:
    void attach(QwtPlot* plot);
    void detach();
    void updateDrawings();
};

class OBJECT_DYNAMIC_CIRCLE : public OBJECT
{
    double radius_m;
public:
    // Initialize
    OBJECT_DYNAMIC_CIRCLE() {}

    // Info set
    inline void setRadius(double r) { this->radius_m = r; }

    // Info retrieve
    inline double getRadius() { return radius_m; }
    std::pair<double, double> getFuturePosition(double time);

    // Behaviors
    void stepSim_linear(double time);

private:
    QwtPlotCurve curveEdge_;
public:
    void attach(QwtPlot* plot);
    void detach();
    void updateDrawings();
};

class OBJECT_DYNAMIC_RECTANGLE : public OBJECT
{
    double length_m;
    double width_m;

    bool hasPath;
    Path path;
    LanePiece* laneTracking;
    LanePiece* laneToTrack;

    bool* isSwitchTrackingLaneTriggered;

public:
    // Initialize
    OBJECT_DYNAMIC_RECTANGLE();

    // Info set
    inline void setLength(double l) { this->length_m = l; }
    inline void setWidth(double w) { this->width_m = w; }
    inline void setLaneTracking(LanePiece* cl) { laneTracking = cl; }
    inline void setLaneToTrack(LanePiece* cl) { laneToTrack = cl; }
    inline void setIsSwitchTrackingLaneTriggered(bool* trigger) { this->isSwitchTrackingLaneTriggered = trigger; }

    // Info retrieve
    inline double getLength() { return this->length_m; }
    inline double getWidth() { return this->width_m; }
    inline bool isHasPath() { return this->hasPath; }
    inline Path getPath() { return this->path; }
    StateSpatial predictPosHeading_Linear(double time);

    // Behaviors
    void checkSwitchLaneTrigger();

    void stepSim_linear(double time);
    void stepSim_tracking(double time);
    void stepSim_tracking_dk(double time, vector<OBJECT_DYNAMIC_RECTANGLE*> objs, HostVehicle* hv);

    //visualization
private:
    QwtPlotCurve curveEdgeFront_;
    QwtPlotCurve curveEdgeBack_;
    QwtPlotCurve curveEdgeLeft_;
    QwtPlotCurve curveEdgeRight_;
public:
    void attach(QwtPlot* plot);
    void detach();
    void updateDrawings();
};

#endif
