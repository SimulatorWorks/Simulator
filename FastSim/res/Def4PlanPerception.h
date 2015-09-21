/**
* @file         Def4PlanPerception.h
* @author       Tianyu Gu
* @date         04/24/2014
*
* @attention    Copyright (C) 2014
* @attention    Carnegie Mellon University
* @attention    All rights reserved
*
*/

#ifndef Def4PlanPerception_H
#define Def4PlanPerception_H

#include "Comm.h"

// Static Object Convolver
struct PH_IndexRange
{
    long rs, re; ///<row index ranges, should be considered rs<=range<re
    long cs, ce; ///<column index ranges, should be considered cs<=range<ce
    void normalize(){
        if (rs > re) {
            int tmp = rs;
            rs = re;
            re = tmp;
        }
        if (cs > ce) {
            int tmp = cs;
            cs = ce;
            ce = tmp;
        }
    }
};


struct PH_POINT_2D
{
    double x;
    double y;

private:
#ifdef FLAG_IN_TRUCS
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned int version){
        ar & x;
        ar & y;
    }
#endif
};


template <class T>
struct PH_MAP
{
    PH_MAP(){
        this->lb.x = 0;
        this->lb.y = 0;
        this->rt.x = 0;
        this->rt.y = 0;

        this->cells.empty();

        this->colDim = 0;
        this->rowDim = 0;
    }
    PH_MAP(double mapSize_m, double cellSize_m, uint col, uint row) {
        this->lb.x = 0;
        this->lb.y = 0;
        this->rt.x = 0;
        this->rt.y = 0;

        this->cells.resize( col * row, 0 );

        this->mapSize_m = mapSize_m;
        this->cellSize_m = cellSize_m;

        this->colDim = col;
        this->rowDim = row;
    }

    ~PH_MAP() {;}

    PH_MAP& operator=(PH_MAP& rhs){
        this->lb = rhs.lb;
        this->rt = rhs.rt;

        this->mapSize_m = rhs.mapSize_m;
        this->cellSize_m = rhs.cellSize_m;

        this->colDim = rhs.colDim;
        this->rowDim = rhs.rowDim;

        this->cells = rhs.cells;

        return *this;
    }

    std::vector< std::pair<int, int> > lineTrace(GenericPoint pt0, GenericPoint ptf) {
        int x0 = (int)floor((pt0.x - this->lb.x)/this->cellSize_m);
        int y0 = (int)floor((pt0.y - this->lb.y)/this->cellSize_m);
        int x1 = (int)floor((ptf.x - this->lb.x)/this->cellSize_m);
        int y1 = (int)floor((ptf.y - this->lb.y)/this->cellSize_m);

        int dx=x1-x0;
        int dy=y1-y0;

        int octant = 0;
        if(dx >= 0 && dy >= 0) {
            if(fabs(dx) >= fabs(dy))
                octant = 0;
            else
                octant = 1;
        }
        else if (dx < 0 && dy >= 0) {
            if(fabs(dx) >= fabs(dy))
                octant = 3;
            else
                octant = 2;
        }
        else if (dx < 0 && dy < 0) {
            if(fabs(dx) >= fabs(dy))
                octant = 4;
            else
                octant = 5;
        }
        else if (dx >= 0 && dy < 0) {
            if(fabs(dx) >= fabs(dy))
                octant = 7;
            else
                octant = 6;
        }

        // determine input octant
        int xz, yz, xf, yf;
        switch(octant) {
             case 0:
                 xz = x0; yz = y0;
                 xf = x1; yf = y1;
                 break;
             case 1:
                 xz = y0; yz = x0;
                 xf = y1; yf = x1;
                 break;
             case 2:
                 xz = y0; yz = -x0;
                 xf = y1; yf = -x1;
                 break;
             case 3:
                 xz = -x0; yz = y0;
                 xf = -x1; yf = y1;
                 break;
             case 4:
                 xz = -x0; yz = -y0;
                 xf = -x1; yf = -y1;
                 break;
             case 5:
                 xz = -y0; yz = -x0;
                 xf = -y1; yf = -x1;
                 break;
             case 6:
                 xz = -y0; yz = x0;
                 xf = -y1; yf = x1;
                 break;
             case 7:
                 xz = x0; yz = -y0;
                 xf = x1; yf = -y1;
        }

        int Dx = xf-xz;
        int Dy = yf-yz;

        // convert input to zero octant
        std::vector<int> Xz; Xz.clear();
        std::vector<int> Yz; Yz.clear();

        int D = 2*Dy - Dx;

        Xz.push_back( xz ); Yz.push_back( yz );

        int y=yz;
        for (int x = xz+1; x <= xf; x++) {
            if (D > 0) {
                y = y+1;
                Xz.push_back(x); Yz.push_back(y);
                D = D + (2*Dy-2*Dx);
            }
            else {
                Xz.push_back(x); Yz.push_back(y);
                D = D + (2*Dy);
            }
        }

        // convert back to actual octant
        std::vector<int> X; X.clear();
        std::vector<int> Y; Y.clear();
        switch(octant) {
             case 0:
                 X = Xz; Y = Yz;
                 break;
             case 1:
                 X = Yz; Y = Xz;
                 break;
             case 2:
                 X = negateVector(Yz); Y = Xz;
                 break;
             case 3:
                 X = negateVector(Xz); Y = Yz;
                 break;
             case 4:
                 X = negateVector(Xz); Y = negateVector(Yz);
                 break;
             case 5:
                 X = negateVector(Yz); Y = negateVector(Xz);
                 break;
             case 6:
                 X = Yz; Y = negateVector(Xz);
                 break;
             case 7:
                 X = Xz; Y = negateVector(Yz);
        }

        std::vector<std::pair<int, int> > list;
        for(int i=0; i<X.size(); i++) {
            std::pair<int, int> tmp(X[i], Y[i]);
            list.push_back(tmp);
        }

        return list;
    }
        std::vector<int> negateVector(std::vector<int> in) {
            std::vector<int> out = in;
            for(int i=0; i<out.size(); i++) {
                out[i] = -out[i];
            }

            return out;
        }

    PH_POINT_2D lb;
    PH_POINT_2D rt;

    double mapSize_m; // A square, vertical and perpendicular of same length
    double cellSize_m; // A square, vertical and perpendicular of same cell sizes

    uint colDim;
    uint rowDim;

    std::vector<T> cells;

private:
#ifdef FLAG_IN_TRUCS
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned int version){
        ar & lb;
        ar & rt;

        ar & mapSize_m & cellSize_m;
        ar & colDim & rowDim;

        ar & cells;
    }
#endif
};

typedef PH_MAP<unsigned char> PH_MAP_UCHAR;
typedef PH_MAP<float> PH_MAP_FLOAT;

template <class T>
struct COLLECTION_OF_PH_MAP
{
    ~COLLECTION_OF_PH_MAP(){
        for(uint i=0; i<this->maps.size();i++){
            delete this->maps[i].second;
        }
    }

    std::vector< std::pair<double, PH_MAP<T>*> > maps; // a vector of pairs of (vehicle orientation and map)
};



enum PH_OBJECT_TYPE {PH_GENERAL, PH_STATIC, PH_PEDESTRIAN, PH_BICYCLIST, PH_VEHICLE};

struct PH_COVERCIRCLE
{
    double x;
    double y;
    double r;
};

struct PH_OBJECT
{
    // Functions
    PH_OBJECT() {}

    PH_OBJECT(PH_OBJECT_TYPE type, double length, double width, int numOfCoverCircles,
              GenericPose pose0, double v0, bool isMove) {
        this->objType = type;
        this->length_m = length;
        this->width_m = width;
        this->numOfCoverCircles = numOfCoverCircles;

        this->pose = pose0;
        this->speedLon_mps = v0;
        this->isMoving = isMove;

        this->hasTraj = false;
    }

    PH_OBJECT(PH_OBJECT_TYPE type, double length, double width, int numOfCoverCircles,
              GenericPose pose0, double v0, bool isMove,
              bool hasTraj,
              vector<double> trajS, vector<double> trajX, vector<double> trajY, vector<double> trajH) {
        this->objType = type;
        this->length_m = length;
        this->width_m = width;
        this->numOfCoverCircles = numOfCoverCircles;

        this->pose = pose0;
        this->speedLon_mps = v0;
        this->isMoving = isMove;

        this->hasTraj = hasTraj;

        this->trajS = trajS;
        this->trajX = trajX;
        this->trajY = trajY;
        this->trajH = trajH;
    }

    inline void setPose(GenericPose pose) { this->pose = pose; }
    inline void setSpeed(double val) { speedLon_mps = val; }

    void stepSim(double dt_s); // Simulate the object
    GenericPose getPoseProjectionByTime(double dt_s);
    vector<PH_COVERCIRCLE> getCoverCircleByPose(GenericPose pose);


    // Member variables
    PH_OBJECT_TYPE objType;
    double length_m;
    double width_m;
    int numOfCoverCircles;

    GenericPose pose;
    double speedLon_mps;

    // Preset Trajectory
    bool hasTraj;
//    vector<double> wpX;
//    vector<double> wpY;

    vector<double> trajS;
    vector<double> trajX;
    vector<double> trajY;
    vector<double> trajH;

    bool isMoving;

#ifdef FLAG_IN_TRUCS
    template <class Archive>
    void serialize(Archive &ar, unsigned int version)
    {
        ar & objType;
        ar & length_m;
        ar & width_m;

        ar & pose;
        ar & speedLon_mps;

        ar & hasTraj;
//        ar & wpX & wpY;
        ar & trajS & trajX & trajY & trajH;

        ar & isMoving;
    }
#endif
};




// Road blockage detection
//typedef std::map<RoadBlockage,bool> BlockageDisableMap;

//struct PersistentState
//{
//    BlockedLinks blockedLinks;
//    roadWorldModel::RoadLinkUpdate::VersionBundleVector versionBundles;
//    BlockageDisableMap allPlannerBlockages;

//    template<class Archive>
//    void serialize( Archive& ar, unsigned int version )
//    {
//        ar
//            & blockedLinks & versionBundles
//            ;
//        if( version > 0 )
//        {
//            ar & allPlannerBlockages;
//        }
//    }
//};


struct seg
{
    static bool between( double a1, double b1, double b2 )
    {
        if( b1 > b2 ) std::swap(b1,b2);
        return (a1 >= b1 && a1 <= b2);
    }

    seg( double x1__, double x2__) : x1(x1__), x2(x2__)
    {
        if( x1 > x2 ) std::swap( x2, x1 );
    }

    double x1, x2;

    seg dilate(double x)
    {
        return seg( x1-x, x2+x );
    }

    bool contains( double x )
    {
        return between( x, x1, x2 );
    }

    bool intersects( seg s )
    {
        return contains( s.x1 ) || contains( s.x2 )
            || s.contains( x1 ) || s.contains( x2 )
            ;
    }

    bool combine( seg s )
    {
        if( intersects( s ) )
        {
            x1 = std::min( std::min( x1, s.x1 ), std::min( x2, s.x2 ) );
            x2 = std::max( std::max( x1, s.x1 ), std::max( x2, s.x2 ) );
            return true;
        }
        return false;
    }

};

///**
// * HystereticBlockage keeps track of road blockage persistence so that
// * we can publish RoadLinkUpdates on blockages that are actually really
// * there. The instantaneous RoadBlockageSet is still published for other
// * purposes, however.
// */
//class HystereticBlockage
//{
//public:
//    HystereticBlockage( );

//    void update( const RoadBlockageSet& blockages,
//                 boost::posix_time::ptime currTime,
//                 logger::Logger& logger );

//    void getPersistentBlockages( RoadBlockageSet& rbs,
//                                 int hysteresis_ms,
//                                 boost::posix_time::ptime currTime,
//                                 logger::Logger& logger ) const;

//private:
//    bool doBlockagesIntersect( const RoadBlockage &rb1,
//                               const RoadBlockage &rb2 ) const;

//    typedef std::map<RoadBlockage,boost::posix_time::ptime> BlockageAgeMap;
//    BlockageAgeMap persistentBlockages_;


//};


//struct PH_ROAD_BLOCKAGE
//{
//    PH_OBJECT_TYPE objType;

//    enum SIDE_TYPE {ST_FULL, ST_LEFT, ST_RIGHT};
//    SIDE_TYPE sideType;

//    roadWorldModel::WorldId laneId; // The index of the lane, which this blockage belongs to
//    double s0_m; // The projected start station on the lane
//    double sf_m; // The projected end station on the lane
//    double perc; // The side of the blockage in terms of the lateral occupation of the lane width
//    double vLon_mps;
//    double vLat_mps;

//    void checkInZones(roadWorldModel::RoadWorldModel* rwm, double x, double y, bool& isInBufferZone, bool& isInFatalZone);

//    static std::pair<double, double> getSLfromXYbyRoadLane(roadWorldModel::RoadLane* rl, double x, double y);

//    template <class Archive>
//    void serialize(Archive &ar, unsigned int version)
//    {
//        ar & objType;
//        ar & sideType;

//        ar & laneId;
//        ar & s0_m & sf_m;
//        ar & perc;
//        ar & vLon_mps;
//        ar & vLat_mps;
//    }
//};

//struct PH_ROAD_BLOCKAGE_RAY
//{
//    double station_m;

//    std::pair<double, double> pt0;
//    std::pair<double, double> ptf;
//    bool foundObstacle;

//    template <class Archive>
//    void serialize(Archive &ar, unsigned int version)
//    {
//        ar & station_m;
//        ar & pt0 & ptf;
//        ar & foundObstacle;
//    }
//};

//struct PH_ROAD_BLOCKAGE_RAY_BENT
//{
//    double latitude_m;

//    std::vector<std::pair<double, double> > ptList;
//    bool foundObstacle;

//    template <class Archive>
//    void serialize(Archive &ar, unsigned int version)
//    {
//        ar & latitude_m;
//        ar & ptList;
//        ar & foundObstacle;
//    }
//};


#endif
