#ifndef WorldModel_H
#define WorldModel_H

#include <res/Comm.h>
#include <res/CommFunctions.h>
#include <res/Def4PlanTrajectory.h>
#include <res/Def4Environment.h>

#include "PathTracker.h"

class WorldModel
{
public:
    WorldModel();
    ~WorldModel();

public:
    void initVehicleState_NE(double easting, double northing, double heading, double curv, double speed, double acc);

    void initVehicleState_Def( VehicleState_Def def );
    void initRoadModel_Def( vector<Lane_Def> defs );
    void initObject_Def( vector<Object_Def> defs,
                         bool* trigger0, bool* trigger1, bool* trigger2 );

    inline void getLanes_SELF( vector<LanePiece*>& lanes ) { lanes = lanes_; }
    void getObjects_SELF(vector<OBJECT_STATIC*>& objsStatic,
                         vector<OBJECT_DYNAMIC_CIRCLE*>& objsDynamic_Ped,
                         vector<OBJECT_DYNAMIC_RECTANGLE*>& objsDynamic_Veh,
                         vector<OBJECT_DYNAMIC_RECTANGLE*>& objsDynamic_Bike);
    void getHostState_SELF(State& vehState, bool& isAutonomous);


    inline vector<LanePiece*> getRoadModel4Display() { return lanes_; }
    inline vector<OBJECT_STATIC*> getObjectsStatic4Display() { return objsStatic_; }
    inline vector<OBJECT_DYNAMIC_CIRCLE*> getObjectsDynamicPed4Display() { return objsDynamic_Ped_; }
    inline vector<OBJECT_DYNAMIC_RECTANGLE*> getObjectsDynamicVeh4Display() { return objsDynamic_Veh_; }
    inline vector<OBJECT_DYNAMIC_RECTANGLE*> getObjectsDynamicBike4Display() { return objsDynamic_Bike_; }

public:

    // World Model
    vector<LanePiece*> lanes_;

    // Vehicle Model, including rear diff states (Easting / Northing, subject to which trans function used)
    HostVehicle* hv_;

    // Static Objects (Easting / Northing, subject to which trans function used)
    vector<OBJECT_STATIC*> objsStatic_;
    vector<OBJECT_DYNAMIC_CIRCLE*> objsDynamic_Ped_;
    vector<OBJECT_DYNAMIC_RECTANGLE*> objsDynamic_Veh_;
    vector<OBJECT_DYNAMIC_RECTANGLE*> objsDynamic_Bike_;


private:
    LanePiece* createNewLanePiece( Lane_Def def );
};

#endif // WorldModel_H
