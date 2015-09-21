#include "WorldModel.h"

WorldModel::WorldModel()
{
    lanes_.clear();
    objsStatic_.clear();
    objsDynamic_Ped_.clear();
    objsDynamic_Veh_.clear();
    objsDynamic_Bike_.clear();
}

WorldModel::~WorldModel()
{

}

void WorldModel::initVehicleState_NE(double easting, double northing, double heading, double curv, double speed, double acc)
{
    hv_->forceSetVehicleState(easting, northing, heading, curv, speed, acc);
}

void WorldModel::initVehicleState_Def( VehicleState_Def def )
{
    hv_ = new HostVehicle();

    double lat = def.Latitude;
    double lon = def.Longitude;
    double heading = def.Heading;
    double curv = def.Curvature;
    double speed = def.Speed;
    double acc = def.Acceleration;

    double easting_m, northing_m;
    LatLon2NE(lat, lon, &easting_m, &northing_m);

    hv_->forceSetVehicleState(easting_m, northing_m, heading, curv, speed, acc);
}

void WorldModel::initRoadModel_Def( vector<Lane_Def> defs )
{
    lanes_.clear();

    for(int i=0; i< defs.size(); i++) {
        LanePiece* lanepiece = createNewLanePiece( defs[i] );
        lanes_.push_back( lanepiece );
    }
}

void WorldModel::initObject_Def( vector<Object_Def> defs,
                                 bool* trigger0, bool* trigger1, bool* trigger2 )
{
    objsStatic_.clear();
    objsDynamic_Ped_.clear();
    objsDynamic_Veh_.clear();
    objsDynamic_Bike_.clear();

    for(int i=0; i<defs.size(); i++) {
        Object_Def& def = defs[i];

        if( def.type == XML_Obj_Static ) {
            OBJECT_STATIC* objStatic = new OBJECT_STATIC();

            objStatic->setName( def.name );

            objStatic->setLength( def.length_m );
            objStatic->setWidth( def.width_m );

            double easting_m, northing_m;
            LatLon2NE(def.Latitude, def.Longitude, &easting_m, &northing_m);
            objStatic->setPose( easting_m, northing_m, def.Heading);

            objsStatic_.push_back( objStatic );
        }
        else if( def.type == XML_Obj_Pedestrian ) {
            OBJECT_DYNAMIC_CIRCLE* objDynamic_Ped = new OBJECT_DYNAMIC_CIRCLE();

            objDynamic_Ped->setName( def.name );

            objDynamic_Ped->setRadius( def.radius_m );

            double easting_m, northing_m;
            LatLon2NE(def.Latitude, def.Longitude, &easting_m, &northing_m);
            objDynamic_Ped->setPose( easting_m, northing_m, def.Heading );
            objDynamic_Ped->setSpeed( def.Speed );

                 if(def.triggerId_Sim == 0) objDynamic_Ped->setIsSimTriggered( trigger0 );
            else if(def.triggerId_Sim == 1) objDynamic_Ped->setIsSimTriggered( trigger1 );
            else if(def.triggerId_Sim == 2) objDynamic_Ped->setIsSimTriggered( trigger2 );

            objsDynamic_Ped_.push_back( objDynamic_Ped );
        }
        else if( def.type == XML_Obj_Car ) {
            OBJECT_DYNAMIC_RECTANGLE* objDynamic_Veh = new OBJECT_DYNAMIC_RECTANGLE();

            objDynamic_Veh->setName( def.name );

            objDynamic_Veh->setLength( def.length_m );
            objDynamic_Veh->setWidth( def.width_m );

            double easting_m, northing_m;
            LatLon2NE(def.Latitude, def.Longitude, &easting_m, &northing_m);
            objDynamic_Veh->setPose( easting_m, northing_m, def.Heading );
            objDynamic_Veh->setSpeed( def.Speed );
            objDynamic_Veh->setAcc( 0.0 );

                 if(def.triggerId_Sim == 0) objDynamic_Veh->setIsSimTriggered( trigger0 );
            else if(def.triggerId_Sim == 1) objDynamic_Veh->setIsSimTriggered( trigger1 );
            else if(def.triggerId_Sim == 2) objDynamic_Veh->setIsSimTriggered( trigger2 );

                 if(def.laneTracking == "Lane1")  objDynamic_Veh->setLaneTracking( lanes_[0] );
            else if(def.laneTracking == "Lane2")  objDynamic_Veh->setLaneTracking( lanes_[1] );
            else if(def.laneTracking == "Lane3")  objDynamic_Veh->setLaneTracking( lanes_[2] );

                 if(def.laneToTrack == "Lane1")  objDynamic_Veh->setLaneToTrack( lanes_[0] );
            else if(def.laneToTrack == "Lane2")  objDynamic_Veh->setLaneToTrack( lanes_[1] );
            else if(def.laneToTrack == "Lane3")  objDynamic_Veh->setLaneToTrack( lanes_[2] );

                 if(def.triggerId_SwitchLane == 0) objDynamic_Veh->setIsSwitchTrackingLaneTriggered( trigger0 );
            else if(def.triggerId_SwitchLane == 1) objDynamic_Veh->setIsSwitchTrackingLaneTriggered( trigger1 );
            else if(def.triggerId_SwitchLane == 2) objDynamic_Veh->setIsSwitchTrackingLaneTriggered( trigger2 );

            objsDynamic_Veh_.push_back( objDynamic_Veh );
        }
        else if( def.type == XML_Obj_Bicyclist ) {
            OBJECT_DYNAMIC_RECTANGLE* objDynamic_Bike = new OBJECT_DYNAMIC_RECTANGLE();

            objDynamic_Bike->setName( def.name );

            objDynamic_Bike->setLength( def.length_m );
            objDynamic_Bike->setWidth( def.width_m );

            double easting_m, northing_m;
            LatLon2NE(def.Latitude, def.Longitude, &easting_m, &northing_m);
            objDynamic_Bike->setPose( easting_m, northing_m,  def.Heading );
            objDynamic_Bike->setSpeed( def.Speed );
            objDynamic_Bike->setAcc( 0.0 );

                 if(def.triggerId_Sim == 0) objDynamic_Bike->setIsSimTriggered( trigger0 );
            else if(def.triggerId_Sim == 1) objDynamic_Bike->setIsSimTriggered( trigger1 );
            else if(def.triggerId_Sim == 2) objDynamic_Bike->setIsSimTriggered( trigger2 );

                 if(def.laneTracking == "Lane1")  objDynamic_Bike->setLaneTracking( lanes_[0] );
            else if(def.laneTracking == "Lane2")  objDynamic_Bike->setLaneTracking( lanes_[1] );
            else if(def.laneTracking == "Lane3")  objDynamic_Bike->setLaneTracking( lanes_[2] );

                 if(def.laneToTrack == "Lane1")  objDynamic_Bike->setLaneToTrack( lanes_[0] );
            else if(def.laneToTrack == "Lane2")  objDynamic_Bike->setLaneToTrack( lanes_[1] );
            else if(def.laneToTrack == "Lane3")  objDynamic_Bike->setLaneToTrack( lanes_[2] );

                 if(def.triggerId_SwitchLane == 0) objDynamic_Bike->setIsSwitchTrackingLaneTriggered( trigger0 );
            else if(def.triggerId_SwitchLane == 1) objDynamic_Bike->setIsSwitchTrackingLaneTriggered( trigger1 );
            else if(def.triggerId_SwitchLane == 2) objDynamic_Bike->setIsSwitchTrackingLaneTriggered( trigger2 );

            objsDynamic_Bike_.push_back( objDynamic_Bike );
        }
    }


}

void WorldModel::getHostState_SELF(State& vehState, bool& isAutonomous)
{
    vehState = hv_->getVehicleState();
    isAutonomous = hv_->getIsAutonomous();
}

void WorldModel::getObjects_SELF(std::vector<OBJECT_STATIC*>& objsStatic,
                                 std::vector<OBJECT_DYNAMIC_CIRCLE*>& objsDynamic_Ped,
                                 std::vector<OBJECT_DYNAMIC_RECTANGLE*>& objsDynamic_Veh,
                                 std::vector<OBJECT_DYNAMIC_RECTANGLE*>& objsDynamic_Bike)
{
    objsStatic = objsStatic_;
    objsDynamic_Ped = objsDynamic_Ped_;
    objsDynamic_Veh = objsDynamic_Veh_;
    objsDynamic_Bike = objsDynamic_Bike_;
}

LanePiece* WorldModel::createNewLanePiece( Lane_Def def )
{
    vector<pair<double, double> > laneWpLL = def.waypoints;
    double width_m = def.width_m;
    double speedLim_mps = def.speedLim_mps;

    vector<pair<double, double> > laneWpNE; laneWpNE.clear();
    for(vector<pair<double, double> >::size_type ii=0; ii<laneWpLL.size(); ii++) {
        pair<double, double> tmpUTM;
        LatLon2NE(laneWpLL[ii].first, laneWpLL[ii].second, &tmpUTM.first, &tmpUTM.second);
        laneWpNE.push_back(tmpUTM);
    }

    LanePiece* lanePiece = new LanePiece();
    lanePiece->generateReference( laneWpLL, laneWpNE, width_m, speedLim_mps );

    return lanePiece;
}
