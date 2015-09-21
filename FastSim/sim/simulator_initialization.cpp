#include "simulator.h"


simulator::simulator()
{
    steeringRateOld_ = 0.0;
    waOld_ = 0.0;

    waOldTime_ = get_sysclock_sec();
    vehOldTime_ = get_sysclock_sec();
    envOldTime_ = get_sysclock_sec();

    wm_ = NULL;
}

//void simulator::initVehicleState_LL(double lat, double lon, double heading, double curv, double speed, double acc)
//{
//    double easting_m, northing_m;
//    LatLon2NE(lat, lon, &easting_m, &northing_m);

//    modelK_.forceSetVehicleState(easting_m, northing_m, heading, curv, speed, acc);
//}

//void simulator::initVehicleState_NE(double easting, double northing, double heading, double curv, double speed, double acc)
//{
//    modelK_.forceSetVehicleState(easting, northing, heading, curv, speed, acc);
//}


//void simulator::initRoadModel(LanePiece lane1,
//                              LanePiece lane2,
//                              LanePiece lane3)
//{
////    lane1gm_ = lane1GM;
////    lane2gm_ = lane2GM;
////    lane3gm_ = lane3GM;

//    lane1_ = lane1;
//    lane2_ = lane2;
//    lane3_ = lane3;
//}
