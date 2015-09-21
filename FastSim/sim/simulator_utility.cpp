#include "simulator.h"


//void simulator::getHostState_GM(core_host_state_t& hostState)
//{
//    hostState.master_state_enum = modelK_.getIsAutonomous() ? STATE_AUTONOMOUS_DRIVING : STATE_MANUAL_DRIVING;

//    hostState.heading_trusted = 1;

//    VEHICLE_STATUS_KINEMATIC vs = modelK_.getVehicleState();

//    double vsCG_x = vs.x_rear + cos(vs.theta) * VEH_WHEELBASE_B;
//    double vsCG_y = vs.y_rear + sin(vs.theta) * VEH_WHEELBASE_B;

//    double lat, lon;
//    NE2LatLon(vsCG_x, vsCG_y, &lat, &lon);


//    hostState.velocity_mps = vs.v;
//    hostState.lat_deg = lat;
//    hostState.lon_deg = lon;
//    hostState.yaw_angle_deg = RAD_TO_DEG * vs.theta;

//    hostState.lat_accel_mpss = 0.0;
//    hostState.lon_accel_mpss = vs.a;
//    hostState.yawrate_dps = 0.0;
//    hostState.road_wheel_angle_deg = RAD_TO_DEG * atan( vs.k * VEH_WHEELBASE );

//    hostState.ts_sec = get_sysclock_sec();
//}

//void simulator::getLanes_GM(trajplan_pathlane_t& currLane)
//{
//    currLane = lane1gm_;
//    currLane.ts_sec = get_sysclock_sec();
//}

//void simulator::getObjects_GM(trajplan_object_t* object_ptr, int& numOfObjects)
//{
////    VEHICLE_STATUS_KINEMATIC vs = modelK_.getVehicleState();

////    double x_veh = vs.x_rear;
////    double y_veh = vs.y_rear;
////    double theta_veh = vs.theta;


////    double x_obj = objStatic_.easting_m;
////    double y_obj = objStatic_.northing_m;
////    double theta_obj = objStatic_.heading_rad;

////    double DX = x_obj - x_veh;
////    double DY = y_obj - y_veh;
////    double DTheta = theta_obj - theta_veh;
////    object_ptr[0].id = 0;
////    object_ptr[0].type = OBJTYPE_STATIONARY;
////    object_ptr[0].ts_sec = get_sysclock_sec();
////    object_ptr[0].rel_x_m = DX * cos(theta_veh) + DY * sin(theta_veh);
////    object_ptr[0].rel_y_m = - DX * sin(theta_veh) + DY * cos(theta_veh);
////    object_ptr[0].rel_phi_deg = RAD_TO_DEG * ( DTheta );
////    object_ptr[0].length_m = objStatic_.length_m;
////    object_ptr[0].width_m = objStatic_.width_m;
////    object_ptr[0].speed_mps = 0.0;




////    x_obj = objDynamic_Ped_.easting_m;
////    y_obj = objDynamic_Ped_.northing_m;
////    theta_obj = objDynamic_Ped_.heading_rad;

////    DX = x_obj - x_veh;
////    DY = y_obj - y_veh;
////    DTheta = theta_obj - theta_veh;

////    object_ptr[1].id = 1;
////    object_ptr[1].type = OBJTYPE_PEDESTRIAN;
////    object_ptr[1].ts_sec = get_sysclock_sec();
////    object_ptr[1].rel_x_m = DX * cos(theta_veh) + DY * sin(theta_veh);
////    object_ptr[1].rel_y_m = - DX * sin(theta_veh) + DY * cos(theta_veh);
////    object_ptr[1].rel_phi_deg = RAD_TO_DEG * ( DTheta );
////    object_ptr[1].length_m = objDynamic_Ped_.radius_m;
////    object_ptr[1].width_m = objDynamic_Ped_.radius_m;
////    object_ptr[1].speed_mps = objDynamic_Ped_.speed_mps;



////    x_obj = objDynamic_Veh_.easting_m;
////    y_obj = objDynamic_Veh_.northing_m;
////    theta_obj = objDynamic_Veh_.heading_rad;

////    DX = x_obj - x_veh;
////    DY = y_obj - y_veh;
////    DTheta = theta_obj - theta_veh;

////    object_ptr[2].id = 2;
////    object_ptr[2].type = OBJTYPE_VEHICLE;
////    object_ptr[2].ts_sec = get_sysclock_sec();
////    object_ptr[2].rel_x_m = DX * cos(theta_veh) + DY * sin(theta_veh);
////    object_ptr[2].rel_y_m = - DX * sin(theta_veh) + DY * cos(theta_veh);
////    object_ptr[2].rel_phi_deg = RAD_TO_DEG * ( DTheta );
////    object_ptr[2].length_m = objDynamic_Veh_.radius_m;
////    object_ptr[2].width_m = objDynamic_Veh_.radius_m;
////    object_ptr[2].speed_mps = objDynamic_Veh_.speed_mps;


//    numOfObjects = 2;
//}



//void simulator::getHostState_SELF(State& vehState, bool& isAutonomous)
//{
//    VEHICLE_STATUS_KINEMATIC vs = modelK_.getVehicleState();

//    vehState.spatial.x = vs.x_rear;
//    vehState.spatial.y = vs.y_rear;
//    vehState.spatial.theta = vs.theta;
//    vehState.spatial.k = vs.k;

//    vehState.temporal.v = vs.v;
//    vehState.temporal.a = vs.a;

//    isAutonomous = modelK_.getIsAutonomous();
//}

//void simulator::getLanes_SELF(LanePiece& lane1, LanePiece& lane2, LanePiece& lane3)
//{
//    lane1 = lane1_;
//    lane2 = lane2_;
//    lane3 = lane3_;
//}

//void simulator::getObjects_SELF(std::vector<OBJECT_STATIC>& objsStatic,
//                                std::vector<OBJECT_DYNAMIC_CIRCLE>& objsDynamic_Ped,
//                                std::vector<OBJECT_DYNAMIC_RECTANGLE>& objsDynamic_Veh,
//                                std::vector<OBJECT_DYNAMIC_RECTANGLE>& objsDynamic_Bike)
//{
//    objsStatic = objsStatic_;
//    objsDynamic_Ped = objsDynamic_Ped_;
//    objsDynamic_Veh = objsDynamic_Veh_;
//    objsDynamic_Bike = objsDynamic_Bike_;
//}

