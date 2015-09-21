#include "simulator.h"

/*
 * Simulator
 */
void simulator::simTrackingControl(bool isPaused)
{
    if(exeTrajFull_.numOfPoints < 2) {
        wm_->hv_->setCommand(0.0, 0.0);
        return;
    }

    // Controller Routine
    PathTracker tracker(VEH_WHEELBASE, 0.2); // create a path tracker

    State vs = wm_->hv_->getVehicleState();
//    cout << "vs.v = " << vs.v <<  endl;

    // calculate cross track & heading error
    tracker.crossTrackError_m_ = exeTrajFull_.getProjectedSignedDistanceByCoordinates(vs.spatial.x, vs.spatial.y);

    State projState = exeTrajFull_.getProjectedTrajectoryStateByCoordinates(vs.spatial.x, vs.spatial.y);
    tracker.headingError_rad_ = -(projState.spatial.theta - vs.spatial.theta);
//    cout << "projState.v = " << projState.temporal.v << " projState.a = " << projState.temporal.a <<  endl;

//    debug_simulator("crossTrackError_m_=%f headingError_rad_=%f vs.x=%f vs.y=%f vs.h=%f",
//                    tracker.crossTrackError_m_,
//                    tracker.headingError_rad_,
//                    vs.x_rear,
//                    vs.y_rear,
//                    vs.theta);


    if (tracker.headingError_rad_ > M_PI) { tracker.headingError_rad_  -= 2 * M_PI; }
    else if (tracker.headingError_rad_ < -M_PI) { tracker.headingError_rad_  += 2 * M_PI; }

    tracker.curvature_path = projState.spatial.k;
    tracker.curvatureDot_path = projState.spatial.dk;
    tracker.curvatureDoubleDot_path = 0.0;

    // update longitudinal speed, and front road wheel angle
    tracker.longitudinalVelocity_mps_ = vs.temporal.v;
    tracker.wheelAngle_rad_ = atan(vs.spatial.k * VEH_WHEELBASE);


    double steeringRate = tracker.getSteeringVelocity();

    double wheelAngleTime = get_sysclock_sec();
    if(isPaused) {
        waOldTime_ = wheelAngleTime;
        wm_->hv_->setCommand(0.0, 0.0);

        return;
    }

    double deltaTime = wheelAngleTime - waOldTime_; if(deltaTime > 1.0){ deltaTime = 1.0; }
//    double deltaTime = 0.1;


    double desiredCurvature_k = tan(waOld_ + steeringRateOld_ * deltaTime) / VEH_WHEELBASE;
    double desiredSpeed_mps = projState.temporal.v + deltaTime * projState.temporal.a;
//    qDebug("waOld_ = %f    steeringRateOld_ = %f", waOld_, steeringRateOld_);
//    qDebug("desiredCurvature_k = %1.5f    desiredSpeed_mps = %1.5f", desiredCurvature_k, desiredSpeed_mps);


    // Set vehicle command
    wm_->hv_->setCommand(desiredSpeed_mps, desiredCurvature_k);

    // Update history variables

    steeringRateOld_ = steeringRate;
    waOld_ = tracker.wheelAngle_rad_;
    waOldTime_ = wheelAngleTime;

}

void simulator::simJoystickControl(double steeringAngle_rad, double acceleration_mpss)
{
    double jsTime = get_sysclock_sec();
    double deltaTime = jsTime - jsOldTime_; if(deltaTime > 1.0){ deltaTime = 1.0; }


    if(wm_->hv_->getVehicleState().temporal.v <= 0 && acceleration_mpss < 0) { acceleration_mpss = 0.0; }
    if(wm_->hv_->getVehicleState().temporal.v >= 50.0 && acceleration_mpss > 0) { acceleration_mpss = 0.0; }


    double desiredCurvature_k = tan( steeringAngle_rad ) / VEH_WHEELBASE;
    double desiredSpeed_mps = wm_->hv_->getVehicleState().temporal.v + deltaTime * acceleration_mpss;

    if(desiredSpeed_mps < 0.0) {desiredSpeed_mps = 0.0;}

    wm_->hv_->setCommand(desiredSpeed_mps, desiredCurvature_k);

    jsOldTime_ = jsTime;
}

void simulator::simVehModel(bool isPaused)
{
    double vehCurrTime = get_sysclock_sec();
    if(isPaused) {
        vehOldTime_ = vehCurrTime;
        return;
    }

    double deltaTime = vehCurrTime - vehOldTime_;
    if(deltaTime > 1.0){ deltaTime = 1.0; }

    wm_->hv_->stepSim(deltaTime);

    vehOldTime_ = vehCurrTime;
}

void simulator::simEnvChange(bool isPaused)
{
    double envCurrTime = get_sysclock_sec();
    if(isPaused) {
        envOldTime_ = envCurrTime;
        return;
    }

    double deltaTime = envCurrTime - envOldTime_; if(deltaTime > 1.0){ deltaTime = 1.0; }
//    double deltaTime = 0.1;


    for(vector<OBJECT_DYNAMIC_CIRCLE*>::size_type i=0; i<wm_->objsDynamic_Ped_.size(); i++) {
        wm_->objsDynamic_Ped_[i]->stepSim_linear(deltaTime);
    }

    for(vector<OBJECT_DYNAMIC_RECTANGLE*>::size_type i=0; i<wm_->objsDynamic_Bike_.size(); i++) {
        //    objDynamic_Veh_.stepSim_linear(deltaTime);
        wm_->objsDynamic_Bike_[i]->stepSim_linear(deltaTime);
    }

//    QTime tt; tt.restart();
    for(vector<OBJECT_DYNAMIC_RECTANGLE*>::size_type i=0; i<wm_->objsDynamic_Veh_.size(); i++) {
        wm_->objsDynamic_Veh_[i]->checkSwitchLaneTrigger();

        //    objDynamic_Veh_.stepSim_linear(deltaTime);
        wm_->objsDynamic_Veh_[i]->stepSim_tracking_dk(deltaTime, wm_->objsDynamic_Veh_, wm_->hv_);
    }
//    cout << "elapsed() = " << tt.elapsed() << endl;

    envOldTime_ = envCurrTime;
}
