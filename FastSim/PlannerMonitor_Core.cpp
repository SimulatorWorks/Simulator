/**
* @file         PlannerMonitor_Core.cpp
* @author       Tianyu Gu
* @date         06/10/2013
*/

#include "PlannerMonitor.h"
#include "ui_PlannerMonitor.h"

void PlannerMonitor::timerQtSimulatorTimeout()
{
//    cout << "-----------------------Simulating-----------------------" << endl;

    // SIMLUATE
    if(isSimulatorEnabled_)
    {
        coreSimulatingSequence( ui->chkSimIsolated->isChecked() );
    }
}

void PlannerMonitor::timerGetJoystickInputTimeout()
{
    double steeringAngle_rad=0;
    double acceleration_mpss=0;

    if(jsConnected_) {
        /* Try Update Joystick Controls in this cycle */
        js_->updateControls();
        /* Get Control from Joystick Controller */
        js_->getCurrentControl(steeringAngle_rad, acceleration_mpss);
    }
}

void PlannerMonitor::corePlanningSequence()
{
    // INPUT & INTERFACING (SELF)
    State vehState; bool isAutonomous;
    vector<LanePiece*> lanes;
    vector<OBJECT_STATIC*> objsStatic; objsStatic.clear();
    vector<OBJECT_DYNAMIC_CIRCLE*> objsDynamic_Ped; objsDynamic_Ped.clear();
    vector<OBJECT_DYNAMIC_RECTANGLE*> objsDynamic_Veh; objsDynamic_Veh.clear();
    vector<OBJECT_DYNAMIC_RECTANGLE*> objsDynamic_Bike; objsDynamic_Bike.clear();
    sim_->wm_->getHostState_SELF(vehState, isAutonomous);
    sim_->wm_->getLanes_SELF(lanes);
    sim_->wm_->getObjects_SELF(objsStatic, objsDynamic_Ped, objsDynamic_Veh, objsDynamic_Bike);
}


void PlannerMonitor::coreSimulatingSequence( bool isSimIsolated )
{
    bool isEnvSimPaused = isSimIsolated ? isSimulatorEnabled_ : false;

    if(!isJoystickOverride_)
    {
        for(int ii = 0; ii<10; ii++)
        {
            double steeringAngle_rad, acceleration_mpss;
            js_->getCurrentControl(steeringAngle_rad, acceleration_mpss);

            sim_->simJoystickControl(steeringAngle_rad, acceleration_mpss);
            sim_->simVehModel(false);
            sim_->simEnvChange(false);
        }
    }
    else
    {
        //        sim_->setTrajectory( plannerOutput_ );
        //        sim_->setTrajectoryFull( plannerOutputFull_ );
        //        sim_->simTrackingControl( !isPlanningSequenceEnabled_ );
        for(int ii = 0; ii<10; ii++)
        {
            sim_->simVehModel(isEnvSimPaused);
            sim_->simEnvChange(isEnvSimPaused);
        }
    }

}
