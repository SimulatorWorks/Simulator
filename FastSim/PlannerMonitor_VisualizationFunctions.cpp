/**
* @file         PlannerMonitor_VisualizationFunctions.h
* @author       Tianyu Gu
* @date         06/11/2013
*/

#include "PlannerMonitor.h"
#include "ui_PlannerMonitor.h"

#define COLOR_RED   QColor(255,0,0)
#define COLOR_BLUE  QColor(0,0,255)

void PlannerMonitor::timerUpdateDisplayTimeout()
{
    ui->lbSimSwitch->setText( isSimulatorEnabled_ ? "Enabled" : "Disabled" );

    if(isJoystickOverride_) { ui->lblJSSwitch->setText(QString("OVERRIDE")); }
    else { ui->lblJSSwitch->setText(QString("IDLE")); }

    // Display Simulator related items
    if(sim_->wm_ != NULL) {
        setWorldPerspective();
        displaySim_VehState(isDisplaySim_VehiclePose_);
        displaySim_RoadStructure(isDisplaySim_RoadStructure_);
        displaySim_Objects(isDisplaySim_GroundTruthObjects_);
    }

    // Display Joystick Inputs
    displayJS_Input();
}


void PlannerMonitor::setWorldPerspective()
{
    //    double northing_m, easting_m;
    //    LatLon2NE(hostState_.lat_deg, hostState_.lon_deg, &easting_m, &northing_m);

    State vehState;
    bool isAutonomous;
    sim_->wm_->getHostState_SELF(vehState, isAutonomous);

    double northing_m, easting_m;

    easting_m = vehState.spatial.x;
    northing_m = vehState.spatial.y;

#define WORLD_WINDOW_LENGTH 100

    ui->plot2Dworld->setAxisScale(0,
                                  northing_m - WORLD_WINDOW_LENGTH * canvasZoomScale_ + canvasVerticalOffset_,
                                  northing_m + WORLD_WINDOW_LENGTH * canvasZoomScale_ + canvasVerticalOffset_);

    ui->plot2Dworld->setAxisScale(2,
                                  easting_m - WORLD_WINDOW_LENGTH * canvasZoomScale_ + canvasHorizontalOffset_,
                                  easting_m + WORLD_WINDOW_LENGTH * canvasZoomScale_ + canvasHorizontalOffset_);

}

void PlannerMonitor::displaySim_VehState(bool isDisplay)
{
//    State vs = sim_->wm_->hv_.getVehicleState();

    if(isDisplay) {
        sim_->wm_->hv_->attach(ui->plot2Dworld);
        sim_->wm_->hv_->updateDrawings();
    }
    else {
        sim_->wm_->hv_->detach();
    }

    if(isDisplay) { ui->btnToggleDisplaySim_VehiclePose->setText(QString("ON")); }
    else { ui->btnToggleDisplaySim_VehiclePose->setText(QString("OFF")); }

}

void PlannerMonitor::displaySim_RoadStructure(bool isDisplay)
{
    vector<LanePiece*> roadModel = sim_->wm_->getRoadModel4Display();

    if(isDisplay) {
//        for(int ii=0; ii<5; ii++) {
//            drawingLanes_[ii].detach();
//        }
        for(vector<LanePiece*>::size_type ii=0; ii<roadModel.size(); ii++) {
            roadModel[ii]->attach(ui->plot2Dworld);
            roadModel[ii]->updateDrawings();
        }
    }
    else {
        for(vector<LanePiece*>::size_type ii=0; ii<roadModel.size(); ii++) {
            roadModel[ii]->detach();
        }
    }


    if(isDisplay) { ui->btnToggleDisplaySim_RoadStructure->setText(QString("ON")); }
    else { ui->btnToggleDisplaySim_RoadStructure->setText(QString("OFF")); }
}


void PlannerMonitor::displaySim_Objects(bool isDisplay)
{
    // Retrieve Info
    vector<OBJECT_STATIC*> objectsStatic = sim_->wm_->getObjectsStatic4Display();
    vector<OBJECT_DYNAMIC_CIRCLE*> objectsDynamic_Ped = sim_->wm_->getObjectsDynamicPed4Display();
    vector<OBJECT_DYNAMIC_RECTANGLE*> objectsDynamic_Veh = sim_->wm_->getObjectsDynamicVeh4Display();
    vector<OBJECT_DYNAMIC_RECTANGLE*> objectsDynamic_Bike = sim_->wm_->getObjectsDynamicBike4Display();

    // Display in World Plot
    if(isDisplay)
    {
        for(vector<OBJECT_STATIC*>::size_type ii=0; ii<objectsStatic.size(); ii++) {
            objectsStatic[ii]->attach( ui->plot2Dworld );
            objectsStatic[ii]->updateDrawings();
        }
        for(vector<OBJECT_DYNAMIC_CIRCLE*>::size_type ii=0; ii<objectsDynamic_Ped.size(); ii++) {
            objectsDynamic_Ped[ii]->attach( ui->plot2Dworld );
            objectsDynamic_Ped[ii]->updateDrawings();
        }
        for(vector<OBJECT_DYNAMIC_RECTANGLE*>::size_type ii=0; ii<objectsDynamic_Veh.size(); ii++) {
            objectsDynamic_Veh[ii]->attach( ui->plot2Dworld );
            objectsDynamic_Veh[ii]->updateDrawings();
        }
        for(vector<OBJECT_DYNAMIC_RECTANGLE*>::size_type ii=0; ii<objectsDynamic_Bike.size(); ii++) {
            objectsDynamic_Bike[ii]->attach( ui->plot2Dworld );
            objectsDynamic_Bike[ii]->updateDrawings();
        }

    }
    else
    {
        for(vector<OBJECT_STATIC*>::size_type ii=0; ii<objectsStatic.size(); ii++) {
            objectsStatic[ii]->detach();
        }

        for(vector<OBJECT_DYNAMIC_CIRCLE*>::size_type ii=0; ii<objectsDynamic_Ped.size(); ii++) {
            objectsDynamic_Ped[ii]->detach();
        }

        for(vector<OBJECT_DYNAMIC_RECTANGLE*>::size_type ii=0; ii<objectsDynamic_Veh.size(); ii++) {
            objectsDynamic_Veh[ii]->detach();
        }

        for(vector<OBJECT_DYNAMIC_RECTANGLE*>::size_type ii=0; ii<objectsDynamic_Bike.size(); ii++) {
            objectsDynamic_Bike[ii]->detach();
        }

    }

    // Display in Corresponding Widget
    if(isDisplay) { ui->btnToggleDisplaySim_GroundTruthObjects->setText(QString("ON")); }
    else { ui->btnToggleDisplaySim_GroundTruthObjects->setText(QString("OFF")); }
}

void PlannerMonitor::displayJS_Input()
{
//    curveSteerWheelHorizontalBar_

    // Display in Corresponding Widget

}
