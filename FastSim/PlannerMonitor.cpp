#include "PlannerMonitor.h"
#include "ui_PlannerMonitor.h"


void PlannerMonitor::on_btnToggleDisplaySim_RoadStructure_clicked()
{
    isDisplaySim_RoadStructure_ = ! isDisplaySim_RoadStructure_;
}

void PlannerMonitor::on_btnToggleDisplaySim_VehiclePose_clicked()
{
    isDisplaySim_VehiclePose_ = ! isDisplaySim_VehiclePose_;
}

void PlannerMonitor::on_btnToggleDisplaySim_GroundTruthObjects_clicked()
{
    isDisplaySim_GroundTruthObjects_ = ! isDisplaySim_GroundTruthObjects_;
}

void PlannerMonitor::on_btnJoystickOverride_clicked()
{
    if(jsConnected_) { isJoystickOverride_ = !isJoystickOverride_; }
    else {isJoystickOverride_ = false;}
}
