/**
* @file         PlannerMonitor_InteractiveControl.cpp
* @author       Tianyu Gu
* @date         06/10/2013
*/


#include "PlannerMonitor.h"
#include "ui_PlannerMonitor.h"

void PlannerMonitor::on_btnSimSwitch_clicked()
{
    isSimulatorEnabled_ = ! isSimulatorEnabled_;
}

void PlannerMonitor::on_btnWorldZoomIn_clicked()
{
    canvasZoomScale_ = canvasZoomScale_ / 1.2;
}

void PlannerMonitor::on_btnWorldZoomOut_clicked()
{
    canvasZoomScale_ = canvasZoomScale_ * 1.2;
}

void PlannerMonitor::on_btnWorldLeft_clicked()
{
    canvasHorizontalOffset_ = canvasHorizontalOffset_ - 5;
}

void PlannerMonitor::on_btnRight_clicked()
{
    canvasHorizontalOffset_ = canvasHorizontalOffset_ + 5;
}

void PlannerMonitor::on_btnWorldUp_clicked()
{
    canvasVerticalOffset_ = canvasVerticalOffset_ + 5;
}

void PlannerMonitor::on_btnWorldDown_clicked()
{
    canvasVerticalOffset_ = canvasVerticalOffset_ - 5;
}

void PlannerMonitor::on_btnUpdateFromSimVs_clicked()
{
//    double northing_m, easting_m;
//    LatLon2NE(hostState_.lat_deg, hostState_.lon_deg, &easting_m, &northing_m);

//    double yaw_rad = DEG_TO_RAD * hostState_.yaw_angle_deg;
//    double curv_k = tan(DEG_TO_RAD * hostState_.road_wheel_angle_deg) / VEH_WHEELBASE;
//    double speed_mps = hostState_.velocity_mps;
//    double acc_mpss = hostState_.lon_accel_mpss;

    State vs;
    bool isAutonomous;
    sim_->wm_->getHostState_SELF(vs, isAutonomous);

    double easting_m = vs.spatial.x;
    double northing_m = vs.spatial.y;
    double yaw_rad = vs.spatial.theta;
    double curv_k = vs.spatial.k;
    double speed_mps = vs.temporal.v;
    double acc_mpss = vs.temporal.a;


    ui->lineVsEasting->setText(QString::number(easting_m, 'g', 10));
    ui->lineVsNorthing->setText(QString::number(northing_m, 'g', 10));
    ui->lineVsYaw->setText(QString::number(yaw_rad, 'g', 5));
    ui->lineVsCurv->setText(QString::number(curv_k, 'g', 5));

    ui->lineVsSpeed->setText(QString::number(speed_mps, 'g', 4));
    ui->lineVsAcc->setText(QString::number(acc_mpss, 'g', 3));
}

void PlannerMonitor::on_btnSetSimVs_clicked()
{
    double easting, northing, yaw, curv, speed, acc;

    easting = ui->lineVsEasting->text().toDouble();
    northing = ui->lineVsNorthing->text().toDouble();
    yaw = ui->lineVsYaw->text().toDouble();
    curv = ui->lineVsCurv->text().toDouble();
    speed = ui->lineVsSpeed->text().toDouble();
    acc = ui->lineVsAcc->text().toDouble();

    sim_->wm_->initVehicleState_NE(easting, northing, yaw, curv, speed, acc);
}



