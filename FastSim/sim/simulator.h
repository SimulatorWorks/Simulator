#ifndef SIMULATOR_H
#define SIMULATOR_H

//#include <interfacing/trajplan_datatypes.h>

#include <res/Comm.h>
#include <res/CommFunctions.h>
#include <res/Def4PlanTrajectory.h>
#include <res/Def4Environment.h>

#include "WorldModel.h"

#include "PathTracker.h"


//#define __DEBUG_SIMULATOR__

#ifdef __DEBUG_SIMULATOR__
    #define debug_simulator(...) qDebug(__VA_ARGS__)
#else
    #define debug_simulator(...)
#endif


class simulator
{
public:
    simulator();

    // Environment Simulation Module
public:
    inline void setWorldModel(WorldModel* wm) { wm_ = wm; }
    void simEnvChange(bool isPaused);
    WorldModel* wm_;

private:
    double envOldTime_;

    // Perception Simulation Module
public:

private:

    // Vehicle/Control Simulation Module
public:
//    inline void setTrajectory(trajplan_trajectory traj) { exeTraj_ = traj; }
    inline void setTrajectoryFull(trajplan_trajectory_plus traj) { exeTrajFull_ = traj; }

    // Sim: do not need to pass time interval
    void simTrackingControl(bool isPaused);
    void simJoystickControl(double steeringAngle_rad, double acceleration_mpss);

    void simVehModel(bool isPaused);

private:
//    trajplan_trajectory exeTraj_; // Point-wise Trajectory to execute (LL)
    trajplan_trajectory_plus exeTrajFull_; // Point-wise Trajectory for sim controller (Easting / Northing, subject to which trans function used)

    double waOldTime_;
    double steeringRateOld_;
    double waOld_;

    double vehOldTime_;
    double jsOldTime_;


    // Other functionalities
private:
    inline double  timeval_as_double(struct timeval* tv) {
        double sec = (double)tv->tv_sec + ((double)tv->tv_usec / 1000000.0);
        return sec;
    }

    inline double  get_sysclock_sec(void) {
        struct timeval tv;
        int ret;
        ret = gettimeofday(&tv, NULL);
        if (ret) {
            perror("gettimeofday");
            return -1.0;
        }
        else {
            return timeval_as_double(&tv);
        }
    }
};

#endif // SIMULATOR_H
