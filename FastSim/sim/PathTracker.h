/*!
 *****************************************************************************
 * @file      PathTracker.h
 * @author    Jarrod Snider
 * @date      02/29/2012
 *
 * @attention Copyright (C) 2012 CMU. All rights reserved.
 *****************************************************************************
 */

#ifndef _PATHTRACKER_H_
#define _PATHTRACKER_H_

#include <assert.h>
#include <math.h>

struct PathPoint
{
    double station, x, y, heading, velocity, k, dk, ddk;
};


class PathTracker
{
    public:
        PathTracker(const double wheelBase_m, const double k);

        double getSteeringVelocity();

        double longitudinalVelocity_mps_, wheelAngle_rad_;
        double crossTrackError_m_, headingError_rad_;
        double curvature_path, curvatureDot_path, curvatureDoubleDot_path;

    private:
        const double wheelBase_m_;
        const double k1_;
        const double k2_;
        const double k3_;
};

#endif
