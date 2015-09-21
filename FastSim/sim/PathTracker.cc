/*!
 *****************************************************************************
 * @file      PathTracker.cc
 * @author    Jarrod Snider
 * @date      02/29/2012
 *
 * @attention Copyright (C) 2012 CMU. All rights reserved.
 *****************************************************************************
 */

#include "PathTracker.h"

PathTracker::PathTracker( const double wheelBase_m, const double k )
    : wheelBase_m_(wheelBase_m),
      k1_(k*k*k),
      k2_(3*k*k),
      k3_(3*k)
{
}

double PathTracker::getSteeringVelocity()
{
    // Fill in the states x2, x3, x4.  We will not need x1.
    double x2, x3, x4;

    x2 = -1.0 * curvatureDot_path * crossTrackError_m_ * tan(headingError_rad_);
    x2 = x2 - curvature_path * (1 - crossTrackError_m_ * curvature_path) * ((1 + sin(headingError_rad_) * sin(headingError_rad_)) / (cos(headingError_rad_) * cos(headingError_rad_)));
    x2 = x2 + ((1 - crossTrackError_m_ * curvature_path) * (1 - crossTrackError_m_ * curvature_path) * tan(wheelAngle_rad_)) / (wheelBase_m_ * cos(headingError_rad_) * cos(headingError_rad_) *  cos(headingError_rad_));
    x3 = (1 - crossTrackError_m_ * curvature_path) * tan(headingError_rad_);
    x4 = crossTrackError_m_;

    // Compute alphas for the variable transformations.

    double partialDerivative_x2_wrt_s, partialDerivative_x2_wrt_crossTrackError_m_, partialDerivative_x2_wrt_headingError_rad_;
    double alpha1, alpha2;

    partialDerivative_x2_wrt_s = -1.0 * curvatureDoubleDot_path * crossTrackError_m_ * tan(headingError_rad_);
    partialDerivative_x2_wrt_s = partialDerivative_x2_wrt_s - (curvatureDot_path - 2 * crossTrackError_m_ * curvature_path * curvatureDot_path) * (1 + sin(headingError_rad_) * sin(headingError_rad_)) / (cos(headingError_rad_) * cos(headingError_rad_));
    partialDerivative_x2_wrt_s = partialDerivative_x2_wrt_s - (2 * (1-crossTrackError_m_ * curvature_path) * crossTrackError_m_ * curvatureDot_path * tan(wheelAngle_rad_)) / (wheelBase_m_ * cos(headingError_rad_) * cos(headingError_rad_) * cos(headingError_rad_));
    partialDerivative_x2_wrt_crossTrackError_m_ = -1.0 * curvatureDot_path * tan(headingError_rad_);
    partialDerivative_x2_wrt_crossTrackError_m_ = partialDerivative_x2_wrt_crossTrackError_m_ + (curvature_path * curvature_path) * (1 + sin(headingError_rad_) * sin(headingError_rad_)) / (cos(headingError_rad_) *  cos(headingError_rad_));
    partialDerivative_x2_wrt_crossTrackError_m_ = partialDerivative_x2_wrt_crossTrackError_m_ - (2 * curvature_path * (1 - crossTrackError_m_ * curvature_path) * tan(wheelAngle_rad_)) / (wheelBase_m_ * cos(headingError_rad_) * cos(headingError_rad_)* cos(headingError_rad_));
    partialDerivative_x2_wrt_headingError_rad_ = -1.0 * curvatureDot_path * crossTrackError_m_ * ( 1 / (cos(headingError_rad_) * cos(headingError_rad_)));
    partialDerivative_x2_wrt_headingError_rad_ = partialDerivative_x2_wrt_headingError_rad_ - curvature_path * (1 - crossTrackError_m_ * curvature_path) * 4 * tan(headingError_rad_) / (cos(headingError_rad_) * cos(headingError_rad_));
    partialDerivative_x2_wrt_headingError_rad_ = partialDerivative_x2_wrt_headingError_rad_ + (3 * (1 - crossTrackError_m_ * curvature_path) * (1 - crossTrackError_m_ * curvature_path) * tan(wheelAngle_rad_) * tan(headingError_rad_)) / (wheelBase_m_ * cos(headingError_rad_) * cos(headingError_rad_)* cos(headingError_rad_));

    alpha1 = partialDerivative_x2_wrt_headingError_rad_ * ((tan(wheelAngle_rad_) * (1 - crossTrackError_m_ * curvature_path)) / (wheelBase_m_ * cos(headingError_rad_)) - curvature_path);
    alpha1 = alpha1 + partialDerivative_x2_wrt_crossTrackError_m_ * (1 - crossTrackError_m_ * curvature_path) * tan(headingError_rad_);
    alpha1 = alpha1 + partialDerivative_x2_wrt_s;
    alpha2 = (wheelBase_m_ * cos(headingError_rad_) * cos(headingError_rad_) * cos(headingError_rad_) * cos(wheelAngle_rad_) * cos(wheelAngle_rad_));
    alpha2 = alpha2 / ((1 - crossTrackError_m_ * curvature_path) * (1 - crossTrackError_m_ * curvature_path));

    // Transform longitudinal velocity to u1.
    const double u1 = (cos(headingError_rad_) / (1 - crossTrackError_m_ * curvature_path)) * longitudinalVelocity_mps_;

    // Calculate u2 using control law.
    const double u2 = -k1_ * fabs(u1) * x4 - k2_ * u1 * x3 - k3_ * fabs(u1) * x2;
    
    // Transform u2 to angular velocity of road wheel angle
    const double wheelAngleVelocity = alpha2 * (u2 - alpha1 * u1);
    
    return wheelAngleVelocity;
}

