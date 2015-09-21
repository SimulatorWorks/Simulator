#include "Def4PlanPerception.h"

//HystereticBlockage::HystereticBlockage( )
//    : persistentBlockages_( )
//{
//}


//void HystereticBlockage::update( const RoadBlockageSet& blockages,
//                            boost::posix_time::ptime currTime,
//                            logger::Logger &logger )
//{
//    /* The algorithm:
//     * newPersistentBlockages \add b with creation time = currTime
//     * for each pb in persistentBlockages_:
//     *   if \exists b \in blockages s.t. pb intersects b
//     *     retainedPersistentBlockages \add b with creation time of pb
//     *   endif
//     * endfor
//     * persistentBlockages = retained \union new, using age of retained
//     *  in case of exact intersection.
//     */

//    BlockageAgeMap retainedBlockages, newBlockages;

//    /* Creat newBlockages.
//     */
//    for( RoadBlockageSet::const_iterator bit = blockages.begin();
//         bit != blockages.end(); ++bit )
//    {
//        const RoadBlockage& bb = *bit;
//        newBlockages[bb] = currTime;
//    }

//    /* Identify existing blockages that should persist past now.
//     */
//    for( BlockageAgeMap::const_iterator pbit =
//             persistentBlockages_.begin();
//         pbit != persistentBlockages_.end(); ++pbit )
//    {
//        const RoadBlockage& pp = pbit->first;
//        for( RoadBlockageSet::const_iterator bit = blockages.begin();
//             bit != blockages.end(); ++bit )
//        {
//            const RoadBlockage& bb = *bit;

//            if( doBlockagesIntersect( pp, bb ) )
//            {
//                retainedBlockages[ bb ] = pbit->second;
//                break;
//            }
//        }
//    }

//    /* Now merge lists of blockages.
//     *
//     * Adding the new blockages first and then inserting the retained
//     * blockages will have the effect of keeping the ages for the
//     * retained blockages when they are identical to a new one.
//     *
//     * If a new one and old one overlaps, we use the size of the new
//     * one and the age of the old one.
//     */
//    persistentBlockages_ = newBlockages;

//    for( BlockageAgeMap::const_iterator rbit = retainedBlockages.begin();
//         rbit != retainedBlockages.end(); ++rbit )
//    {
//        persistentBlockages_[ rbit->first ] = rbit->second;
//    }

//}


//bool HystereticBlockage::doBlockagesIntersect( const RoadBlockage &rb1, const RoadBlockage &rb2 ) const
//{
//    bool result = false;

//    if( rb1.laneId == rb2.laneId )
//    {
//        if( seg( rb1.distanceAlong_m, + rb1.distanceAlong_m + rb1.length_m ).intersects( seg( rb2.distanceAlong_m, rb2.distanceAlong_m + rb2.length_m ) ) )
//        {
//            result = true;
//        }
//    }
//    return result;
//}


///**
// * Fill rbs with blockages of a certain age.
// */
//void HystereticBlockage::getPersistentBlockages(RoadBlockageSet& rbs,
//                                                int hysteresis_ms,
//                                                boost::posix_time::ptime currTime,
//                                                logger::Logger& logger ) const
//{
//    for( BlockageAgeMap::const_iterator it = persistentBlockages_.begin();
//         it != persistentBlockages_.end(); ++it )
//    {
//        int age_ms = (currTime - it->second).total_milliseconds();
//        if( age_ms > hysteresis_ms )
//        {
//            logger.log_notice("Persistent blockage of age %d > %d",
//                               age_ms, hysteresis_ms );

//            rbs.push_back( it->first );
//        } else {
//            logger.log_notice("Spring chicken blockage of age %d < %d",
//                               age_ms, hysteresis_ms );

//        }
//    }
//}


//void PH_ROAD_BLOCKAGE::checkInZones(roadWorldModel::RoadWorldModel* rwm, double x, double y, bool& isInBufferZone, bool& isInFatalZone)
//{
//    roadWorldModel::RoadLane * rl = NULL;
//    if (!rwm->bindWorldElement(this->laneId, rl))
//    {
//        // In fatal zone
//        isInBufferZone = false;
//        isInFatalZone = true;
//    }

//    std::pair<double, double> sl = getSLfromXYbyRoadLane(rl, x, y);
//    double s = sl.first;
//    double l = sl.second;

//    double S0(this->s0_m - 0.5), Sf(this->sf_m + 0.5);
//    double S0Buff(S0 - 1.00), SfBuff(Sf + 1.00);

//    double width, widthSTD;
//    rl->getWidthAt(width, widthSTD, 0.0);

//    double L0, Lf;
//    double L0Buff, LfBuff;
//    if(this->sideType == PH_ROAD_BLOCKAGE::ST_FULL)
//    {
//        L0 = - width / 2;
//        Lf = width / 2;
//        L0Buff = - width / 2;
//        LfBuff =   width / 2;
//    }
//    else if(this->sideType == PH_ROAD_BLOCKAGE::ST_LEFT)
//    {
//        L0 = width / 2 - this->perc * width;
//        Lf = width / 2;
//        L0Buff = width / 2 - this->perc * width - 1.00;
//        LfBuff = width / 2;
//    }
//    else if(this->sideType == PH_ROAD_BLOCKAGE::ST_RIGHT)
//    {
//        L0 = - width / 2;
//        Lf = - width / 2 + this->perc * width;
//        L0Buff = - width / 2;
//        LfBuff = - width / 2 + this->perc * width + 1.00;
//    }

//    if(s >= S0 && s <= Sf && l >= L0 && l <= Lf)
//    {
//        isInFatalZone = true;
//    }
//    else
//    {
//        isInFatalZone = false;
//    }

//    if(s >= S0Buff && s <= SfBuff && l >= L0Buff && l <= LfBuff)
//    {
//        isInBufferZone = true;
//    }
//    else
//    {
//        isInBufferZone = false;
//    }
//}

//std::pair<double, double> PH_ROAD_BLOCKAGE::getSLfromXYbyRoadLane(roadWorldModel::RoadLane* rl, double x, double y)
//{
//    double s = rl->getDistanceNear(RecPoint2D(y, x));
//    RecPose2D rlPos, rlPosSTD;
//    rl->getPoseAt(rlPos, rlPosSTD, s);
//    double rlWidth, rlWidthSTD;
//    rl->getWidthAt(rlWidth, rlWidthSTD, s);

//    double xC = rlPos.y;
//    double yC = rlPos.x;
//    double hC = M_PI_2 - rlPos.rotZ;

//    double l = fmin( hypot(xC - x, yC - y), rlWidth / 2 );

//    double xL = xC - rlWidth / 2 * sin(hC);
//    double yL = yC + rlWidth / 2 * cos(hC);

//    double xR = xC + rlWidth / 2 * sin(hC);
//    double yR = yC - rlWidth / 2 * cos(hC);

//    double dist2Left = hypot(xL - x, yL - y);
//    double dist2Right = hypot(xR - x, yR - y);

//    double sign = dist2Right - dist2Left >= 0 ? 1 : -1;

//    std::pair<double, double> tmp(s, l*sign);

//    return tmp;
//}


void PH_OBJECT::stepSim(double dt_s) {
    if( speedLon_mps!= 0 ) isMoving = true;
    else isMoving = false;

    if( isMoving ) {
        if(hasTraj) {
            if(trajS.empty()) return;

            // Locate myself in terms of S
            int nearestIdx = -1;
            double minDist = GSL_POSINF;
            for(vector<double>::size_type i=0; i<trajS.size(); i++) {
                double dist = hypot(pose.x-trajX[i], pose.y-trajY[i]);
                if(dist < minDist) {
                    minDist = dist;
                    nearestIdx = i;
                }
            }

            // Sim
            double s0 = trajS[nearestIdx];
            double l = speedLon_mps * dt_s;
            double sf = s0 + l;

            // Get Pose
            nearestIdx = -1;
            minDist = GSL_POSINF;
            for(vector<double>::size_type i=0; i<trajS.size(); i++) {
                double dist = fabs(sf - trajS[i]);
                if(dist < minDist) {
                    minDist = dist;
                    nearestIdx = i;
                }
            }

            pose.x = trajX[nearestIdx];
            pose.y = trajY[nearestIdx];
            pose.theta = trajH[nearestIdx];
        }
        else {
            double l = speedLon_mps * dt_s;
            pose.x += l * cos(pose.theta);
            pose.y += l * sin(pose.theta);
        }
    }
}


GenericPose PH_OBJECT::getPoseProjectionByTime(double dt_s)
{
    GenericPose poseNew;

    if(hasTraj && !trajS.empty()) {
        // Locate myself in terms of S
        int nearestIdx = -1;
        double minDist = GSL_POSINF;
        for(vector<double>::size_type i=0; i<trajS.size(); i++) {
            double dist = hypot(pose.x-trajX[i], pose.y-trajY[i]);
            if(dist < minDist) {
                minDist = dist;
                nearestIdx = i;
            }
        }

        // Sim
        double s0 = trajS[nearestIdx];
        double l = speedLon_mps * dt_s;
        double sf = s0 + l;

        // Get Pose
        nearestIdx = -1;
        minDist = GSL_POSINF;
        for(vector<double>::size_type i=0; i<trajS.size(); i++) {
            double dist = fabs(sf - trajS[i]);
            if(dist < minDist) {
                minDist = dist;
                nearestIdx = i;
            }
        }

        poseNew.x = trajX[nearestIdx];
        poseNew.y = trajY[nearestIdx];
        poseNew.theta = trajH[nearestIdx];
    }
    else {
        double l = speedLon_mps * dt_s;
        poseNew.x = pose.x + l * cos(pose.theta);
        poseNew.y = pose.y + l * sin(pose.theta);
        poseNew.theta = pose.theta;
    }

    return poseNew;
}


vector<PH_COVERCIRCLE> PH_OBJECT::getCoverCircleByPose(GenericPose pose)
{
    int numCircle = numOfCoverCircles;

    vector<PH_COVERCIRCLE> circles; circles.clear();

    double theta = pose.theta;
    double xR = pose.x - length_m/2 * cos(theta);
    double yR = pose.y - length_m/2 * sin(theta);

    double radius = sqrt( (length_m / numCircle / 2)*(length_m / numCircle / 2) + (width_m/2)*(width_m/2) );

    for(int i=0; i<numCircle; i++) {
        double l = i * length_m / numCircle + (length_m / numCircle / 2);

        PH_COVERCIRCLE circle;
        circle.x = xR + l * cos(theta);
        circle.y = yR + l * sin(theta);
        circle.r = radius;

        circles.push_back( circle );
    }

    return circles;
}
