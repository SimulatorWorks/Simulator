// LatLongCalculations.c

// Derived from LatLongCalculations.cpp by P. Mudalige (and others?)
// Made Lat,Lon <--> N,E conversion routines consider reference lat. & lon.

// Added assess_location( ) function to determine if one point is ahead,
// behind, left, right, another point

// Latest update by M. Losh (under contract to GM) 2012.07.11

#include <stdio.h>
#include "LatLongCalculations.h"
//#include "common_types.h"

int CalcDistAzimuth( double hostLat,
                    double hostLon,
                    double hostHeading,
                    double povLat,
                    double povLon,
                    double povHeading,
                    double *pRange,
                    double *pAzimuth )
{
	int	   iResult = 0 ;

	iResult |= CalcDist( hostLat, hostLon, povLat, povLon, pRange ) ;

	iResult |= CalcAzimuth( hostLat, hostLon, hostHeading, povLat, povLon, povHeading, pAzimuth ) ;

	return iResult ;
}

int CalcDist( double Lat1,
             double Lon1,
             double Lat2,
             double Lon2,
             double *pRange )
{
	int	   iResult = 0 ;

   double   r ;   // radius of earth
   iResult |= GetEarthRadius( Lat1, Lon1, &r ) ;

   double   lat1_rad = Lat1*DEG_TO_RAD ;
   double   lon1_rad = Lon1*DEG_TO_RAD ;
   double   lat2_rad = Lat2*DEG_TO_RAD ;
   double   lon2_rad = Lon2*DEG_TO_RAD ;
   //double   dlon_rad = lon2_rad - lon1_rad ;
   //double   dlat_rad = lat2_rad - lat1_rad ;

/*
   // use great circle method using Haversine distance and assuming perfect sphere
   // from http://mathforum.org/library/drmath/view/51879.html
   double   a, c ;
   a = pow(sin(dlat_rad/2.),2.) + cos(lat1_rad)*cos(lat2_rad)*pow(sin(dlon_rad/2.),2.) ;
   c = 2 * atan2(sqrt(a), sqrt(1-a)) ;
   *pRange =  r*c ;
*/

	// use great circle method assuming perfect sphere
   // may have roundoff errors for closely spaced points
	*pRange = r * acos( sin(lat1_rad)*sin(lat2_rad) + cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad) ) ;

	return iResult ;
}

int CalcAzimuth( double Lat1,
                double Lon1,
                double Heading1,
                double Lat2,
                double Lon2,
                double Heading2,
                double *pAzimuth )
{
	int	   iResult = 0 ;

	double	Bearing ;

   iResult |= CalcBearing( Lat1, Lon1, Lat2, Lon2, &Bearing ) ;

   *pAzimuth = ReturnPlusMinus180( Bearing - Heading1 ) ;

	return iResult ;
}

// calculate bearing from (Lat1, Lon1) to (Lat2, Lon2)
// with zero bearing being North
int CalcBearing( double Lat1,
                double Lon1,
                double Lat2,
                double Lon2,
                double *pBearing )
{
	int	   iResult = 0 ;
   double   E, N ;
   double   b ;

   // convert to North/East coordinates in meters
//   iResult |= ConvertLatLonToXY(Lat1, Lon1, Lat2-Lat1, Lon2-Lon1, &E, &N) ; // ML: seems wrong!
   iResult |= ConvertLatLonToXY(Lat1, Lon1, Lat2, Lon2, &E, &N) ;             // ML: fixed version
   b = atan2( E, N )*RAD_TO_DEG ;
   /*// atan2 should correctly identify the quadrant
   if ( E<0 )
      b += 180.0 ;
   */

   *pBearing = ReturnPlusMinus180( b ) ;

	return iResult ;
}



int ConvertLatLonToXYVehicleCoords( double latref,
                      double lonref,
                      double heading,
                      double latDelta,
                      double lonDelta,
                      double *pX,
                      double *pY)
{
   int	   iResult = 0 ;
   double   E, N ;
   double   heading_rad = heading*DEG_TO_RAD ;

   // first, retrieve the point in NE coordinates
   iResult |= ConvertLatLonToNE(latref,
                        lonref,
                        latDelta,
                        lonDelta,
                        &E,
                        &N) ;

   // next, rotate NE into XY
   *pX =  N*cos(heading_rad) + E*sin(heading_rad) ;
   *pY = -N*sin(heading_rad) + E*cos(heading_rad) ;

   return iResult ;
}



int	ConvertLatLonToNE(double latref,
                        double lonref,
                        double lat,
                        double lon,
                        double *pE,
                        double *pN)
{
   // Let: R be the radius of the Earth and lon/lat be expressed in radians
   // X = Rcos(latref)(lon-lonref) and Y = R(lat-latref)
	int	   iResult = 0 ;

   double   r ;
   // retrieve the radius of the earth
   iResult |= GetEarthRadius( latref, lonref, &r ) ;

   double   latref_rad = latref * DEG_TO_RAD;
   double   lonref_rad = lonref * DEG_TO_RAD;
   double   lat_rad    = lat    * DEG_TO_RAD;
   double   lon_rad    = lon    * DEG_TO_RAD;

   *pE = r * cos(latref_rad) * (lon_rad - lonref_rad);
   *pN = r * (lat_rad - latref_rad);

	return iResult ;
}
/* all calls to this function should be replaced by ConvertLatLonToNE */
int	ConvertLatLonToXY(double latref,
                        double lonref,
                        double lat,
                        double lon,
                        double *pE,
                        double *pN)
{
   return ConvertLatLonToNE( latref, lonref, lat, lon, pE, pN ) ;
}




int ConvertNEToLatLon( double E,
                      double N,
                      double latref,
                      double lonref,
//                      double headingref,
                      double *pLat,
                      double *pLon)
{
   // Let: R be the radius of the Earth and lon/lat be expressed in radians
   // X = Rcos(latref)(lon-lonref) and Y = R(lat-latref)
	int	   iResult = 0 ;

   double   r ;
   // retrieve the radius of the earth
   iResult |= GetEarthRadius( latref, lonref, &r ) ;

   double   latref_rad = latref * DEG_TO_RAD;
   //double   lonref_rad = lonref * DEG_TO_RAD;
   double   lat_rad ;
   double   lon_rad ;

   // these equations originated from ConvertLatLongToXY(...)
   lon_rad = E/(r*cos(latref_rad)) ;
   lat_rad = N/r ;

   // assign outputs
   *pLat = lat_rad*RAD_TO_DEG + latref;
   *pLon = lon_rad*RAD_TO_DEG + lonref;

	return iResult ;
}
/* all calls to this function should be replaced by ConvertNEToLatLon */
int ConvertXYToLatLon( double E,
                      double N,
                      double latref,
                      double lonref,
                      double *pLat,
                      double *pLon)
{
   return ConvertNEToLatLon( E, N, latref, lonref, pLat, pLon ) ;
}




int GetEarthRadius( double latref,
                   double lonref,
                   double *pRadius )
{
   int      iResult = 0 ;

   *pRadius = 6358240.0; // 6437376.0 ;

   return iResult ;
}

double	ReturnPlusMinus180(double degrees)
{
	while ( degrees >= 180.0l ) degrees -= 360.0l ;
	while ( degrees < -180.0l ) degrees += 360.0l ;

	return degrees ;
}

double	Return0to360(double degrees)
{
	while ( degrees >= 360.0l ) degrees -= 360.0l ;
	while ( degrees < 0.0l ) degrees += 360.0l ;

	return degrees ;
}

double YawForHeading(double heading)
{
    double yaw = 90.0 - heading;
    yaw = ReturnPlusMinus180(yaw);
    return yaw;
}

double HeadingForYaw(double yaw)
{
    double heading = 90.0 - yaw;
    heading = Return0to360(heading);
    return heading;
}

double AngleDifference(double ang1, double ang2)
{
    double delta_deg = ang2 - ang1;
    delta_deg = ReturnPlusMinus180(delta_deg);
    return delta_deg;
}

// check the accuracy of this function
void ComputeRelativeVelocityPhi(double RefVel, double RefPhi, double TgtVel, double TgtPhi, double* dRelVel, double * dRelPhi ){


	// Headings are in 0-360 mode measured from North clockwise
    double dRefPhiInRad = RefPhi*DEG_TO_RAD;
    double dTgtPhiInRad = TgtPhi*DEG_TO_RAD;

    *dRelVel = sqrt(pow(TgtVel*cos(dTgtPhiInRad)-RefVel*cos(dRefPhiInRad),2)+pow(TgtVel*sin(dTgtPhiInRad)-RefVel*sin(dRefPhiInRad),2));
    double dAng, dAngInRad;
    double dx=0; double dy=0;
    if (fabs(*dRelVel) < 0.1){
    	dAngInRad = dRefPhiInRad;
    }
    else{
    	dx = TgtVel*sin(dTgtPhiInRad)-RefVel*sin(dRefPhiInRad);
    	dy = TgtVel*cos(dTgtPhiInRad)-RefVel*cos(dRefPhiInRad);
    	dAngInRad = atan2(dx,dy);
    }

    dAng = Return0to360(dAngInRad*RAD_TO_DEG);
    double dAngDiff = dAng-RefPhi; // This can take values between -360 to +360
    if (dAngDiff < 0) dAngDiff = 360+dAngDiff;
    *dRelPhi = dAngDiff;

    printf("Angle Calculations: Input : RefV:%2.1f, RefPhi:%2.1f, TgtV:%2.1f, TgtPhi:%2.1f\n"
           "Output: RelVel:%2.1f RelPhi:%2.1f, RelAnglFromN:%2.1f\n", RefVel, RefPhi, TgtVel,TgtPhi,*dRelVel,*dRelPhi, dAng);

    /*
    if (*dRelPhi > 0){
        if (*dRelPhi>PI) {*dRelPhi = -1*(2*PI-*dRelPhi);}
    }
    else{
        if (*dRelPhi<-PI) {*dRelPhi = 2*PI+*dRelPhi;}
    }
    *dRelPhi = *dRelPhi*RAD_TO_DEG;
    */
}
