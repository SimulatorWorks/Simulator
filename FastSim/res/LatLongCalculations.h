
#ifndef  _LATLONGCALCULATIONS_H_
#define  _LATLONGCALCULATIONS_H_

#include <math.h>

#ifndef  PI
#define	PI          3.1415926535897932384626433832795
#endif

#ifndef  DEG_TO_RAD
#define	DEG_TO_RAD	(PI/180.0)
#endif

#ifndef  RAD_TO_DEG
#define	RAD_TO_DEG	(180.0/PI)
#endif

#ifndef  KPH_TO_MPS
#define	KPH_TO_MPS	(1000.0/3600.0)
#endif

#ifndef  KPH_TO_MPH
#define  KPH_TO_MPH  (0.621371192237)
#endif

#ifndef  MPS_TO_KPH
#define	MPS_TO_KPH	(3600.0/1000.0)
#endif

#ifndef  MPS_TO_MPH
#define	MPS_TO_MPH	(2.23693629205)
#endif

int CalcDistAzimuth( double hostLat,
                    double hostLon,
                    double hostHeading,
                    double povLat,
                    double povLon,
                    double povHeading,
                    double *pRange,
                    double *pAzimuth ) ;



int CalcDist( double Lat1,
             double Lon1,
             double Lat2,
             double Lon2,
             double *pRange ) ;



int CalcAzimuth( double Lat1,
                double Lon1,
                double Heading1,
                double Lat2,
                double Lon2,
                double Heading2,
                double *pAzimuth ) ;



int CalcBearing( double Lat1,
                double Lon1,
                double Lat2,
                double Lon2,
                double *pBearing ) ;



int ConvertLatLonToNE( double latref,
                      double lonref,
                      double lat,
                      double lon,
                      double *pE,
                      double *pN) ;
/* please do not use this function, it has been replaced by ConvertLatLonToNE */
int ConvertLatLonToXY( double latref,
                      double lonref,
                      double lat,
                      double lon,
                      double *pE,
                      double *pN) ;



/* transforms the latitude and longitude offset from the host position
   to XY coordinates for the vehicle where X points in the direction of travel
   and Y points towards the passenger side with the z-axis pointint into the
   earth in a right-hand coordinate system

   All angles should be in degrees.
   X and Y will be returned in meters
*/
int ConvertLatLonToXYVehicleCoords( double latref,
                      double lonref,
                      double heading,
                      double latDelta,
                      double lonDelta,
                      double *pX,
                      double *pY) ;



int ConvertNEToLatLon( double E,
                      double N,
                      double latref,
                      double lonref,
//                      double headingref,
                      double *pLat,
                      double *pLon) ;
/* please do not use this function, it has been replaced by ConvertNEToLatLon */
int ConvertXYToLatLon( double E,
                      double N,
                      double latref,
                      double lonref,
                      double *pLat,
                      double *pLon) ;

int GetEarthRadius( double latref,
                   double lonref,
                   double *pRadius ) ;

double	Return0to360(double degrees) ;
double	ReturnPlusMinus180(double degrees) ;
double YawForHeading(double heading) ;
double HeadingForYaw(double yaw) ;
double AngleDifference(double ang1, double ang2);
void ComputeRelativeVelocityPhi(double RefVel, double RefPhi, double TgtVel, double TgtPhi, double* relVel, double * relPhi );

#endif   // _LATLONGCALCULATIONS_H_

