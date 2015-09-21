/**
---------------------------------------------------------------------------
Copyright  (c) 2010-2013 Carnegie Mellon University,
All rights reserved.

This source code was developed in part with support from sponsors including
General Motors, the National Science Foundation and the US Department of
Transportation.

Use of this software is strictly governed by agreements between Carnegie
Mellon University and various sponsors.

Users of this software must also be fully aware and agree that Carnegie
Mellon does not guarantee the correct functioning of this software in any
system. Carnegie Mellon or any of its affiliates will not be liable for
any damage and/or penalties resulting from the use of this software.
Any user of this software takes complete responsibility for the use of any
software or design.
---------------------------------------------------------------------------
* @file         utm.cc
* @author       Tianyu Gu
* @date         06/10/2013
*/

#include "utm.h"

static projPJ pj_utm,pj_latlong;
static int pj_utm_zone = -1;

bool need_init(int zone)
{
    if(pj_utm_zone == -1 || pj_utm_zone != zone)
        return true;

    return false;
}

void init_proj(int zone)
{
    std::stringstream ss;
    ss << zone;
    std::string utm_init_cmd = "+proj=utm +zone="+ ss.str() +" +ellps=WGS84";
    if(!(pj_utm = pj_init_plus(utm_init_cmd.c_str()))
    && zone > 0 && zone <= 60) {
      printf("could not init UTM\n");
      exit(1);
    }
    pj_utm_zone = zone;

    if(!(pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84"))) {
      printf("could not init lat long\n");
      exit(1);
    }
}

void LatLonToUtmWGS84(double lat, double lon, double & northing, double & easting, int zone)
{
    if(need_init(zone))
    {
        init_proj(zone);
    }

    double x = lat * DEG_TO_RAD;
    double y = lon * DEG_TO_RAD;

    pj_transform(pj_latlong,pj_utm,1,1,&y,&x,NULL);

    easting = y;
    northing = x;
}

void UtmToLatLonWGS84(double & lat, double & lon, double northing, double easting, int zone)
{
    if(need_init(zone))
    {
        init_proj(zone);
    }

    double x = easting;
    double y = northing;

    pj_transform(pj_utm,pj_latlong,1,1,&x,&y,NULL);

    lat = y / DEG_TO_RAD;
    lon = x / DEG_TO_RAD;
}
