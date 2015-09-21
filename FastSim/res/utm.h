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
* @file         utm.h
* @author       Tianyu Gu
* @date         06/10/2013
*/


#ifndef _UTM_H_
#define _UTM_H_

#include <proj_api.h>
#include <cstdio>
#include <sstream>

void LatLonToUtmWGS84(double lat, double lon, double & northing, double & easting, int zone);
void UtmToLatLonWGS84(double & lat, double & lon, double northing, double easting, int zone);

#endif
