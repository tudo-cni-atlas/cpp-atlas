//
//  coordinate_systems.cpp
//  atlas
//
//  Created by Janis on 18.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "coordinate_systems.h"

#include <stdio.h>
#include <math.h>
#include <vector>

arma::vec ecef2lla(arma::vec ecef)
{
    double a = 6378137;
    double e = 8.1819190842622e-2;

    double x = ecef.at(0);
    double y = ecef.at(1);
    double z = ecef.at(2);

    double b = sqrt(pow(a, 2) * (1 - pow(e, 2)));
    double ep = sqrt((pow(a, 2) - pow(b, 2)) / pow(b, 2));
    double p = sqrt(pow(x, 2) + pow(y, 2));
    double th = atan2(a * z, b * p);
    double lon = atan2(y, x);
    double lat = atan2(z + pow(ep, 2) * b * pow(sin(th), 3), p - pow(e, 2) * a * pow(cos(th), 3));
    double N = a / sqrt(1 - pow(e, 2) * pow(sin(lat), 2));
    double alt = p / cos(lat) - N;

    lat = (lat * 180) / M_PI;
    lon = (lon * 180) / M_PI;

    arma::vec lla = { lat, lon, alt };

    return lla;
}

arma::vec lla2ecef(arma::vec lla)
{
    double a = 6378137;
    double e = 8.1819190842622e-2;

    double lat = (lla.at(0) * M_PI) / 180;
    double lon = (lla.at(1) * M_PI) / 180;
    double alt = lla.at(2);

    double N = a / sqrt(1 - pow(e, 2) * pow(sin(lat), 2));

    double x = (N + alt) * cos(lat) * cos(lon);
    double y = (N + alt) * cos(lat) * sin(lon);
    double z = ((1 - pow(e, 2)) * N + alt) * sin(lat);

    arma::vec ecef = { x, y, z };

    return ecef;
}

arma::vec enu2ecef(arma::vec ref_ecef, arma::vec enu)
{
    arma::vec lla = ecef2lla(ref_ecef);

    double lat = (lla.at(0) * M_PI) / 180;
    double lon = (lla.at(1) * M_PI) / 180;
    //double alt = lla.at(2);

    double e = enu.at(0);
    double n = enu.at(1);
    double u = enu.at(2);

    double xr = ref_ecef.at(0);
    double yr = ref_ecef.at(1);
    double zr = ref_ecef.at(2);

    double x = -sin(lon) * e - sin(lat) * cos(lon) * n + cos(lat) * cos(lon) * u + xr;
    double y = cos(lon) * e - sin(lat) * sin(lon) * n + cos(lat) * sin(lon) * u + yr;
    double z = cos(lat) * n + sin(lat) * u + zr;

    arma::vec ecef = { x, y, z };

    return ecef;
}

arma::vec ecef2enu(arma::vec ref_ecef, arma::vec ecef)
{
    arma::vec lla = ecef2lla(ref_ecef);

    double lat = (lla.at(0) * M_PI) / 180;
    double lon = (lla.at(1) * M_PI) / 180;

    double x = ecef.at(0);
    double y = ecef.at(1);
    double z = ecef.at(2);

    double xr = ref_ecef.at(0);
    double yr = ref_ecef.at(1);
    double zr = ref_ecef.at(2);

    double e = -sin(lon) * (x - xr) + cos(lon) * (y - yr);
    double n = -sin(lat) * cos(lon) * (x - xr) - sin(lat) * sin(lon) * (y - yr) + cos(lat) * (z - zr);
    double u = cos(lat) * cos(lon) * (x - xr) + cos(lat) * sin(lon) * (y - yr) + sin(lat) * (z - zr);

    arma::vec enu = { e, n, u };

    return enu;
}

arma::vec lrf2enu(arma::vec lrf, double angle)
{
    double x = lrf.at(0);
    double y = lrf.at(1);
    double z = lrf.at(2);

    double phi = (angle * M_PI) / 180;

    double e = cos(phi) * x - sin(phi) * y;
    double n = sin(phi) * x + cos(phi) * y;
    double u = z;

    arma::vec enu = { e, n, u };

    return enu;
}

double calculateAngle(arma::vec reflla, arma::vec lla)
{
    arma::vec refecef = lla2ecef(reflla);
    arma::vec ecef = lla2ecef(lla);
    arma::vec enu = ecef2enu(refecef, ecef);

    enu.at(2) = 0;
    arma::vec refenu = { 1, 0, 0 };

    double scal = refenu.at(0) * enu.at(0) + refenu.at(1) * enu.at(1) + refenu.at(2) * enu.at(2);
    double sum1 = sqrt(pow(refenu.at(0), 2) + pow(refenu.at(1), 2) + pow(refenu.at(2), 2));
    double sum2 = sqrt(pow(enu.at(0), 2) + pow(enu.at(1), 2) + pow(enu.at(2), 2));
    double phi = acos(scal / (sum1 + sum2));
    phi = -((phi * 180) / M_PI);

    return phi;
}