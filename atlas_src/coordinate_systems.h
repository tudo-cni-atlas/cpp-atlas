//
//  coordinate_systems.h
//  atlas
//
//  Created by Janis on 18.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__coordinate_systems__
#define __atlas__coordinate_systems__

#include <stdio.h>
#include <armadillo>

arma::vec ecef2lla(arma::vec ecef);
arma::vec lla2ecef(arma::vec lla);
arma::vec enu2ecef(arma::vec ref_ecef, arma::vec enu);
arma::vec ecef2enu(arma::vec ref_ecef, arma::vec ecef);
arma::vec lrf2enu(arma::vec lrf, double angle);
double calculateAngle(arma::vec reflla, arma::vec lla);

#endif /* defined(__atlas__coordinate_systems__) */