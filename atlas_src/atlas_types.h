//
//  atlas_types.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__atlas_types__
#define __atlas__atlas_types__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include <armadillo>

typedef struct
{
    std::chrono::time_point<std::chrono::system_clock> hts;
    uint64_t ts;
    uint64_t txeui;
    uint64_t rxeui;
    uint64_t seq;
    double toa;
    double fpPower;
    double rxPower;
    double fpRatio;
} measurement_t;

typedef struct
{
    std::chrono::time_point<std::chrono::system_clock> hts;
    uint64_t txeui;
    uint64_t seq;
    std::map<uint64_t, measurement_t> meas;
} sample_t;

typedef struct
{
    std::chrono::time_point<std::chrono::system_clock> hts;
    uint64_t eui;
    arma::vec pos;
    arma::vec dpos;
    arma::vec enu;
    arma::vec ecef;
    arma::vec wgs84;
    double wgs84_heading;
    double wgs84_speed;
    double gdop;
    double tdop;
    double xdop;
    double ydop;
    double hdop;
    double vdop;
    int32_t num_s;
} position_t;


typedef struct
{
    std::map<uint64_t, arma::vec> positions;
    std::map<uint64_t, std::string> ports;
} configAnchors_t;

typedef struct
{
    uint64_t eui;
    uint64_t interval;
    std::string port;
    int baudrate;
    arma::vec position;
} configSync_t;

typedef struct
{
    uint64_t eui;
    uint64_t window;
    arma::vec position;
} configCal_t;

typedef struct
{
    std::map<uint64_t, uint64_t> tagWhitelist;
} configParser_t;

typedef struct
{
    double processNoise;
    double measurementVariance;
    double initialInterval;
    double initialVariance;
    double initialVarianceDelta;
    arma::vec initialState;
} configEKF_t;

typedef struct
{
    arma::vec wgs84ref;
    arma::vec wgs84axis;
} configLRF_t;

typedef struct
{
    std::string port;
    int baudrate;
} configOutput_t;

typedef struct
{
    bool verbose;
    bool quiet;
} configVerbosity_t;


#endif /* defined(__atlas__atlas_types__) */