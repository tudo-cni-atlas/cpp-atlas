//
//  tdoa.h
//  atlas
//
//  Created by Janis on 18.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__tdoa__
#define __atlas__tdoa__

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <map>

#include <armadillo>

#include "atlas_types.h"

typedef struct
{
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;
    arma::vec state;
    arma::mat stateCovariance;
    arma::mat stateTransition;
    arma::mat processNoiseCovariance;
} ekf_t;

class PositionerTDOA
{
private:
    // system parameters
    std::map<uint64_t, arma::vec> m_anchorPositions;
    arma::vec m_syncPosition;

    // local reference frame
    arma::vec m_ecefRef;
    double m_lrfAngle;

    // validation
    std::map<uint64_t, position_t> m_lastPosition;
    std::map<uint64_t, sample_t> m_lastSample;
    std::map<uint64_t, uint64_t> m_updateCount;

    // calibration
    std::map<uint64_t, double> m_calOffset;
    uint64_t m_calNode;
    arma::vec m_calPosition;
    double m_calIncrement;

    // ekf initialization and state
    std::map<uint64_t, ekf_t> m_ekf;
    double m_processNoise;
    double m_measurementNoise;
    double m_initialStateVariance;
    double m_initialStateVarianceDelta;
    double m_initialInterval;
    arma::vec m_initialState;

    // outlier detection
    double m_outlierThreshold;
    double m_outlierThresholdDelta;

public:
    PositionerTDOA ();

    void initialize(const configAnchors_t &anchors, const configSync_t &sync, const configLRF_t &lrf, const configEKF_t &ekf, const configCal_t &cal);
    void createNewEKF(uint64_t eui, std::chrono::time_point<std::chrono::system_clock> ts);

    bool calculatePositionEKFInner (const sample_t &s, position_t *p);
    bool calculatePositionEKF (const sample_t &s, position_t *p);
};

#endif /* defined(__atlas__tdoa__) */
