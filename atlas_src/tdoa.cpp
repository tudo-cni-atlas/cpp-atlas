//
//  tdoa.cpp
//  atlas
//
//  Created by Janis on 18.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "tdoa.h"

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>

#include "coordinate_systems.h"


PositionerTDOA::PositionerTDOA ()
{

}

void PositionerTDOA::initialize(const configAnchors_t &anchors, const configSync_t &sync, const configLRF_t &lrf, const configEKF_t &ekf, const configCal_t &cal)
{
    std::cout << "Initializing PositionerTDOA..." << std::endl;
    std::cout << "Config ekf: " << ekf.processNoise << "," << ekf.measurementVariance << ","
              << ekf.initialInterval << "," << ekf.initialVariance << "," << ekf.initialVarianceDelta << std::endl;

    m_anchorPositions = anchors.positions;
    m_syncPosition = sync.position;

    m_ecefRef = lla2ecef(lrf.wgs84ref);
    m_lrfAngle = calculateAngle(lrf.wgs84ref, lrf.wgs84axis);
    std::cout << "LRF Angle: " << m_lrfAngle << std::endl;

    m_processNoise = ekf.processNoise;
    m_measurementNoise = ekf.measurementVariance;
    m_initialInterval = ekf.initialInterval;
    m_initialStateVariance = ekf.initialVariance;
    m_initialStateVarianceDelta = ekf.initialVarianceDelta;
    m_initialState = ekf.initialState;

    m_outlierThreshold = 15;
    m_outlierThresholdDelta = 0.5;

    m_calNode = cal.eui;
    m_calIncrement = 1.0/cal.window;
    m_calPosition = cal.position;
}

void PositionerTDOA::createNewEKF(uint64_t eui, std::chrono::time_point<std::chrono::system_clock> ts)
{
    ekf_t ekf;
    ekf.lastUpdate = ts;

    ekf.state = m_initialState;
    //std::cout << "state:" << std::endl;
    //std::cout << state << std::endl;

    arma::mat stateTransition = arma::eye<arma::mat>(6, 6);
    stateTransition.diag(3).fill(m_initialInterval);
    ekf.stateTransition = stateTransition;
    //std::cout << "stateTransition:" << std::endl;
    //std::cout << stateTransition << std::endl;

    arma::vec dia;
    dia << m_initialStateVariance << m_initialStateVariance << m_initialStateVariance
        << m_initialStateVarianceDelta << m_initialStateVarianceDelta << m_initialStateVarianceDelta << arma::endr;
    arma::mat stateCovariance = arma::zeros<arma::mat>(6, 6);
    stateCovariance.diag() = dia;
    ekf.stateCovariance = stateCovariance;
    //std::cout << "stateCovariance:" << std::endl;
    //std::cout << ekf.stateCovariance << std::endl;

    m_ekf.insert(std::make_pair(eui, ekf));
}

bool PositionerTDOA::calculatePositionEKFInner(const sample_t &s, position_t *p)
{
    int count = s.meas.size();
    arma::mat anchorPositions = arma::zeros<arma::mat>(count, 3);
    arma::vec anchorTOAs = arma::zeros<arma::vec>(count);

    if (s.txeui == m_calNode)
    {
        // distance of first anchor to calibration
        double firstAnchorDistance;
        double firstToa;

        int row = 0;
        for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
        {
            arma::vec vds = m_anchorPositions[it->first] - m_syncPosition;
            double syncAnchorDistance = sqrt(arma::accu(arma::square(vds)));
            double toa = it->second.toa * 299792458 + syncAnchorDistance;

            arma::vec vdc = m_anchorPositions[it->first] - m_calPosition;
            double anchorDistance = sqrt(arma::accu(arma::square(vdc)));

            if(row == 0)
            {
                arma::vec vd = m_anchorPositions[it->first] - m_calPosition;
                firstAnchorDistance = sqrt(arma::accu(arma::square(vd)));
                firstToa = toa;
            }

            double expectedToa = anchorDistance - firstAnchorDistance;
            double relativeToa = toa - firstToa;
            double diffToa = relativeToa - expectedToa;

            if(m_calOffset.find(s.txeui) == m_calOffset.end())
            {
                m_calOffset.insert(std::make_pair(s.txeui, 0.0));
            }

            m_calOffset[it->first] = m_calOffset[it->first] - (m_calOffset[it->first] - diffToa) * m_calIncrement;

            std::cout << std::hex << it->first << " expected: " << expectedToa << ", toa: " << relativeToa << ", diff: " << diffToa << ", off: " << m_calOffset[it->first] << std::endl;
            row++;
        }
    }
    
    int row = 0;
    for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
    {
        anchorPositions.row(row) = m_anchorPositions[it->first].t();

        arma::vec vd = m_anchorPositions[it->first] - m_syncPosition;
        double d = sqrt(arma::accu(arma::square(vd)));

        if(m_calOffset.find(it->first) != m_calOffset.end())
        {
            d = d - m_calOffset[it->first];
        }

        anchorTOAs(row) = it->second.toa * 299792458 + d;
        row++;
    }

    // --- get ekf state for specific participant ---
    ekf_t ekf = m_ekf[s.txeui];
    arma::vec xk = ekf.state;
    arma::mat Pk = ekf.stateCovariance;

    // --- create dynamic state transition matrix ---
    double interval = m_initialInterval;
    if(m_lastPosition.find(p->eui) != m_lastPosition.end())
    {
        interval = std::chrono::duration<double>(s.hts - m_lastPosition[p->eui].hts).count();
        //std::cout << "interval: " << interval << std::endl;
    }
    arma::mat Fk = arma::eye<arma::mat>(6, 6);
    Fk.diag(3).fill(interval);

    // --- create dynamic process noise covariance matrix ---
    arma::mat Q;
    arma::mat pt = arma::eye<arma::mat>(3, 3) * pow(interval, 2) / 2;
    arma::mat pb = arma::eye<arma::mat>(3, 3) * interval;
    arma::mat pcv = arma::join_vert(pt, pb);
    Q = pcv * (arma::eye<arma::mat>(3, 3) * m_processNoise) * pcv.t();
    //ekf.processNoiseCovariance = Q;
    //std::cout << "Q:" << std::endl << Q << std::endl;

    // --- prediction ---
    // estimate state
    arma::mat exk = Fk * xk;
    //std::cout << "exk:" << std::endl << exk << std::endl;

    // estimate covariance
    arma::mat ePk = Fk * Pk * Fk.t() + Q;
    //std::cout << "ePk:" << std::endl << ePk << std::endl;

    // --- correction ---
    arma::vec referenceAnchor = anchorPositions.row(0).t();

    // observation vector
    arma::vec td = anchorTOAs - anchorTOAs(0);
    arma::vec dv = td.subvec(1, count - 1);
    //std::cout << "dv:" << std::endl << dv << std::endl;

    arma::vec ep = xk.rows(0, 2);
    //std::cout << "ep:" << std::endl << ep << std::endl;

    arma::mat epmat = arma::repmat(ep.t(), count - 1, 1);
    //std::cout << "epmat:" << std::endl << epmat << std::endl;

    arma::mat refmat = arma::repmat(referenceAnchor.t(), count - 1, 1);
    //std::cout << "refmat:" << std::endl << refmat << std::endl;

    arma::mat anmat = anchorPositions.rows(1, count - 1);
    //std::cout << "anmat:" << std::endl << anmat << std::endl;

    arma::mat ta1 = epmat - anmat;
    arma::mat ta2 = epmat - refmat;
    //std::cout << "ta1:" << std::endl << ta1 << std::endl << "ta2:" << std::endl << ta2 << std::endl;

    arma::mat distanceToAnchors = arma::sqrt(arma::sum(arma::square(ta1), 1));
    arma::mat distanceToReference = arma::sqrt(arma::sum(arma::square(ta2), 1));
    arma::mat ta1dr = arma::repmat(distanceToAnchors, 1, 3);
    arma::mat ta2dr = arma::repmat(distanceToReference, 1, 3);
    arma::mat t1 = ta1 / ta1dr;
    arma::mat t2 = ta2 / ta2dr;
    arma::mat j = t1 - t2;
    //std::cout << "ta1dr:" << std::endl << ta1dr << std::endl << "ta2dr:" << std::endl << ta2dr << std::endl;
    //std::cout << "t1:" << std::endl << t1 << std::endl << "t2:" << std::endl << t2 << std::endl << "j:" << std::endl << j << std::endl;

    arma::mat Hk = arma::join_horiz( j, arma::zeros<arma::mat>(count - 1, 3) );
    //std::cout << "Hk:" << std::endl << Hk << std::endl;

    // innovation
    arma::vec expectedMeasurement = distanceToAnchors - distanceToReference;
    arma::vec observedMeasurement = dv;
    arma::vec innovation = observedMeasurement - expectedMeasurement;
    //std::cout << "innovation:" << std::endl << innovation << std::endl;

    arma::mat Rk;
    Rk = arma::zeros<arma::mat>(count - 1, count - 1);
    Rk.diag().fill(m_measurementNoise);
    //std::cout << "Rk:" << std::endl << Rk << std::endl;

    // covariance of the innovation
    arma::mat Sk = Hk * ePk * Hk.t() + Rk;
    //std::cout << "Sk:" << std::endl << Sk << std::endl;

    // kalman gain computation
    arma::mat Kk = ePk * Hk.t() * Sk.i();
    //std::cout << "Kk:" << std::endl << Kk << std::endl;

    // --- state update ---
    // a posteriori state estimate
    xk = exk + Kk * innovation;
    //std::cout << "xk:" << std::endl << xk << std::endl;

    // a posteriori state covariance
    Pk = (arma::eye<arma::mat>(6, 6) - Kk * Hk) * ePk;
    //std::cout << "Pk:" << std::endl << Pk << std::endl;

    ekf.state = xk;
    ekf.stateCovariance = Pk;

    m_ekf[s.txeui] = ekf;

    p->pos = xk.rows(0, 2);
    p->dpos = xk.rows(3, 5);

    return true;
}

bool PositionerTDOA::calculatePositionEKF(const sample_t &s, position_t *p)
{
    if(s.meas.size() < 8)
    {
        return false;
    }

    if(m_ekf.find(s.txeui) == m_ekf.end())
    {
        createNewEKF(s.txeui, s.hts);
        m_updateCount.insert(std::make_pair(s.txeui, 0));
        m_lastSample.insert(std::make_pair(s.txeui, s));
        return false;
    }

    m_updateCount[s.txeui] = m_updateCount[s.txeui] + 1;
    if(m_updateCount[s.txeui] < 2)
    {
        return false;
    }

    // outlier detection
    double lastToa = m_lastSample[s.txeui].meas.begin()->second.toa * 299792458;
    double toa = s.meas.begin()->second.toa * 299792458;

    bool discard = false;
    for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
    {
        if(m_lastSample[s.txeui].meas.find(it->first) != m_lastSample[s.txeui].meas.end())
        {
            double lastTdoa = m_lastSample[s.txeui].meas[it->first].toa * 299792458 - lastToa;
            double tdoa = it->second.toa * 299792458 - toa;
            double diff = fabs(lastTdoa - tdoa);

            if(diff > m_outlierThresholdDelta)
            {
                std::cout << "Warning - Outlier: TDOA diff " << s.txeui << "," << it->first << " exceeded d: " << diff << std::endl;
                discard = true;
            }

            if(tdoa > m_outlierThreshold || tdoa < -m_outlierThreshold)
            {
                std::cout << "Warning - Outlier: TDOA large " << s.txeui << "," << it->first << " exceeded threshold: " << tdoa << std::endl;
                discard = true;
            }
        }
    }
    m_lastSample[s.txeui] = s;

    if(!discard)
    {
        if(!calculatePositionEKFInner(s, p))
        {
            return false;
        }
    }

    if(discard)
    {
        return false;
    }

    // set basic position result props
    p->hts = s.hts;
    p->eui = s.txeui;
    p->num_s = s.meas.size();

    // coordinate conversions
    p->enu = lrf2enu(p->pos, m_lrfAngle);
    p->ecef = enu2ecef(m_ecefRef, p->enu);
    p->wgs84 = ecef2lla(p->ecef);

    // calculate heading and velocities
    arma::vec denu = lrf2enu(p->dpos, m_lrfAngle);
    double vG = sqrt( pow(denu.at(0), 2) + pow(denu.at(1), 2) );
    double v = sqrt( pow(denu.at(0), 2) + pow(denu.at(1), 2) + pow(denu.at(2), 2) );
    double heading = 0;
    if (v != 0)
    {
        double h = atan2(denu.at(0), denu.at(1));
        if(denu.at(0) < 0)
        {
            h = h + 2 * M_PI;
        }
        heading = ((h * 180) / M_PI);
    }

    std::chrono::duration<double> dtc = p->hts - m_lastPosition[p->eui].hts;
    double dt = dtc.count();
    p->wgs84_heading = heading;
    p->wgs84_speed = vG * dt;

    m_lastPosition[p->eui] = *p;

    return true;
}