//
//  logger.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "logger.h"

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <sys/types.h>

#include <chrono>
#include <ctime>

#include <list>
#include <map>
#include <string>

#include "atlas_types.h"

Logger::Logger ()
{
    /* create directory name from current date */
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    char buf[sizeof "log/2011-10-08_07-07-07"];
    strftime(buf, sizeof buf, "log/%Y-%m-%d_%H-%M-%S", gmtime(&in_time_t));

    std::cout << "Creating logfiles: " << buf << std::endl;

    /* keep directory for future use */
    std::string dir(buf);
    m_dir = dir;

    m_positionLog.open(dir + "_positions.csv");
    m_positionLog << "iso,unix,eui,x,y,z,dx,dy,dz,ecef_x,ecef_y,ecef_z,wgs84_lat,wgs84_lon,wgs84_alt,wgs84_heading,wgs84_speed" << std::endl;

    m_sampleLog.open(dir + "_samples.csv");
    m_sampleLog << "iso,unix,txeui,seq,rxeui,toa,ts" << std::endl;
}

Logger::~Logger()
{
    m_positionLog.close();
    m_sampleLog.close();
}

void Logger::logPosition(const position_t &p)
{
    auto in_time_t = std::chrono::system_clock::to_time_t(p.hts);
    char buf[sizeof "2011-10-08_07-07-07"];
    strftime(buf, sizeof buf, "%Y-%m-%d_%H-%M-%S", gmtime(&in_time_t));
    std::string iso(buf);

    auto d = p.hts.time_since_epoch();
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(d).count();
    double unix = micros / 1000000.0;

    m_positionLog << iso << "," << std::setprecision(6) << std::fixed << unix << ",";
    m_positionLog << std::hex << p.eui << "," << std::dec << std::setprecision(3) << std::fixed << p.pos.at(0) << "," << p.pos.at(1) << "," << p.pos.at(2) << "," << p.dpos.at(0) << "," << p.dpos.at(1) << "," << p.dpos.at(2) << ",";
    m_positionLog << std::setprecision(12) << std::fixed << p.wgs84.at(0) << "," << p.wgs84.at(1) << "," << std::setprecision(3) << std::fixed << p.wgs84.at(2) << "," << p.wgs84_heading << "," << p.wgs84_speed << ",";
    m_positionLog << std::endl;
}

void Logger::logSample(const sample_t &s)
{
    auto in_time_t = std::chrono::system_clock::to_time_t(s.hts);
    char buf[sizeof "2011-10-08_07-07-07"];
    strftime(buf, sizeof buf, "%Y-%m-%d_%H-%M-%S", gmtime(&in_time_t));
    std::string iso(buf);

    for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
    {
        auto d = it->second.hts.time_since_epoch();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(d).count();
        double unix = micros / 1000000.0;

        m_sampleLog << iso << "," << std::setprecision(6) << std::fixed << unix << ",";
        m_sampleLog << std::hex << s.txeui << "," << std::dec << s.seq << "," << std::hex << it->second.rxeui << "," << std::setprecision(12)  << std::fixed << it->second.toa << "," << std::dec << it->second.ts << ",";
        m_sampleLog << std::endl;
    }
}