//
//  reporter.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "reporter.h"

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>

#include <chrono>
#include <string>

#include "atlas_types.h"


Reporter::Reporter (const std::string port)
    : m_port(port, 115200)
{

}

Reporter::~Reporter()
{

}

void Reporter::reportPosition(const position_t &p)
{
    std::cout << p.eui << ", " << std::setprecision(3) << std::fixed << p.pos.at(0) << ", " << p.pos.at(1) << ", " << p.pos.at(2) << std::endl;

    auto d = p.hts.time_since_epoch();
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(d).count();
    double unix = micros / 1000000.0;

    std::stringstream report;
    report << "atlas," << std::hex << p.eui << "," << std::dec;
    report << std::setprecision(3) << std::fixed << p.pos.at(0) << "," << p.pos.at(1) << "," << p.pos.at(2) << ",";
    report << std::setprecision(9) << std::fixed << p.wgs84.at(0) << "," << p.wgs84.at(1) << "," << std::setprecision(3) << p.wgs84.at(2) << "\n";

    char msg[1024];
    strncpy(msg, report.str().c_str(), sizeof(msg));
    msg[sizeof(msg) - 1] = 0;

    m_port.send((const uint8_t*)msg, report.str().length());
}
