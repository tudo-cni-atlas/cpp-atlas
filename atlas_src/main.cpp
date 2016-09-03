//
//  main.cpp
//  atlas
//
//  Created by Janis on 26.03.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <csignal>
#include <exception>

#include <armadillo>
#include "docopt.h"

#include "atlas_types.h"
#include "config.h"
#include "parser.h"
#include "tdoa.h"
#include "logger.h"
#include "reporter.h"


static const char USAGE[] =
    R"(atlas.

    Usage:
        atlas [-c CFILE] [--quiet | --verbose]
        atlas list [-c CFILE]
        atlas (-h | --help)
        atlas --version

        Options:
        -c CFILE            Specify config file [default: config.yaml]
        -v --verbose        Verbose output
        -q --quiet          Do not output
        -h --help           Show this usage screen.
        --version           Show version.
)";

void signal_handler(int signal)
{
    std::cerr << " Quitting..." << std::endl;
    exit (EXIT_SUCCESS);
}

int main(int argc, const char* argv[])
{
    std::signal(SIGINT, signal_handler);
    std::map<std::string, docopt::value> args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true, "atlas 1.0");

    std::string config_file = args.find("-c")->second.asString();
    Config config(config_file);

    if(args.find("list")->second.asBool())
    {
        std::cout << "Listing config file: " << config_file << std::endl;
        config.list();
    }

    Parser parser;
    parser.initialize(config.m_parser, config.m_anchors, config.m_sync);
    ClockCorrection cor;

    Logger log;
    Reporter rep(config.m_output.port);

    PositionerTDOA pos;
    pos.initialize(config.m_anchors, config.m_sync, config.m_lrf, config.m_ekf, config.m_cal);

    auto ts = std::chrono::high_resolution_clock::now();
    while (true)
    {
        parser.poll();

        std::vector<sample_t> s;
        parser.extractSamples(&s);

        cor.processSamples(&s);

        std::vector<sample_t> cs;
        cs = cor.getCorrectedSamples();

        for (auto it = cs.begin(); it != cs.end(); ++it)
        {
            sample_t s = *it;

            position_t p;
            if(pos.calculatePositionEKF(s, &p))
            {
                log.logSample(s);
                log.logPosition(p);
                rep.reportPosition(p);
            }

        }
    }

    return 0;
}
