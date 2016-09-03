//
//  parser.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__parser_tdoa__
#define __atlas__parser_tdoa__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <armadillo>

#include "protocol.h"
#include "atlas_types.h"

class Parser
{
private:
    const uint64_t ticksPerRevolution = 0x10000000000;
    std::map<uint64_t, protocol::Protocol*> m_decoders;
    std::map<uint64_t, std::map<uint64_t, std::map<uint64_t, measurement_t> > > m_measurements;
    std::map<uint64_t, uint64_t> m_lastSequence;
    std::map<uint64_t, uint64_t> m_lastTimestamp;
    std::map<uint64_t, uint64_t> m_tagWhitelist;

public:
    Parser ();
    ~Parser();

    void initialize(const configParser_t &parser, const configAnchors_t &anchors, const configSync_t &sync);
    void poll();
    void extractSamples(std::vector<sample_t> *s);
};

class ClockModel
{
private:
    const uint64_t ticksPerSecond = 128 * 499.2e6;
    const uint64_t ticksPerInterval = 128 * 499.2e5;
    double m_lastSync;
    double m_lastOffset;
    double m_lastDrift;

public:
    ClockModel();
    ~ClockModel();

    void processSynchronizationFrame(uint64_t seq, uint64_t ts);
    double getCorrectedTOA(uint64_t ts);
};

class ClockCorrection
{
private:
    const uint64_t syncEUI = 0xdeca030000000001;
    std::map<uint64_t, ClockModel> m_anchorClocks;
    std::vector<sample_t> m_samples;
    std::chrono::time_point<std::chrono::system_clock> m_lastSync;

    uint64_t m_lastSequence;

public:
    ClockCorrection();
    ~ClockCorrection();

    void processSample(sample_t sample);
    void processSamples(std::vector<sample_t> *samples);
    std::vector<sample_t> getCorrectedSamples();
};


#endif /* defined(__atlas__parser_tdoa__) */