//
//  parser.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "parser.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>
#include <map>
#include <string>

#include "atlas_types.h"
#include "config.h"
#include "protocol.h"
#include "protocol_structures.h"

typedef enum
{
    CFG_ROLE_LISTENER = 0,
    CFG_ROLE_TAG,
    CFG_ROLE_ANCHOR,
    CFG_ROLE_TAG_TDOA,
    CFG_ROLE_NUM_MODES,
    CFG_ROLE_SYNC_ANCHOR
} cfgRole_t;

Parser::Parser()
{

}

Parser::~Parser()
{

}

void Parser::initialize(const configParser_t &p, const configAnchors_t &a, const configSync_t &s)
{
    std::cout << "Initializing Parser..." << std::endl;
    std::cout << "Whitelist: ";
    for (auto it = p.tagWhitelist.begin(); it != p.tagWhitelist.end(); it++)
    {
        std::cout << std::hex << it->first << ",";
    }
    std::cout << std::endl;
    //std::cout << "Sync: " << s.port << "," << std::dec << s.baudrate << "," << std::hex << s.eui << "," << std::dec << s.interval << std::endl;

    std::cout << "Sync Configuration:" << std::endl;
    protocol::Protocol *sync = new protocol::Protocol(s.port);
    protocol::msgConfig_t config;
    sync->sendMsgPoll(&config);
    //std::cout << std::hex << s.eui << " Config: " << (int)config.role << ", " << (uint64_t)config.eui << ", " << (int)config.channel << std::endl;

    protocol::msgPeriod_t period;
    sync->sendMsgPoll(&period);
    //std::cout << std::hex << s.eui << " Periods: " << (int)period.syncPeriod << ", " << (int)period.tagPeriod << std::endl;

    config.role = CFG_ROLE_SYNC_ANCHOR;
    config.eui = s.eui;
    config.channel = 0x20;
    sync->sendMsg(config);

    //std::cout << s.eui << " configured to role SYNC_ANCHOR and channel" << (int)config.channel << std::endl;
    usleep(10000);

    std::cout << "Anchor Configuration:" << std::endl;
    for (auto it = a.ports.begin(); it != a.ports.end(); ++it)
    {
        m_decoders.insert(std::make_pair(it->first, new protocol::Protocol(it->second)));
    }
    for (auto it = m_decoders.begin(); it != m_decoders.end(); it++)
    {
        protocol::msgConfig_t config;
        it->second->sendMsgPoll(&config);
        //std::cout << it->first << " Config: " << std::hex << (int)config.role << ", " << (uint64_t)config.eui << ", " << (int)config.channel << std::endl;

        protocol::msgPeriod_t period;
        it->second->sendMsgPoll(&period);
        //std::cout << it->first << " Periods: " << std::hex << (int)period.syncPeriod << ", " << (int)period.tagPeriod << std::endl;

        config.role = CFG_ROLE_ANCHOR;
        std::string::size_type sz = 0;
        //config.eui = std::stoull(it->first, &sz, 16);
        config.eui = it->first;
        config.channel = 0x20;
        it->second->sendMsg(config);

        //std::cout << it->first << " configured to role ANCHOR and channel" << (int)config.channel << std::endl;
        usleep(10000);
    }

    m_tagWhitelist = p.tagWhitelist;
}


void Parser::poll()
{
    for (auto it = m_decoders.begin(); it != m_decoders.end(); ++it)
    {
        protocol::packet_t packet;
        if(it->second->poll(&packet))
        {
            if (it->second->calculateCheckSum(&packet) != packet.checksum)
            {
                std::cout << it->first << " CHECKSUM ERROR!!! ";
                std::cout << std::hex << "x" << packet.messageId << ", x" << packet.payloadLength << ", x" << packet.checksum << ", x" << it->second->calculateCheckSum(&packet);
                std::cout << " payload:";
                for(uint16_t i = 0; i < packet.payloadLength; ++i)
                {
                    std::cout << std::hex << " x" << (int)packet.payload[i];
                }
                std::cout << std::endl;
            }
            else
            {
                protocol::msgToa_t msg;
                memcpy(&msg, packet.payload, sizeof(protocol::msgToa_t));

                //std::cout << it->first << " Received packet ";
                //std::cout << std::hex << msg.txId << ", " << msg.rxId << ", x" << msg.rxTs << ", x" << (int)msg.seqNr << std::endl;

                // Linearize receive Timestamps
                uint64_t ts = msg.rxTs;
                uint64_t rxeui = msg.rxId;

                if(m_lastTimestamp.find(rxeui) == m_lastTimestamp.end())
                {
                    m_lastTimestamp.insert(std::make_pair(rxeui, ts));
                }
                else
                {
                    int64_t lastts = m_lastTimestamp[rxeui];
                    int64_t res = lastts % ticksPerRevolution;

                    if((int64_t)ts - (int64_t)res > 0)
                    {
                        m_lastTimestamp[rxeui] = lastts + (ts - res);
                        //std::cout << "+ ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                    if((int64_t)ts - (int64_t)res < 0)
                    {
                        m_lastTimestamp[rxeui] = lastts + (ts - res) + ticksPerRevolution;
                        //std::cout << "- ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                }

                if(m_tagWhitelist.find(msg.txId) != m_tagWhitelist.end())
                {
                    measurement_t meas;
                    meas.hts = std::chrono::system_clock::now();
                    meas.ts = m_lastTimestamp[rxeui];
                    meas.txeui = msg.txId;
                    meas.rxeui = msg.rxId;
                    meas.seq = msg.seqNr;
                    meas.fpPower = 0;
                    meas.rxPower = 0;
                    meas.fpRatio = 0;

                    m_measurements[meas.txeui][meas.seq].insert(std::make_pair(meas.rxeui, meas));
                }
                else
                {
                    std::cout << "Warning: " << msg.txId << " not in Whitelist " << std::endl;
                }
            }
        }
    }

    usleep(100);
}

void Parser::extractSamples(std::vector<sample_t> *samples)
{
    // go through measurements and schedule latest
    auto now = std::chrono::system_clock::now();

    // go through txeuis
    for (auto it_txeui = m_measurements.begin(); it_txeui != m_measurements.end(); ++it_txeui)
    {
        // go through sequence numbers
        for (auto it_seq = it_txeui->second.begin(); it_seq != it_txeui->second.end(); ++it_seq)
        {
            bool dispatch = false;

            // go through rxeuis
            for (auto it_rxeui = it_seq->second.begin(); it_rxeui != it_seq->second.end(); ++it_rxeui)
            {
                // go through
                auto d = now - it_rxeui->second.hts;
                if(d > std::chrono::milliseconds(10))
                {
                    dispatch = true;
                }
            }

            if(dispatch)
            {
                // Linearize sequence number
                int64_t seq = it_seq->first;
                uint64_t txeui = it_txeui->first;

                if(m_lastSequence.find(txeui) == m_lastSequence.end())
                {
                    m_lastSequence.insert(std::make_pair(txeui, seq));
                }
                else
                {
                    int64_t lastseq = m_lastSequence[txeui];
                    int64_t res = lastseq % 256;

                    if(seq - res > 0)
                    {
                        m_lastSequence[txeui] = lastseq + (seq - res);
                        //std::cout << "+ seq:" << seq << " lastseq:" << lastseq << " res:" << res << std::endl;
                    }
                    if(seq - res < 0)
                    {
                        m_lastSequence[txeui] = lastseq + (seq - res) + 256;
                        //std::cout << "- seq:" << seq << " lastseq:" << lastseq << " res:" << res << std::endl;
                    }
                }

                //std::cout << "Dispatching sample " << txeui << " seq:" << m_lastSequence[txeui] << " size:" << it_seq->second.size() << std::endl;

                sample_t s;
                s.hts = now;
                s.txeui = txeui;
                s.seq = m_lastSequence[txeui];
                s.meas = it_seq->second;

                it_txeui->second.erase(it_seq);

                samples->push_back(s);
            }
        }
    }
}


ClockModel::ClockModel ()
{
    m_lastSync = 0;
    m_lastOffset = 0;
    m_lastDrift = 0;
}

ClockModel::~ClockModel()
{

}

void ClockModel::processSynchronizationFrame(uint64_t seq, uint64_t ts)
{
    double syncFrequency = (double)ticksPerSecond / ticksPerInterval;
    double ref = (double)seq * (1 / syncFrequency);
    double toa = (double)ts / ticksPerSecond;
    double offset = toa - ref;
    double drift = (offset - m_lastOffset) * syncFrequency;

    //std::cout << "Sync - reference: " << std::setprecision(2) << std::fixed << ref;
    //std::cout << std::setprecision(2) << std::fixed << "s, offset: " << offset*1000;
    //std::cout << std::setprecision(2) << std::fixed << "ms, drift: " << drift*1000000000 << "ns/s" << std::endl;

    m_lastSync = toa;
    m_lastOffset = offset;
    m_lastDrift = drift;
}

double ClockModel::getCorrectedTOA(uint64_t ts)
{
    double toa = (double)ts / ticksPerSecond;
    double td = toa - m_lastSync;
    double correction = m_lastOffset + m_lastDrift * td;
    double corrected = toa - correction;

    //std::cout << "Loc - toa: " << std::setprecision(3) << std::fixed << toa;
    //std::cout << std::setprecision(2) << std::fixed << "s, td: " << td*1000;
    //std::cout << std::setprecision(2) << std::fixed << "ms, drift*td: " << m_lastDrift*td*1000000000;
    //std::cout << std::setprecision(9) << std::fixed << "ns, corrected: " << corrected << std::endl;

    return corrected;
}


ClockCorrection::ClockCorrection ()
{

}

ClockCorrection::~ClockCorrection()
{

}

void ClockCorrection::processSample(sample_t sample)
{
    if(sample.txeui == syncEUI)
    {
        for (auto it = sample.meas.begin(); it != sample.meas.end(); ++it)
        {
            if(m_anchorClocks.find(it->first) == m_anchorClocks.end())
            {
                ClockModel clock;
                m_anchorClocks.insert(std::make_pair(it->first, clock));
            }
            m_anchorClocks[it->first].processSynchronizationFrame(sample.seq, it->second.ts);
        }
        m_lastSync = sample.hts;
        m_lastSequence = sample.seq;
        if((int64_t)sample.seq - (int64_t)m_lastSequence > 1)
        {
            std::cout << "Warning: Missed sync packet " << sample.seq << std::endl;
        }
    }
    else
    {
        for (auto it = sample.meas.begin(); it != sample.meas.end(); ++it)
        {
            if(m_anchorClocks.find(it->first) == m_anchorClocks.end())
            {
                ClockModel clock;
                m_anchorClocks.insert(std::make_pair(it->first, clock));
            }
            it->second.toa = m_anchorClocks[it->first].getCorrectedTOA(it->second.ts);
        }

        /* Check time since last sync */
        int64_t d = std::chrono::duration_cast<std::chrono::milliseconds>(sample.hts - m_lastSync).count();
        if(d < 200)
        {
            m_samples.push_back(sample);
        }
        else
        {
            std::cout << "Warning: Last sync older than 200ms!!!" << std::endl;
        }
    }
}

void ClockCorrection::processSamples(std::vector<sample_t> *samples)
{
    for (auto it = samples->begin(); it != samples->end(); ++it)
    {
        processSample(*it);
    }
}

std::vector<sample_t> ClockCorrection::getCorrectedSamples()
{
    std::vector<sample_t> s = m_samples;
    m_samples.clear();
    return s;
}