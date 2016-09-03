//
//  main.cpp
//  nodeconf
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

#include "docopt.h"
#include "protocol.h"


static const char USAGE[] =
    R"(nodeconf.

    Usage:
        nodeconf <port> 
        nodeconf <port> config <channel> <eui> (--anchor | --tag) 
        nodeconf <port> period <tag> <sync>
        nodeconf (-h | --help)
        nodeconf --version

        Options:
        -h --help           Show this usage screen.
        --version           Show version.
)";

bool run = true;

void signal_handler(int signal) 
{
    run = false;
    std::cerr << " Quitting..." << std::endl;
}

int main(int argc, const char* argv[])
{
    std::signal(SIGINT, signal_handler);
    std::map<std::string, docopt::value> args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true, "nodeconf 0.1");

    for(auto const& arg : args) 
    {
        std::cout << arg.first << ": " << arg.second << std::endl;
    }

    protocol::Protocol prot(args.find("<port>")->second.asString());

    protocol::msgConfig_t config;
    if(prot.sendMsgPoll(&config))
    {
        std::cout << "Read Config successful... ";
        std::cout << std::hex << "Role:" << (int)config.role << ", EUI:" << (uint64_t)config.eui << ", Channel:" << (int)config.channel << std::endl;
    }

    protocol::msgPeriod_t period;
    if(prot.sendMsgPoll(&period))
    {
        std::cout << "Read Period successful... ";
        std::cout << std::hex << "Sync:" << (uint32_t)period.syncPeriod << ", Tag:" << (uint32_t)period.tagPeriod << std::endl;
    }

    if(args.find("config")->second.asBool())
    {
        protocol::msgConfig_t cfg;

        if(args.find("--anchor")->second.asBool())
        {
            cfg.role = 2;
        }
        if(args.find("--tag")->second.asBool())
        {
            cfg.role = 1;
        }

        std::string::size_type sz = 0;
        cfg.channel = (uint16_t)std::stoul(args.find("<channel>")->second.asString(), &sz, 16);
        cfg.eui = std::stoull(args.find("<eui>")->second.asString(), &sz, 16);

        if(prot.sendMsg(cfg))
        {
            std::cout << "Write Config successful... ";
            std::cout << std::hex << "Role:" << (int)cfg.role << ", EUI:" << (uint64_t)cfg.eui << ", Channel:" << (int)cfg.channel << std::endl;
        }
    }

    if(args.find("period")->second.asBool())
    {
    protocol::msgPeriod_t per;

    std::string::size_type sz = 0;
    per.tagPeriod = (uint32_t)std::stoul(args.find("<tag>")->second.asString(), &sz, 16);
    per.syncPeriod = (uint32_t)std::stoul(args.find("<sync>")->second.asString(), &sz, 16);

    if(prot.sendMsg(per))
    {
    std::cout << "Write Period successful... ";
    std::cout << std::hex << "Tag:" << (uint32_t)per.tagPeriod << ", Sync:" << (uint32_t)per.syncPeriod << std::endl;
    }
    }

    return 0;
}
