//
//  config.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "config.h"

#include <stdio.h>
#include <iostream>

#include <armadillo>
#include <yaml-cpp/yaml.h>

Config::Config (const std::string file)
{
    YAML::Node config = YAML::LoadFile(file);

    // Anchor Configuration
    for(int i = 0; i < config["anchors"].size(); ++i)
    {
        uint64_t eui = config["anchors"][i]["euid"].as<uint64_t>();
        std::string port = config["anchors"][i]["port"].as<std::string>();
        m_anchors.ports.insert(std::pair<uint64_t, std::string>(eui, port));
        arma::vec pos;
        pos << config["anchors"][i]["position"][0].as<double>()
            << config["anchors"][i]["position"][1].as<double>()
            << config["anchors"][i]["position"][2].as<double>()
            << arma::endr;
        m_anchors.positions.insert(std::pair<uint64_t, arma::vec>(eui, pos));
    }

    // Sync Configuration
    if (config["sync"]["port"] && config["sync"]["baudrate"])
    {
        m_sync.port = config["sync"]["port"].as<std::string>();
        m_sync.baudrate = config["sync"]["baudrate"].as<int>();
        m_sync.eui = config["sync"]["euid"].as<uint64_t>();
        m_sync.interval = config["sync"]["interval"].as<uint64_t>();

        arma::vec pos;
        pos << config["sync"]["position"][0].as<double>()
            << config["sync"]["position"][1].as<double>()
            << config["sync"]["position"][2].as<double>()
            << arma::endr;
        m_sync.position = pos;

        m_parser.tagWhitelist.insert(std::pair<uint64_t, uint64_t>(m_sync.eui, m_sync.eui));
    }
    else
    {
        std::cout << "Error - Config Port for UWB Sync Node not defined!" << std::endl;
    }

    // Calibration Configuration
    if (config["calibration"]["euid"] && config["calibration"]["position"])
    {
        m_cal.eui = config["calibration"]["euid"].as<uint64_t>();
        m_cal.window = config["calibration"]["window"].as<uint64_t>();

        arma::vec pos;
        pos << config["calibration"]["position"][0].as<double>()
            << config["calibration"]["position"][1].as<double>()
            << config["calibration"]["position"][2].as<double>()
            << arma::endr;
        m_cal.position = pos;

        m_parser.tagWhitelist.insert(std::pair<uint64_t, uint64_t>(m_cal.eui, m_cal.eui));
    }
    else
    {
        std::cout << "Error - Config Port for UWB Calibration Node not defined!" << std::endl;
    }

    // Output Configuration
    if (config["output"]["port"] && config["output"]["baudrate"])
    {
        m_output.port = config["output"]["port"].as<std::string>();
        m_output.baudrate = config["output"]["baudrate"].as<int>();
    }
    else
    {
        std::cout << "Error - Config Port for Output not defined!" << std::endl;
    }

    // Parser Configuration
    for(int i = 0; i < config["whitelist"].size(); ++i)
    {
        uint64_t eui = config["whitelist"][i].as<uint64_t>();
        m_parser.tagWhitelist.insert(std::pair<uint64_t, uint64_t>(eui, eui));
    }

    // LRF Configuration
    if(config["lrf"]["wgs84ref"])
    {
        m_lrf.wgs84ref << config["lrf"]["wgs84ref"][0].as<double>()
                       << config["lrf"]["wgs84ref"][1].as<double>()
                       << config["lrf"]["wgs84ref"][2].as<double>()
                       << arma::endr;
    }
    else
    {
        std::cout << "Error - Config LRF Reference not defined!" << std::endl;
    }

    if(config["lrf"]["wgs84axis"])
    {
        m_lrf.wgs84axis << config["lrf"]["wgs84axis"][0].as<double>()
                        << config["lrf"]["wgs84axis"][1].as<double>()
                        << config["lrf"]["wgs84axis"][2].as<double>()
                        << arma::endr;
    }
    else
    {
        std::cout << "Error - Config LRF Axis not defined!" << std::endl;
    }

    // EKF Configuration
    if(config["ekf"]["processNoise"]) m_ekf.processNoise = config["ekf"]["processNoise"].as<double>();
    if(config["ekf"]["measurementVariance"]) m_ekf.measurementVariance = config["ekf"]["measurementVariance"].as<double>();
    if(config["ekf"]["initialInterval"]) m_ekf.initialInterval = config["ekf"]["initialInterval"].as<double>();
    if(config["ekf"]["initialVariance"]) m_ekf.initialVariance = config["ekf"]["initialVariance"].as<double>();
    if(config["ekf"]["initialVarianceDelta"]) m_ekf.initialVarianceDelta = config["ekf"]["initialVarianceDelta"].as<double>();

    if(config["ekf"]["initialState"])
    {
        m_ekf.initialState << config["ekf"]["initialState"][0].as<double>()
                           << config["ekf"]["initialState"][1].as<double>()
                           << config["ekf"]["initialState"][2].as<double>()
                           << config["ekf"]["initialState"][3].as<double>()
                           << config["ekf"]["initialState"][4].as<double>()
                           << config["ekf"]["initialState"][5].as<double>()
                           << arma::endr;
    }
}


void Config::list()
{
    std::cout << "Config file listing:" << std::endl;

    for (auto it = m_anchors.ports.begin(); it != m_anchors.ports.end(); ++it)
    {
        std::cout << " " << it->first << ": " << it->second << std::endl;
    }

    for (auto it = m_anchors.positions.begin(); it != m_anchors.positions.end(); ++it)
    {
        std::cout << " " << it->first << ": " << it->second(0) << ", " << it->second(1) << ", " << it->second(2) << std::endl;
    }
}


Config::~Config()
{

}