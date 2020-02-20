/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   helpers.h
 * Author: gulatin4
 *
 * Created on January 31, 2020, 12:49 PM
 */

#ifndef HELPERS_H
#define HELPERS_H

#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"

struct Cartesian{
    double xCoord;
    double yCoord;
};


std::pair<Cartesian, Cartesian>  convertLatLonToCartesian(std::pair<LatLon, LatLon> points);
bool compareOSMID(OSMID id1, OSMID id2);
const OSMWay* getWayFromOSMID(OSMID way_id);
const OSMNode* getNodeFromOSMID(OSMID node_id);
std::string removeSpaceAndConcat(std::string remove);
bool pairCompareStringInt(std::pair<std::string, int>, std::pair<std::string, int>);
void getBounds(float &minLon, float &maxLon, float &minLat, float &maxLat);
float x_from_lon(float lon);
float y_from_lat(float lat);
float lon_from_x(float x);
float lat_from_y(float y);

#endif /* HELPERS_H */

