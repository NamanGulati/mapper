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

#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <cmath>

#ifndef HELPERS_H
#define HELPERS_H
struct Cartesian{
    double xCoord;
    double yCoord;
};

std::pair<Cartesian, Cartesian>  convertLatLonToCartesian(std::pair<LatLon, LatLon> points);
bool compareOSMID(OSMID id1, OSMID id2);
const OSMWay* getWayFromOSMID(OSMID way_id);
const OSMNode* getNodeFromOSMID(OSMID node_id);
std::string removeSpaceAndConcat(std::string remove);
bool pairCompare(std::pair<std::string, int>, std::pair<std::string, int>);

std::pair<Cartesian, Cartesian>  convertLatLonToCartesian(std::pair<LatLon, LatLon> points){
    
    std::pair<Cartesian, Cartesian> convertedPoints;
    double lon1 = points.first.lon()*DEGREE_TO_RADIAN;
    double lon2 = points.second.lon()*DEGREE_TO_RADIAN;
    double lat1 = points.first.lat()*DEGREE_TO_RADIAN;
    double lat2 = points.second.lat()*DEGREE_TO_RADIAN;
    double lat_avg = (lat1 + lat2)/2;
    convertedPoints.first.xCoord = lon1*cos(lat_avg);
    convertedPoints.second.xCoord = lon2*cos(lat_avg);
    convertedPoints.first.yCoord = lat1;
    convertedPoints.second.yCoord = lat2;
    
    return convertedPoints; 
}
bool compareOSMID(OSMID id1, OSMID id2){
 return id1<id2;   
}

std::string removeSpaceAndConcat(std::string remove){
    std::string newString = "";
    remove.erase(std::remove_if(remove.begin(), remove.end(), isspace), remove.end());
    for (int x = 0; x < remove.length(); x ++){
        newString.push_back(tolower(remove[x]));
    }
    return newString;
} 

bool pairCompare(std::pair<std::string, int> item1, std::pair<std::string, int> item2){
    return item1.first < item2.first;
}

#endif /* HELPERS_H */

