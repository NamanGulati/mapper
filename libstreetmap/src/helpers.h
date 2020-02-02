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
const OSMWay* getWayFromOSMID(OSMID way_id){
    int nWays = getNumberOfWays();
    if(nWays==0) return 0;
    const OSMWay * targetWay;
    for(int i =0;i<nWays;i++){
        const OSMWay * temp = getWayByIndex(i);
        if(temp->id()==way_id){
            targetWay=temp;
            break;
        }
    }
    return targetWay;
}


const OSMNode* getNodeFromOSMID(OSMID node_id){
    int nNodes=getNumberOfNodes();
    for(int i=0;i<nNodes;i++){
        const OSMNode* temp = getNodeByIndex(i);
        if(temp->id()==node_id)
            return temp;
    }
    return nullptr;
}

std::string removeSpaceAndConcat(std::string remove){
    std::string newString = "";
    remove.erase(std::remove_if(remove.begin(), remove.end(), isspace), remove.end());
    for (int x = 0; x < remove.length(); x ++){
        newString.push_back(tolower(remove[x]));
    }
    return newString;
}

#endif /* HELPERS_H */

