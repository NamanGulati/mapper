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
#include "m1.h"
#include <cmath>
#include <math.h>

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
bool pairCompareStringInt(std::pair<std::string, int>, std::pair<std::string, int>);
void getBounds(float &minLon, float &maxLon, float &minLat, float &maxLat);
float x_from_lon(float lon);
float y_from_lat(float lat);
float lon_from_x(float x);
float lat_from_y(float y);
float lat_avg;

//converts a pair of LatLon points to a pair of Cartesian points
std::pair<Cartesian, Cartesian>  convertLatLonToCartesian(std::pair<LatLon, LatLon> points){
    
    std::pair<Cartesian, Cartesian> convertedPoints;
    double lon1 = points.first.lon()*DEGREE_TO_RADIAN;
    double lon2 = points.second.lon()*DEGREE_TO_RADIAN;
    double lat1 = points.first.lat()*DEGREE_TO_RADIAN;
    double lat2 = points.second.lat()*DEGREE_TO_RADIAN;
//    double lat_avg = (lat1 + lat2)/2;
    convertedPoints.first.xCoord = lon1*cos(lat_avg);
    convertedPoints.second.xCoord = lon2*cos(lat_avg);
    convertedPoints.first.yCoord = lat1;
    convertedPoints.second.yCoord = lat2;
    
    return convertedPoints; 
}

float x_from_lon(float lon){
    return lon*DEGREE_TO_RADIAN*cos(lat_avg);
}

float y_from_lat(float lat){
    return lat*DEGREE_TO_RADIAN;
}

float lon_from_x(float x){
    return x/DEGREE_TO_RADIAN/cos(lat_avg);
}

float lat_from_y(float y){
    return y/DEGREE_TO_RADIAN;
}

void getBounds(float &minLon, float &maxLon, float &minLat, float &maxLat){
    float currLon, currLat;
    minLon = getIntersectionPosition(0).lon();
    maxLon = minLon;
    minLat = getIntersectionPosition(0).lat();
    maxLat = minLat;    
    
    for(int i = 1; i < getNumIntersections(); i++){
        currLon = getIntersectionPosition(i).lon();
        currLat = getIntersectionPosition(i).lat();
        
        if(currLon < minLon)
            minLon = currLon;
        
        else if(currLon > maxLon)
            maxLon = currLon;
        
        if(currLat < minLat)
            minLat = currLat;
        
        else if(currLat > maxLat)
            maxLat = currLat;
    }

    
}


bool compareOSMID(OSMID id1, OSMID id2){
 return id1<id2;   
}

//Used to format strings for comparison in the finding street index from partial name function
std::string removeSpaceAndConcat(std::string remove){
    std::string newString = "";
    remove.erase(std::remove_if(remove.begin(), remove.end(), isspace), remove.end());
    for (int x = 0; x < remove.length(); x ++){
        newString.push_back(tolower(remove[x]));
    }
    return newString;
} 

//Used for the sort function to sort a vector of pairs
bool pairCompareStringInt(std::pair<std::string, int> item1, std::pair<std::string, int> item2){
    return item1.first < item2.first;
}

#endif /* HELPERS_H */

