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
#include "ezgl/point.hpp"
#include <vector>

struct Cartesian{
    double xCoord;
    double yCoord;
};

struct intersection_data {
        LatLon position;
        std::string name;
        IntersectionIndex idx;
        bool isHighlighted=false;
};


enum StreetType{
    CITY_ROAD, //secondary,tertiary 
    EXPRESSWAY, //motorway, trunk
    SMALL_HIGHWAY, //primary
    RESIDENTIAL, //residential,
    OTHER
};

struct StreetSegmentData{
    InfoStreetSegment info;
    int idx;
    const OSMWay* way;
    StreetType type;
    int lanes=-1;
    std::vector<LatLon> curvePts;
    std::vector<ezgl::point2d> convertedCurvePoints;

};

struct FeatureData{
    std::string name;
    FeatureType type;
    bool isClosed;
    std::vector<LatLon> points;
    std::vector<ezgl::point2d> convertedPoints;
};

struct POIData{
    std::string type;
    std::string name;
    LatLon position;
    //ezgl::point2d location;
    const OSMNode * node;
};

std::pair<Cartesian, Cartesian>  convertLatLonToCartesian(std::pair<LatLon, LatLon> points);
ezgl::point2d LatLonTo2d(LatLon point);
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
std::vector<std::string> parse2Streets(std::string s);

#endif /* HELPERS_H */

