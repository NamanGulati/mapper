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
#include "ezgl/control.hpp"
#include "ezgl/camera.hpp"
#include "ezgl/canvas.hpp"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <vector>
#include <sstream>

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
    double drawHieght=0;
};

struct FeatureData{
    std::string name;
    FeatureType type;
    double area;
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
std::string removeSpaceAndConcatAndDash(std::string remove);
bool pairCompareStringInt(std::pair<std::string, int>, std::pair<std::string, int>);
void getBounds();
float x_from_lon(float lon);
float y_from_lat(float lat);
float lon_from_x(float x);
float lat_from_y(float y);
std::vector<std::string> parse2Streets(std::string s);
bool sortFeatures(FeatureData first, FeatureData second);
void zoomOnIntersection(ezgl::application *app, int idx);
void clearHighlights();
char * castToCharArray(std::string s);
std::string toLower(std::string s);
std::string createMapPath(std::string s);
void loadImages(ezgl::renderer *g);
std::string parseTransitInfo(std::stringstream& ss);
void infoPopup(ezgl::application *app, std::vector<int> interId, std::string transitInfo);
void onDialogResponse(GtkDialog *dialog, gint response_id, gpointer user_data);
#endif /* HELPERS_H */

