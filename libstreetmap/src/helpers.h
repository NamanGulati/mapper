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

enum TurnType{
    RIGHT,
    LEFT,
    STRAIGHT_SAME_STREET,
    STRAIGHT_DIFF_STREET,
    NONE
};

struct StreetSegmentData{
    InfoStreetSegment info;
    int idx;
    const OSMWay* way;
    StreetType type;
    int lanes=-1;
    std::vector<LatLon> curvePts;
    std::vector<ezgl::point2d> convertedCurvePoints;
    double drawHeight=0;
    bool isOnPath=false;
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

class segIntersectionData{
    public:
        IntersectionIndex intersection;
        StreetSegmentIndex segment;
        double distance;
    
    
    segIntersectionData(int intersect, int sgmt){
        intersection=intersect;
        segment=sgmt;
        distance=INT_MAX;
    }

    segIntersectionData(int intersect,int sgmt, int dist){
        intersection=intersect;
        segment=sgmt;
        distance=dist;
    }
    
    segIntersectionData(){
        intersection=-1;
        segment=-1;
        distance=INT_MAX;
    }

    bool operator>(segIntersectionData & rhs){
        return distance>rhs.distance;
    }
    bool operator<(segIntersectionData & rhs){
        return distance<rhs.distance;
    }
};

class segIntersectionDataComparator{
    public:
        bool operator()(segIntersectionData& lhs, segIntersectionData & rhs){
            return lhs.distance > rhs.distance;
        }
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
std::vector<std::string> parse4Streets(std::string textInput);
bool sortFeatures(FeatureData first, FeatureData second);
void zoomOnIntersection(ezgl::application *app, int idx);
void clearHighlights();
//char * castToCharArray(std::string s);
std::string toLower(std::string s);
std::string createMapPath(std::string s);
std::string parseTransitInfo(std::stringstream& ss);
void infoPopup(ezgl::application *app, std::vector<int> interId, std::string transitInfo);
void onDialogResponse(GtkDialog *dialog, gint response_id, gpointer user_data);
TurnType determineDirection(LatLon O, LatLon A, LatLon B);
LatLon getFirstCurvePoint(IntersectionIndex idx, InfoStreetSegment seg);
LatLon getLastCurvePoint(IntersectionIndex idx, InfoStreetSegment seg);
IntersectionIndex findIntersectionOfSegments(StreetSegmentIndex first, StreetSegmentIndex second);
int getTotalPathDistance(std::vector<StreetSegmentIndex> path);
int findTotalPathDistance(std::vector<StreetSegmentIndex> path);
void drawPathStreetSegment(ezgl::renderer * g, StreetSegmentData& segDat, const ezgl::color * color);
TurnType findTurnType(StreetSegmentIndex first, StreetSegmentIndex second);
void printDirections(std::vector<IntersectionIndex> walkPath);
std::string getLengthStreet(std::vector<StreetSegmentIndex> path, StreetIndex street_id, int idx);
std::vector<std::string> getDirections(std::vector<IntersectionIndex> walkPath);
std::string calcBearing(LatLon A, LatLon B);
std::string calcDirection(InfoStreetSegment seg1, InfoStreetSegment seg2);
#endif /* HELPERS_H */

