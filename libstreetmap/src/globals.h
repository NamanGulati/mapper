/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   globals.h
 * Author: coelhona
 *
 * Created on February 20, 2020, 4:53 PM
 */
#ifndef GLOBALS_H
#define GLOBALS_H

#include <map>
#include <vector> 
#include <set>
#include <string>
#include <unordered_set>
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "helpers.h"

extern std::unordered_map<StreetIndex,std::vector<StreetSegmentData>> streetSegData; //unordered map holding vectors of street segments for each street

extern std::unordered_map<StreetIndex,std::vector<StreetSegmentIndex>> streetSegsVectors; //unordered map holding vectors of street segments for each street

extern std::unordered_map<StreetIndex, std::set<IntersectionIndex>> streetIntersections; //unordered map holds a set of intersections corresponding to a street id

extern std::unordered_map<StreetIndex,std::vector<StreetSegmentIndex>> streetIntersectionsVectors; //unordered map that holds data in streetIntersections as a vector

extern std::vector<std::vector<StreetSegmentIndex>> intersections; //a vector that holds a vector to street segments at an intersection at intersectionIndex in the vector

extern std::unordered_map<OSMID,const OSMWay *> ways; //unordered map that holds all OSMWay* with their OSMID as keys

extern std::unordered_map<OSMID,const OSMNode*> nodes; //unordered map that holds all OSMNode* with their OSMID as keys

extern std::vector<std::pair<std::string, int>> streetNames; 

extern std::vector<float> speedLim; // a vector that holds speed limit of a street segment at index=StreetSegmentIndex

extern std::vector<double> segLen; // a vector that holds length of a street segment at index=StreetSegmentIndex

extern float lat_avg; //average latitude of current map

extern std::vector<intersection_data> intersectionsData; //unordered map that holds data in intersection_data as a vector

extern std::vector<FeatureData> featureData; //vector of all natural features on the map

extern std::vector<POIData> pois;

extern std::vector<IntersectionIndex> highlighted;

extern float max_lat, min_lat, max_lon, min_lon, max_x, min_x, max_y, min_y, diff_y, diff_x;

const std::string mapPathPre = "/cad2/ece297s/public/maps/";

const std::string mapPathSuf = ".streets.bin";

extern int zoomLevel;

const std::string locos[19] = {"beijing,china", "cairo,egypt", "cape-town,southafrica","golden-horseshoe,canada", "hamilton,canada", "hong-kong,china", "iceland", 
                            "interlaken,switzerland", "london,england", "moscow,russia", "new-delhi,india", "new-york,usa", "rio-de-janeiro,brazil", "saint-helena", 
                            "singapore", "sydney,australia", "tehran,iran", "tokyo,japan", "toronto,canada"};

const std::unordered_set<std::string> poiTypes = {"atm","bank","bar","bicycle_parking","bus_station","construction","food_court","gym","library","parking",
                                                   "restaurant", "college", "school", "university", "blank"};
extern std::unordered_map<std::string, ezgl::surface*> iconImgs;
#endif /* GLOBALS_H */

