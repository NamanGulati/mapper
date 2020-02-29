/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "helpers.h"
#include "globals.h"
#include "ezgl/point.hpp"
#include "ezgl/control.hpp"
#include "ezgl/camera.hpp"
#include "ezgl/canvas.hpp"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m1.h"
#include <cmath>
#include <string>
#include <boost/algorithm/string.hpp>
#include <bits/stdc++.h>

std::unordered_map< std::string, ezgl::surface*> iconImgs;
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

ezgl::point2d LatLonTo2d(LatLon point){
    

    double lon= point.lon()*DEGREE_TO_RADIAN;
    double lat = point.lat()*DEGREE_TO_RADIAN;
    double x = lon*cos(lat_avg);
    double y = lat;
    
    return ezgl::point2d(x,y); 
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

void getBounds(){
    
    float currLon, currLat;
    min_lon = getIntersectionPosition(0).lon();
    max_lon = min_lon;
    min_lat = getIntersectionPosition(0).lat();
    max_lat = min_lat;    
    
    for(int i = 1; i < getNumIntersections(); i++){
        currLon = getIntersectionPosition(i).lon();
        currLat = getIntersectionPosition(i).lat();
        
        if(currLon < min_lon)
            min_lon = currLon;
        
        else if(currLon > max_lon)
            max_lon = currLon;
        
        if(currLat < min_lat)
            min_lat = currLat;
        
        else if(currLat > max_lat)
            max_lat = currLat;
    }

    lat_avg = DEGREE_TO_RADIAN*(min_lat + max_lat)/2;
    
    max_x = x_from_lon(max_lon);
    max_y = y_from_lat(max_lat);
    min_x = x_from_lon(min_lon);
    min_y = y_from_lat(min_lat);
}

//void getDiff(float &diffX, float &diffY){
//    
//    if(min_x*max_x > 0 && min_y*max_y > 0){
//        diffX = abs(abs(max_x) - abs(min_x));
//        diffY = abs(abs(max_y) - abs(min_y));
//        std::cout << "smd" << std::endl;
//        return;
//    }
//    if(min_x < 0 && max_x > 0)
//        diffX = max_x - min_x;
//    if(min_y < 0 && max_y > 0)
//        diffY = max_y - min_y;
//}


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

std::string removeSpaceAndConcatAndDash(std::string remove){
    std::string newString = removeSpaceAndConcat(remove);
    while(newString.find("-") != std::string::npos){
        newString.erase(newString.find("-"), 1);
    }
    
    return newString;
}

//Used for the sort function to sort a vector of pairs
bool pairCompareStringInt(std::pair<std::string, int> item1, std::pair<std::string, int> item2){
    return item1.first < item2.first;
}

std::vector<std::string> parse2Streets(std::string textInput){
    std::vector<std::string> the2Streets;
    std::string s1, s2, toDrop;
    if (textInput.find(" and ") != std::string::npos){
        the2Streets.push_back(textInput.substr(0, textInput.find(" and ")));
        the2Streets.push_back(textInput.substr(textInput.find(" and ") + 5));
        //boost::split(the2Streets, s, boost::is_any_of("/and"));
        
    }
    else if(textInput.find("&") != std::string::npos){
        boost::split(the2Streets, textInput, boost::is_any_of("&"));
    }
    else{
        the2Streets.push_back(textInput);
    }
    
    return the2Streets;
}

bool sortFeatures(FeatureData first, FeatureData second){
    return (first.area > second.area);
}

void zoomOnIntersection(ezgl::application *app, int idx){
    float x_2d = x_from_lon(getIntersectionPosition(idx).lon());
    float y_2d = y_from_lat(getIntersectionPosition(idx).lat());
    ezgl::rectangle region({x_2d-diff_x/500, y_2d-diff_y/500}, {x_2d+diff_x/500, y_2d+diff_y/500});
    zoomLevel = 9;
    std::string main_canvas_id = app->get_main_canvas_id();
    auto canvas = app->get_canvas(main_canvas_id);
    ezgl::zoom_fit(canvas, region);
}

void clearHighlights(){
    if(highlighted.size() == 0)
        return;
    for(int i = 0; i < highlighted.size(); i++){
        intersectionsData[highlighted[i]].isHighlighted = false;
    }
    highlighted.clear();
}
//bool sortStreets()
char * castToCharArray(std::string s){
    int a = s.length();
    char charArr[a + 1]; 
    strcpy(charArr, s.c_str());
}

std::string toLower(std::string s){
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

std::string createMapPath(std::string s){
    std::string checker = removeSpaceAndConcatAndDash(s);
    std::cout << checker << std::endl;
    std::string locat, replace;
    for (int x = 0; x < 19; x ++){
        if (removeSpaceAndConcatAndDash(locos[x]).find(checker)!= std::string::npos){
            if(locos[x].find(",") != std::string::npos){
                replace = locos[x];
                replace.replace(locos[x].find(","), 1, "_");
                locat = replace;
            }
            else{
                locat = locos[x];
            }
            return (mapPathPre + locat + mapPathSuf);
        }
    }
    
    return "DNE";
    
}

void loadImages(ezgl::renderer *g){
    for(auto type: poiTypes)
        iconImgs.emplace(type, g->load_png(("libstreetmap/resources/Icons/"+type+"_icon.png").c_str()));
}