/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "helpers.h"
#include "globals.h"
#include "ezgl/point.hpp"
#include "m1.h"
#include <cmath>
#include <boost/algorithm/string.hpp>

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
    
    return the2Streets;
}

bool sortFeatures(FeatureData first, FeatureData second){
    return (first.area > second.area);
}