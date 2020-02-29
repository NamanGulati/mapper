/* 
 * Copyright 2020 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include "globals.h"
#include <map>
#include <vector> 
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <math.h>
#include <set>
#include <unordered_set>
#include "OSMDatabaseAPI.h"
#include "helpers.h"

// load_map will be called with the name of the file that stores the "layer-2"
// map data (the street and intersection data that is higher-level than the
// raw OSM data). This file name will always end in ".streets.bin" and you 
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1 
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the 
// name of the ".osm.bin" file that matches your map -- just change 
// ".streets" to ".osm" in the map_streets_database_filename to get the proper
// name.

std::unordered_map<StreetIndex,std::vector<StreetSegmentIndex>> streetSegsVectors; //unordered map holding vectors of street segments for each street
std::unordered_map<StreetIndex,std::vector<StreetSegmentData>> streetSegData; //unordered map holding vectors of data on street segments for each street
std::unordered_map<StreetIndex, std::set<IntersectionIndex>> streetIntersections; //unordered map holds a set of intersections corresponding to a street id
std::unordered_map<StreetIndex,std::vector<IntersectionIndex>> streetIntersectionsVectors; //unordered map that holds data in streetIntersections as a vector
std::vector<intersection_data> intersectionsData; //unordered map that holds data in intersection_data as a vector
std::vector<std::vector<StreetSegmentIndex>> intersections; //a vector that holds a vector to street segments at an intersection at intersectionIndex in the vector
std::unordered_map<OSMID,const OSMWay *> ways; //unordered map that holds all OSMWay* with their OSMID as keys
std::unordered_map<OSMID,const OSMNode*> nodes; //unordered map that holds all OSMNode* with their OSMID as keys
std::vector<std::pair<std::string, int>> streetNames; 
std::vector<float> speedLim; // a vector that holds speed limit of a street segment at index=StreetSegmentIndex
std::vector<double> segLen; // a vector that holds length of a street segment at index=StreetSegmentIndex
std::vector<FeatureData> featureData; //vector of all natural features on the map
std::vector<POIData> pois; //vector of all points of interest on the map
float lat_avg; //average latitude of current map
float max_lat, min_lat, max_lon, min_lon, max_x, min_x, max_y, min_y;
constexpr double LARGE_DIST= 10000000000;
constexpr double KM_per_H_to_M_per_S = 3.6;
/**
 * Loading above data structures by pulling data from StreetsDatabaseAPI and OSMDatabaseAPI
 * 
 * @return bool if all data was loaded successfully
 **/ 
bool load_map(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully
    load_successful = loadStreetsDatabaseBIN(map_streets_database_filename);
    
    if(!load_successful)
        return false;

    int index = map_streets_database_filename.find(".streets");
    load_successful = loadOSMDatabaseBIN(map_streets_database_filename.replace(index,8,".osm"));
    
    if(!load_successful)
        return false;
    
    std::cout << "load_map: " << map_streets_database_filename << std::endl;
    
    getBounds(min_lon, max_lon, min_lat, max_lat);    
    lat_avg = DEGREE_TO_RADIAN*(min_lat + max_lat)/2;
    
    max_x = x_from_lon(max_lon);
    max_y = y_from_lat(max_lat);
    min_x = x_from_lon(min_lon);
    min_y = y_from_lat(min_lat);
    
    
    /**
     * Loading of ways and nodes
     **/
    int nWays = getNumberOfWays();
    for(int i=0;i<nWays;i++){
        const OSMWay * temp = getWayByIndex(i);
        ways[temp->id()]=temp;
    }
    
    int nNodes=getNumberOfNodes();
    for(int i=0;i<nNodes;i++){
        const OSMNode* temp = getNodeByIndex(i);
        nodes[temp->id()] = temp;
    }


    /**
     * Loading of streetSegsVectors,streetIntersections,
     *              speedLim,segLen,streetIntersectionsVectors
     **/
    int nStreetSegments = getNumStreetSegments();
    for(int i=0;i<nStreetSegments;i++){
        struct InfoStreetSegment sgmt = getInfoStreetSegment(i);
        streetSegsVectors[sgmt.streetID].push_back(i);
        streetIntersections[sgmt.streetID].insert(sgmt.from);
        streetIntersections[sgmt.streetID].insert(sgmt.to);
        speedLim.push_back(sgmt.speedLimit);
        StreetSegmentData dat;
        dat.info=sgmt;
        dat.idx=i;
        dat.way=ways[sgmt.wayOSMID];
        
        int tags = getTagCount(dat.way);
        for(int j=0;j<tags;j++){
            std::pair<std::string,std::string> wayTag = getTagPair(dat.way,j);

            if(wayTag.first=="highway"){
                if(wayTag.second=="secondary"||wayTag.second=="tertiary"){
                    dat.type=StreetType::CITY_ROAD;
                }  
                else if(wayTag.second=="motorway"||wayTag.second=="trunk"){
                    dat.type=StreetType::EXPRESSWAY;
                }       
                else if(wayTag.second=="primary"){
                    dat.type=StreetType::SMALL_HIGHWAY;
                }         
                else if(wayTag.second=="residential"){
                    dat.type=StreetType::RESIDENTIAL;
                }
                else{
                    dat.type=StreetType::OTHER;
                }
            }
            else if(wayTag.first=="lanes"){
                dat.lanes=std::stoi(wayTag.second);
                
            }
    
        }
        if(dat.lanes==-1&&streetSegData[sgmt.streetID].size()!=0){
            dat.lanes=(streetSegData[sgmt.streetID].back()).lanes;
        }
            

        dat.curvePts.push_back(getIntersectionPosition(sgmt.from));
        dat.convertedCurvePoints.push_back(LatLonTo2d(getIntersectionPosition(sgmt.from)));
        for(int j=0;j<sgmt.curvePointCount;j++){
            dat.curvePts.push_back(getStreetSegmentCurvePoint(j,i));
            dat.convertedCurvePoints.push_back(LatLonTo2d(getStreetSegmentCurvePoint(j,i)));
        }
        dat.convertedCurvePoints.push_back(LatLonTo2d(getIntersectionPosition(sgmt.to)));
        dat.curvePts.push_back(getIntersectionPosition(sgmt.to));

        streetSegData[sgmt.streetID].push_back(dat);
        
        double length = 0;
        std::pair <LatLon, LatLon> pointsL;
        
        if (sgmt.curvePointCount != 0){
            pointsL.first = getIntersectionPosition(sgmt.from);
            pointsL.second = getStreetSegmentCurvePoint(0,i);
            length += find_distance_between_two_points(pointsL);
            for (int x = 0; x < (sgmt.curvePointCount-1); x ++){
                pointsL.first = getStreetSegmentCurvePoint(x,i);
                pointsL.second = getStreetSegmentCurvePoint(x + 1,i);
                length += find_distance_between_two_points(pointsL);
            }
            pointsL.first = getStreetSegmentCurvePoint(sgmt.curvePointCount - 1,i);
            pointsL.second = getIntersectionPosition(sgmt.to);
            length += find_distance_between_two_points(pointsL);
        }
        else{
            pointsL.first = getIntersectionPosition(sgmt.from);
            pointsL.second = getIntersectionPosition(sgmt.to);
            length += find_distance_between_two_points(pointsL);
        }
        segLen.push_back(length);

    }
    
    for(int i = 0; i < streetIntersections.size(); i++)
        (streetIntersectionsVectors[i]).assign(streetIntersections[i].begin(), streetIntersections[i].end());
    
    /**
     * Loading of intersections
     **/
    int nIntersections=getNumIntersections();
    intersectionsData.resize(nIntersections);
    for(int i = 0;i<nIntersections;i++){
        std::vector<StreetSegmentIndex> segmentsAtIntersection;
        intersection_data dt;
        dt.position=getIntersectionPosition(i);
        dt.name = getIntersectionName(i);
        dt.idx = i;
        //intersectionsData[i]=dt;
        for(int j=0;j<getIntersectionStreetSegmentCount(i);j++){
            
            segmentsAtIntersection.push_back(getIntersectionStreetSegment(i,j));
        }
        intersections.push_back(segmentsAtIntersection);
    }
    
    /**
     * Loading of streetNames
     **/
    std::pair<std::string, int> nameIndexes;
    for (int x = 0; x < getNumStreets(); x ++ ){
        nameIndexes.first = removeSpaceAndConcat(getStreetName(x));
        nameIndexes.second = x;
        streetNames.push_back(nameIndexes);
    }
    std::sort(streetNames.begin(), streetNames.end(), pairCompareStringInt);

    
    for(int i=0;i<getNumFeatures();i++){
        FeatureData fd;
        fd.name=getFeatureName(i);
        fd.type=getFeatureType(i);
        fd.area=find_feature_area(i);
//        std::vector<LatLon> points;
        for(int j=0;j<getFeaturePointCount(i);j++){
            fd.points.push_back(getFeaturePoint(j,i));
            fd.convertedPoints.push_back(LatLonTo2d(getFeaturePoint(j,i)));
        }
        if((getFeaturePoint(0, i).lat() != getFeaturePoint(getFeaturePointCount(i)-1, i).lat()) 
            || (getFeaturePoint(0, i).lon() != getFeaturePoint(getFeaturePointCount(i)-1, i).lon())){
            fd.isClosed = false;
        }
        else
            fd.isClosed = true;
        
        featureData.push_back(fd);
    }

    std::sort(featureData.begin(), featureData.end(), sortFeatures);
    
    for(int i=0;i<getNumPointsOfInterest();i++){
        POIData poi;
        poi.type=getPointOfInterestType(i);
        poi.name = getPointOfInterestName(i);
        poi.position=getPointOfInterestPosition(i);
        //poi.location=LatLonTo2d(poi.position);
        poi.node=nodes[getPointOfInterestOSMNodeID(i)];
        pois.push_back(poi);
    }
    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

/**
 * clear all data loaded in load_map
 **/
void close_map() {
    closeStreetDatabase();
    closeOSMDatabase();
    streetIntersections.clear();
    streetIntersectionsVectors.clear();
    streetSegsVectors.clear();
    intersections.clear();
    ways.clear();
    nodes.clear();
    streetNames.clear();
    segLen.clear();
    speedLim.clear();
}

//finds the distance between two given LatLon points by converting them to Cartesian and using pythagoras' theorem
double find_distance_between_two_points(std::pair<LatLon, LatLon> points){
    std::pair<Cartesian, Cartesian> convertedPoints = convertLatLonToCartesian(points);
    return EARTH_RADIUS_METERS*sqrt(pow((convertedPoints.second.yCoord - convertedPoints.first.yCoord),2) 
            + pow((convertedPoints.second.xCoord - convertedPoints.first.xCoord),2));
}//nathan

double find_street_segment_length(int street_segment_id){
    return segLen[street_segment_id];
}//rob

double find_street_segment_travel_time(int street_segment_id){
    return (find_street_segment_length(street_segment_id)/speedLim[street_segment_id])*KM_per_H_to_M_per_S;
}//rob

//determines the closest intersection to a given point on the map by finding the minimum distance to an intersection 
int find_closest_intersection(LatLon my_position){
    double min = LARGE_DIST; //TODO: Replace with some constant thats a magic number
    int curr = 0;
    for(int i = 0; i < getNumIntersections(); i++){
        double dist = find_distance_between_two_points(std::pair<LatLon, LatLon>(my_position, getIntersectionPosition(i)));
        if(dist < min){
            min = dist;
            curr = i;
        }
    }
    return curr;
}//nathan

std::vector<int> find_street_segments_of_intersection(int intersection_id){
    return intersections[intersection_id];
}//naman

std::vector<std::string> find_street_names_of_intersection(int intersection_id){
    std::vector<StreetSegmentIndex> segs = intersections[intersection_id];
    std::vector<std::string> streetNamesL;
    for(int i=0;i<segs.size();i++){
        streetNamesL.push_back(getStreetName(getInfoStreetSegment(segs[i]).streetID));
    }
    return streetNamesL;
}//naman

bool are_directly_connected(std::pair<int, int> intersection_ids){
    std::vector<StreetSegmentIndex> adjoiningSegments=intersections[intersection_ids.first];
    
    //checking if each of the segments at the first intersection connect
    //with the second intersection
    for(int i=0;i<adjoiningSegments.size();i++){
        struct InfoStreetSegment segment=getInfoStreetSegment(adjoiningSegments[i]);
        if((segment.from==intersection_ids.second&&!segment.oneWay)||segment.to==intersection_ids.second)
            return true;
    }
    return false;    
}//naman

std::vector<int> find_adjacent_intersections(int intersection_id){
    std::vector<StreetSegmentIndex> segs = intersections[intersection_id];
    std::vector<IntersectionIndex> intersects;
    std::unordered_map<IntersectionIndex,bool> visited;
    for (int x = 0; x < segs.size(); x ++){
        struct InfoStreetSegment segment = getInfoStreetSegment(segs[x]);
        if (!visited.count(segment.from) && !visited.count(segment.to)){
            if (segment.oneWay){
                if (segment.to != intersection_id){
                    intersects.push_back(segment.to);
                    visited.insert(std::pair<int,bool>(segment.to,1));
                }
            }
            else if (segment.from != intersection_id){
                intersects.push_back(segment.from);
                visited.insert(std::pair<int,bool>(segment.from,1));
            }
            else{
                intersects.push_back(segment.to);
                visited.insert(std::pair<int,bool>(segment.to,1));
            }     
        }
        
    }
    std::sort(intersects.begin(), intersects.end());
    return intersects;
}//rob

//returns the segments of a street from data initialized in load map
std::vector<int> find_street_segments_of_street(int street_id){
    return streetSegsVectors[street_id];
}//nathan

std::vector<int> find_intersections_of_street(int street_id){
    return streetIntersectionsVectors[street_id];
}//naman

//finds the common intersections of two streets by calling on the above function and finding the intersection of the vectors
std::vector<int> find_intersections_of_two_streets(std::pair<int, int> street_ids){
    std::vector<int> intersections1 (find_intersections_of_street(street_ids.first));
    std::vector<int> intersections2 (find_intersections_of_street(street_ids.second));
    std::vector<int> commonIntersections; 

    std::set_intersection(intersections1.begin(), intersections1.end(), 
                            intersections2.begin(), intersections2.end(),
                            std::back_inserter(commonIntersections));
    
    return commonIntersections;
}//nathan

std::vector<int> find_street_ids_from_partial_street_name(std::string street_prefix){
    if(street_prefix.length()==0)
        return std::vector<int>();
    std::vector<int> streetsFromPartial; 
    std::string name, searchParam;
    searchParam = removeSpaceAndConcat(street_prefix);
    int l = 0;
    int r = streetNames.size() - 1;
    
    //Binary Searches through the vector of street names to find the first occurrence of a matching street
    while (l != r - 1) { 
        int mid = (l+r)/ 2; 
  
        if (streetNames[mid].first < searchParam)
            l = mid;
        else 
            r = mid;
    } 
    
    //Corner case where either the first element or second last element of the vector are the first matching street
    int iterate = r;
    name = streetNames[l].first;
    std::size_t foundLoc = name.find(searchParam);
    if(foundLoc != std::string::npos && foundLoc == 0){
        streetsFromPartial.push_back(streetNames[l].second);
    }
    
    //Loop from the first occurrence until the street doesn't match
    while(iterate < streetNames.size()){
        name = streetNames[iterate].first;
        foundLoc = name.find(searchParam);
        if (foundLoc != std::string::npos && foundLoc == 0){
            streetsFromPartial.push_back(streetNames[iterate].second);
        }
        else
            break;
        iterate ++;
    }
    return streetsFromPartial;
}//rob

//determines the area of a feature by converting to Cartesian and using the shoelace algorithm, returns 0 if the feature is not a closed polygon
double find_feature_area(int feature_id){
    if((getFeaturePoint(0, feature_id).lat() != getFeaturePoint(getFeaturePointCount(feature_id)-1, feature_id).lat()) 
            || (getFeaturePoint(0, feature_id).lon() != getFeaturePoint(getFeaturePointCount(feature_id)-1, feature_id).lon())){
        return 0;
    }

    double area = 0.0;
    int j = getFeaturePointCount(feature_id) - 1;
    
    for(int i = 0; i < getFeaturePointCount(feature_id); i++){
        std::pair<LatLon, LatLon> points;
        points.first = getFeaturePoint(i, feature_id);
        points.second = getFeaturePoint(j, feature_id);
        std::pair<Cartesian, Cartesian> convertPoints = convertLatLonToCartesian(points);
        area += (convertPoints.second.xCoord + convertPoints.first.xCoord)*(convertPoints.second.yCoord - convertPoints.first.yCoord);
        j = i;
    }
    return abs(area/2.0)*EARTH_RADIUS_METERS*EARTH_RADIUS_METERS;
}//Nathan



double find_way_length(OSMID way_id){

    const OSMWay* targetWay = ways[way_id];
    std::vector<OSMID> nodeIds = getWayMembers(targetWay);
    double wayLength=0;
    for(int i=0;i<nodeIds.size()-1;i++){
        wayLength+=find_distance_between_two_points(std::pair<LatLon,LatLon>(getNodeCoords(nodes[nodeIds[i]]),getNodeCoords(nodes[nodeIds[i+1]])));
    }
    return wayLength;
}//naman

