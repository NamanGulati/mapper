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
#include <map>
#include <vector> 
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <math.h>
#include <set>
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

struct StreetSegmentData{
    struct InfoStreetSegment segInfo;
    StreetSegmentIndex segId;
};


std::map<StreetIndex,std::vector<StreetSegmentData>> streets;
std::map<int,std::vector<StreetSegmentData>> intersections;
//std::map<OSMID,std::vector<const OSMNode *>> osmNodeStore;

bool load_map(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully

    std::cout << "load_map: " << map_streets_database_filename << std::endl;
    loadStreetsDatabaseBIN(map_streets_database_filename);
    int index = map_streets_database_filename.find(".streets");
    
    loadOSMDatabaseBIN(map_streets_database_filename.replace(index,8,".osm"));
    //Load your map related data structures here
    
    int nStreetSegments = getNumStreetSegments();
    for(int i=0;i<nStreetSegments;i++){
        struct InfoStreetSegment sgmt = getInfoStreetSegment(i);
        struct StreetSegmentData segDataWrapper;
        segDataWrapper.segId=i;
        segDataWrapper.segInfo=sgmt;
        streets[sgmt.streetID].push_back(segDataWrapper);
    }
    
    int nIntersections=getNumIntersections();
    for(int i = 0;i<nIntersections;i++){
        //int nSegs = getIntersectionStreetSegmentCount(i);
        std::vector<StreetSegmentData> segmentsAtIntersection;
        for(int j=0;j<getIntersectionStreetSegmentCount(i);j++){
            struct StreetSegmentData segData;
            segData.segId=getIntersectionStreetSegment(i,j);
            segData.segInfo=getInfoStreetSegment(getIntersectionStreetSegment(i,j));
            segmentsAtIntersection.push_back(segData);
        }
        intersections[i]=(segmentsAtIntersection);
    }
    
   /* int nWays = getNumberOfWays();
    for(int i =0;i<nWays;i++){
        const OSMWay * temp = getWayByIndex(i);
        std::vector<OSMID> nodeIds = getWayMembers(temp);
        int nNodes = getNumberOfNodes();
        for(int j=0;j<nodeIds.size();j++){
            for(int k=0;k<nNodes;k++){
                const OSMNode * temp = getNodeByIndex(k);
                if(temp->id()==nodeIds[j]){
                    osmNodeStore[temp->id()].push_back(temp);
                    break;
                }
            }
        }
    }*/
    
    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

void close_map() {
    //Clean-up your map related data structures here
    closeStreetDatabase();
    closeOSMDatabase();
    streets.clear();
    intersections.clear();
    
}

double find_distance_between_two_points(std::pair<LatLon, LatLon> points){//nayfon
    std::pair<Cartesian, Cartesian> convertedPoints = convertLatLonToCartesian(points);
    return EARTH_RADIUS_METERS*sqrt(pow((convertedPoints.second.yCoord - convertedPoints.first.yCoord),2) 
            + pow((convertedPoints.second.xCoord - convertedPoints.first.xCoord),2));
}

double find_street_segment_length(int street_segment_id){
    double length = 0;
    InfoStreetSegment seg = getInfoStreetSegment(street_segment_id);
    std::pair <LatLon, LatLon> points;
    
    if (seg.curvePointCount != 0){
        points.first = getIntersectionPosition(seg.from);
        points.second = getStreetSegmentCurvePoint(0,street_segment_id);
        length += find_distance_between_two_points(points);
        for (int x = 0; x < (seg.curvePointCount-1); x ++){
            points.first = getStreetSegmentCurvePoint(x,street_segment_id);
            points.second = getStreetSegmentCurvePoint(x + 1,street_segment_id);
            length += find_distance_between_two_points(points);
        }
        points.first = getStreetSegmentCurvePoint(seg.curvePointCount - 1,street_segment_id);
        points.second = getIntersectionPosition(seg.to);
        length += find_distance_between_two_points(points);
    }
    else{
        points.first = getIntersectionPosition(seg.from);
        points.second = getIntersectionPosition(seg.to);
        length += find_distance_between_two_points(points);
    }
    
    return length;
}//rob

double find_street_segment_travel_time(int street_segment_id){
    return (find_street_segment_length(street_segment_id)/getInfoStreetSegment(street_segment_id).speedLimit)*3.6;
}//rob

int find_closest_intersection(LatLon my_position){
    double min = 10000000000;
    int curr;
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
    std::vector<StreetSegmentData> segs = intersections[intersection_id];
    std::vector<int> segIds;
    for(int i=0;i<segs.size();i++){
        segIds.push_back(segs[i].segId);
    }
    return segIds;
}//naman

std::vector<std::string> find_street_names_of_intersection(int intersection_id){
    std::vector<StreetSegmentData> segs = intersections[intersection_id];
    std::vector<std::string> streetNames;
    for(int i=0;i<segs.size();i++){
        streetNames.push_back(getStreetName(segs[i].segInfo.streetID));
    }
    return streetNames;
}//naman

bool are_directly_connected(std::pair<int, int> intersection_ids){
    std::vector<StreetSegmentData> adjoiningSegments=intersections[intersection_ids.first];
    for(int i=0;i<adjoiningSegments.size();i++){
        if(adjoiningSegments[i].segInfo.from==intersection_ids.second||adjoiningSegments[i].segInfo.to==intersection_ids.second)
            return true;
    }
    return false;    
}//naman

std::vector<int> find_adjacent_intersections(int intersection_id){
    std::vector<StreetSegmentData> segs = intersections[intersection_id];
    std::vector<IntersectionIndex> intersects;
    std::map<IntersectionIndex,bool> visited;
    for (int x = 0; x < segs.size(); x ++){
        if (!visited.at(segs[x].segInfo.from) && !visited.at(segs[x].segInfo.to)){
            if (segs[x].segInfo.oneWay){
                if (segs[x].segInfo.to != intersection_id){
                    intersects.push_back(segs[x].segInfo.to);
                    visited.insert(std::pair<int,bool>(segs[x].segInfo.to,1));
                }
            }
            else if (segs[x].segInfo.from != intersection_id){
                intersects.push_back(segs[x].segInfo.from);
                visited.insert(std::pair<int,bool>(segs[x].segInfo.from,1));
            }
            else{
                intersects.push_back(segs[x].segInfo.to);
                visited.insert(std::pair<int,bool>(segs[x].segInfo.to,1));
            }     
        }
        
    }
    
    return intersects;
}//rob

std::vector<int> find_street_segments_of_street(int street_id){
    std::vector<StreetSegmentData> segments = streets[street_id];
    std::vector<int> segIds;
    for(int i=0;i<segments.size();i++){
        segIds.push_back(segments[i].segId);
    }
    return segIds;
}//nathan

std::vector<int> find_intersections_of_street(int street_id){
    std::vector<StreetSegmentData> segments = streets[street_id];
    std::set<int> streetIntersections;
    for(int i=0;i<segments.size();i++){
        streetIntersections.insert(segments[i].segInfo.from);
        streetIntersections.insert(segments[i].segInfo.to);
    }
    return std::vector<int> (streetIntersections.begin(),streetIntersections.end());
    
}//naman

std::vector<int> find_intersections_of_two_streets(std::pair<int, int> street_ids){
    std::vector<int> street1 = find_intersections_of_street(street_ids.first);
    std::vector<int> street2 = find_intersections_of_street(street_ids.second);
    std::vector<int> commonIntersections; 
    
    std::set_intersection(street1.begin(), street1.end(), street2.begin(), street2.end(), commonIntersections.begin());
    
    return commonIntersections;
}//rob

std::vector<int> find_street_ids_from_partial_street_name(std::string street_prefix){
    std::vector<int> streetsFromPartial;
    std::string name;
    for (int x = 0; x < streets.size(); x ++ ){
        name = getStreetName(x);
        if (name.find(street_prefix)){
            streetsFromPartial.push_back(x);
        }
    }
    return streetsFromPartial;
}//rob

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
    /*int nWays = getNumberOfWays();
    if(nWays==0) return 0;
    const OSMWay * targetWay;
    for(int i =0;i<nWays;i++){
        const OSMWay * temp = getWayByIndex(i);
        if(temp->id()==way_id){
            targetWay=temp;
            break;
        }
    }*/
    const OSMWay* targetWay = getWayFromOSMID(way_id);
    std::vector<OSMID> nodeIds = getWayMembers(targetWay);
    /*std::vector<const OSMNode*>nodes;
    std::sort(nodeIds.begin(),nodeIds.end(),compareOSMID);
    int nNodes = getNumberOfNodes();
    for(int i=0;i<nodeIds.size();i++){
        for(int j=0;j<nNodes;j++){
            const OSMNode * temp = getNodeByIndex(j);
            if(temp->id()==nodeIds[i]){
                nodes.push_back(temp);
                break;
            }
        }
    }
    */
    double wayLength=0;
    for(int i=0;i<nodeIds.size()-1;i++){
        wayLength+=find_distance_between_two_points(std::pair<LatLon,LatLon>(getNodeCoords(getNodeFromOSMID(nodeIds[i])),getNodeCoords(getNodeFromOSMID(nodeIds[i+1]))));
    }
    return wayLength;
}//naman

