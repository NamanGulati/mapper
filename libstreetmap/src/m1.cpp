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
std::vector<std::vector<StreetSegmentData>> intersections;


bool load_map(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully

    std::cout << "load_map: " << map_streets_database_filename << std::endl;
    loadStreetsDatabaseBIN(map_streets_database_filename);
    int index = map_streets_database_filename.find(".streets");
    
    loadOSMDatabaseBIN(map_streets_database_filename.replace(index,8,".osm"));
    //Load your map related data structures here
    //
    /*
     * Data needed:
     * Streets      map: StreetIndex(name) -> vector(segments)
     * Intersection
     */
    
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
        int nSegs = getIntersectionStreetSegmentCount(i);
        std::vector<StreetSegmentData> segmentsAtIntersection;
        for(int j=0;j<nSegs;j++){
            struct StreetSegmentData segData;
            segData.segId=getIntersectionStreetSegment(i,j);
            segData.segInfo=getInfoStreetSegment(segData.segId);
            segmentsAtIntersection.push_back(segData);
        }
        intersections.push_back(segmentsAtIntersection);
    }
    
    std::cout<<streets.size()<<std::endl;
    
    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

void close_map() {
    //Clean-up your map related data structures here
    
}

double find_distance_between_two_points(std::pair<LatLon, LatLon> points){//nayfon
    double lon1 = points.first.lon()*DEGREE_TO_RADIAN;
    double lon2 = points.second.lon()*DEGREE_TO_RADIAN;
    double lat1 = points.first.lat()*DEGREE_TO_RADIAN;
    double lat2 = points.second.lat()*DEGREE_TO_RADIAN;
    double lat_avg = (lat1 + lat2)/2;
    double x1 = lon1*cos(lat_avg);
    double x2 = lon2*cos(lat_avg);
    return EARTH_RADIUS_METERS*sqrt(pow(lat2-lat1, 2) + pow(x2-x1, 2));
}

double find_street_segment_length(int street_segment_id){
    double length = 0;
    InfoStreetSegment seg = getInfoStreetSegment(street_segment_id);
    std::pair <LatLon, LatLon> points;
    
    if (seg.curvePointCount != 0){
        points.first = getIntersectionPosition(seg.from);
        points.second = getStreetSegmentCurvePoint(1,seg.curvePointCount);
        length += find_distance_between_two_points(points);
        for (int x = 2; x < seg.curvePointCount; x ++){
            points.first = getStreetSegmentCurvePoint(x,seg.curvePointCount);
            points.second = getStreetSegmentCurvePoint(x + 1,seg.curvePointCount);
            length += find_distance_between_two_points(points);
        }
        points.first = getStreetSegmentCurvePoint(seg.curvePointCount,seg.curvePointCount);
        points.second = getIntersectionPosition(seg.to);
        length += find_distance_between_two_points(points);
    }
    else{
        points.first = getIntersectionPosition(seg.from);
        points.second = getIntersectionPosition(seg.to);
        length += find_distance_between_two_points(points);
    }
    
    return length;
}

double find_street_segment_travel_time(int street_segment_id){//rob
    return 0;
}
int find_closest_intersection(LatLon my_position){return 0;}//nathan
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
std::vector<int> find_adjacent_intersections(int intersection_id){return std::vector<int>();}//rob
std::vector<int> find_street_segments_of_street(int street_id){return std::vector<int>();}//nathan
std::vector<int> find_intersections_of_street(int street_id){
    std::vector<StreetSegmentData> segments = streets[street_id];
    std::set<int> streetIntersections;
    for(int i=0;i<segments.size();i++){
        streetIntersections.insert(segments[i].segInfo.from);
        streetIntersections.insert(segments[i].segInfo.to);
    }
    return std::vector<int> (streetIntersections.begin(),streetIntersections.end());
    
}//naman
std::vector<int> find_intersections_of_two_streets(std::pair<int, int> street_ids){return std::vector<int>();}//rob
std::vector<int> find_street_ids_from_partial_street_name(std::string street_prefix){return std::vector<int>();}//rob
double find_feature_area(int feature_id){return 0;}//Nathan
double find_way_length(OSMID way_id){
    int nWays = getNumberOfWays();
    const OSMWay * targetWay;
    for(int i =0;i<nWays;i++){
        const OSMWay * temp = getWayByIndex(i);
        if(temp->id()==way_id){
            targetWay=temp;
            break;
        }
    }
    std::vector<OSMID> nodeIds = getWayMembers(targetWay);
    std::vector<const OSMNode*>nodes;
    int nNodes = getNumberOfNodes();
    for(int i =0;i<nNodes;i++){
        const OSMNode * temp = getNodeByIndex(i);
        for(int j=0;j<nodeIds.size();j++){
            if(temp->id()==nodeIds[j]){
                nodes.push_back(temp);
                break;
            }
        }
    }
    double wayLength=0;
    for(int i=0;i<nodes.size()-1;i++){
        wayLength+=find_distance_between_two_points(std::pair<LatLon,LatLon>(getNodeCoords(nodes[i]),getNodeCoords(nodes[i+1])));
    }
    return wayLength;
    
            
}//naman
