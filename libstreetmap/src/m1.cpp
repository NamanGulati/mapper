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
std::map<StreetIndex,std::vector<InfoStreetSegment>> streets;
std::vector<std::vector<InfoStreetSegment>> intersections;


bool load_map(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully

    std::cout << "load_map: " << map_streets_database_filename << std::endl;
    loadStreetsDatabaseBIN(map_streets_database_filename);
    //
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
        streets[sgmt.streetID].push_back(sgmt);
        
    }
    int nIntersections=getNumIntersections();
    for(int i = 0;i<nIntersections;i++){
        int nSegs = getIntersectionStreetSegmentCount(i);
        std::vector<InfoStreetSegment> segmentsAtIntersection;
        for(int j=0;j<nSegs;j++){
            segmentsAtIntersection.push_back(getInfoStreetSegment(getIntersectionStreetSegment(i,j)));
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

double find_distance_between_two_points(std::pair<LatLon, LatLon> points){
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
double find_street_segment_travel_time(int street_segment_id){return 0;}
int find_closest_intersection(LatLon my_position){return 0;}
std::vector<int> find_street_segments_of_intersection(int intersection_id){return std::vector<int>();}
std::vector<std::string> find_street_names_of_intersection(int intersection_id){return std::vector<std::string>();}
bool are_directly_connected(std::pair<int, int> intersection_ids){return false;}
std::vector<int> find_adjacent_intersections(int intersection_id){return std::vector<int>();}
std::vector<int> find_street_segments_of_street(int street_id){return std::vector<int>();}
std::vector<int> find_intersections_of_street(int street_id){return std::vector<int>();}
std::vector<int> find_intersections_of_two_streets(std::pair<int, int> street_ids){return std::vector<int>();}
std::vector<int> find_street_ids_from_partial_street_name(std::string street_prefix){return std::vector<int>();}
double find_feature_area(int feature_id){return 0;}
double find_way_length(OSMID way_id){return 0;}