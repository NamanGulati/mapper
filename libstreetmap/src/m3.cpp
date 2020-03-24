#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include <queue>
#include "globals.h"
#include "m1.h"
#include "helpers.h"
#include <chrono>
#include <thread>


//typedef std::pair<int, double> intDoubPair;
//
//class pairCompareIntDouble
//{ 
//public: 
//    double operator() (const intDoubPair& p1, const intDoubPair& p2) 
//    { 
//        return p1.second > p2.second; 
//    } 
//};

void delay (int milliseconds) ;


double heuristic(IntersectionIndex current, IntersectionIndex destination);
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next, IntersectionIndex intersction, const double turn_penalty);
double compute_segment_walking_time(StreetSegmentIndex seg, const double walking_speed);

double compute_path_travel_time(const std::vector<StreetSegmentIndex>& path, const double turn_penalty){

    double sum=0;
    for(int i=0;i<path.size()-1;i++){
        InfoStreetSegment seg = getInfoStreetSegment(path[i]);
        sum+=find_street_segment_travel_time(path[i]);
        if(seg.streetID!=getInfoStreetSegment(path[i+1]).streetID)
            sum+=turn_penalty;
    }
    sum+=find_street_segment_travel_time(path.back());
    return sum;
}

double compute_path_walking_time(const std::vector<StreetSegmentIndex>& path, const double walking_speed, const double turn_penalty){
    double sum=0;
    for(int i=0;i<path.size()-1;i++){
        InfoStreetSegment seg = getInfoStreetSegment(path[i]);
        sum+=segLen[path[i]]*walking_speed*3.6;
        if(seg.streetID!=getInfoStreetSegment(path[i+1]).streetID)
            sum+=turn_penalty;
    }
    sum+=segLen[path.back()];
    return sum;
}



std::vector<StreetSegmentIndex> find_path_between_intersections(const IntersectionIndex intersect_id_start, const IntersectionIndex intersect_id_end, const double turn_penalty){
    std::priority_queue<segIntersectionData,std::vector<segIntersectionData>, segIntersectionDataComparator> openSet;
    std::vector<segIntersectionData>cameFrom(adjacencyList.size());
    std::vector<double> gScore(adjacencyList.size(),DBL_MAX);
    gScore[intersect_id_start]=0;

    openSet.emplace(intersect_id_start,-1,heuristic(intersect_id_start,intersect_id_end)*10);
    ezgl::renderer * g = appl->get_renderer();
    
    while(!openSet.empty()){
        segIntersectionData current = openSet.top();
        if(current.intersection==intersect_id_end){
            segIntersectionData temp = current;
            std::vector<StreetSegmentIndex> path;
            while(temp.segment != -1){
                path.push_back(temp.segment);
                temp = cameFrom[temp.intersection];
            }
            path.push_back(current.segment);
            std::reverse(path.begin(),path.end());
            return path;
        }
        openSet.pop();
        for(segIntersectionData neighbor : adjacencyList[current.intersection]){
            drawPathStreetSegment(g,segmentData[neighbor.segment],&ezgl::BLUE);
            appl->flush_drawing();
            double tentative_gScore = gScore[current.intersection] + get_segment_cost(current.segment,neighbor.segment,current.intersection,turn_penalty);
            if(tentative_gScore < gScore[neighbor.intersection]){
                cameFrom[neighbor.intersection] = current;
                gScore[neighbor.intersection]= tentative_gScore;
                neighbor.distance = tentative_gScore + heuristic(current.intersection,neighbor.intersection)*10;
                openSet.push(neighbor);
            }
        }
    }
    return std::vector<StreetSegmentIndex>(0);

}


double heuristic(IntersectionIndex current, IntersectionIndex destination){
    return find_distance_between_two_points(std::make_pair(getIntersectionPosition(current),getIntersectionPosition(destination)));
}
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next, IntersectionIndex intersction, const double turn_penalty){
    if(current==-1)
        return find_street_segment_travel_time(next);
    return  find_street_segment_travel_time(next) + (getInfoStreetSegment(current).streetID!=getInfoStreetSegment(next).streetID?turn_penalty:0);
}


    
void delay (int milliseconds) {  // Pause for milliseconds
    std::chrono::milliseconds duration(milliseconds);
    std::this_thread::sleep_for(duration);
}

std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> //check units
         find_path_with_walk_to_pick_up(
                          const IntersectionIndex start_intersection, 
                          const IntersectionIndex end_intersection,
                          const double turn_penalty,
                          const double walking_speed, 
                          const double walking_time_limit){

    
    std::vector<double> dist(adjacencyListWalking.size(),INT_MAX); 
    std::vector<segIntersectionData> parent(adjacencyListWalking.size()); //the path vector
    std::vector<bool> visited(getNumIntersections(),false);
    segIntersectionData top; //top of the pq
    std::vector<StreetSegmentIndex> walk;
    std::vector<segIntersectionData> drivingStart; //vector to start the driving search from
    
    ezgl::renderer * g = appl->get_renderer(); //drawing for testing
    
    std::priority_queue < segIntersectionData, std::vector<segIntersectionData>, segIntersectionDataComparator > pq;
    pq.emplace(start_intersection, -1, 0);
    dist[start_intersection] = 0;
    
    while (!pq.empty()){
        top = pq.top(); //break when end is on top
        if (dist[top.intersection] >= walking_time_limit || top.intersection == end_intersection) //within walking distance
            break;
        visited[top.intersection] = true;
        drivingStart.push_back(top);
        pq.pop();
         
        for (int x = 0; x < adjacencyListWalking[top.intersection].size(); x ++){
            segIntersectionData currInter = adjacencyListWalking[top.intersection][x];
            double walkTime = compute_segment_walking_time(currInter.segment, walking_speed);
            if (visited[currInter.intersection] == false && dist[currInter.intersection] > dist[top.intersection] + walkTime) 
            { 
                // Updating distance of current Intersection 
                dist[currInter.intersection] = dist[top.intersection] + walkTime; 
                parent[currInter.intersection] = top; //keep track of path
                InfoStreetSegment previous = getInfoStreetSegment(parent[currInter.intersection].segment); 
                InfoStreetSegment current = getInfoStreetSegment(currInter.segment);
                if(current.streetID != previous.streetID){
                    dist[currInter.intersection] += turn_penalty;
                }
                pq.emplace(currInter.intersection, currInter.segment, dist[currInter.intersection]);
            } 
        }
    }
    int x = top.intersection;
    while(parent[x].intersection != -1){
        walk.push_back(parent[x].segment);
        x = parent[x].intersection;
    }
    std::reverse(walk.begin(), walk.end());
    
    //std::vector<StreetSegmentIndex> drive = find_path_between_intersections(const IntersectionIndex intersect_id_start, const IntersectionIndex intersect_id_end, turn_penalty);
    
    std::vector<StreetSegmentIndex> drive;
    std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> result(walk, drive);
    return result;
}

double compute_segment_walking_time(StreetSegmentIndex seg, const double walking_speed){
    return find_street_segment_length(seg)/walking_speed * 3.6; //dont know if conversion is needed
}

void printDirections(std::vector<StreetSegmentIndex> path){
    int totalPathDistance = getTotalPathDistance(path);

    for(int i = 1; i < path.size(); i++){
        
    }
}