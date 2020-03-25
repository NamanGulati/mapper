#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include <queue>
#include "globals.h"
#include "m1.h"
#include "helpers.h"
#include <chrono>
#include <thread>
#include <iostream>


void delay (int milliseconds) ;

std::vector<StreetSegmentIndex> uber_pool_test(
                          const IntersectionIndex start_intersection, 
                          const IntersectionIndex end_intersection,
                          const double turn_penalty);
double heuristic(IntersectionIndex current, IntersectionIndex destination);
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty);
double compute_segment_walking_time(StreetSegmentIndex seg, const double walking_speed);


double compute_path_travel_time(const std::vector<StreetSegmentIndex>& path, const double turn_penalty){
    if(path.size()==0)
        return 0;
    double sum=0;
    for(int i=1;i<path.size();i++){
        InfoStreetSegment seg = getInfoStreetSegment(path[i]);
        sum+=find_street_segment_travel_time(path[i]);
        
        if(seg.streetID!=getInfoStreetSegment(path[i-1]).streetID)
            sum+=turn_penalty;
    }
    sum+=find_street_segment_travel_time(path[0]);
    return sum;
}

double compute_path_walking_time(const std::vector<StreetSegmentIndex>& path, const double walking_speed, const double turn_penalty){
    if(path.size() == 0)
        return 0;
    double sum=0;
    for(int i=0;i<path.size()-1;i++){
        InfoStreetSegment seg = getInfoStreetSegment(path[i]);
        sum+=find_street_segment_length(path[i])/walking_speed;
        if(seg.streetID!=getInfoStreetSegment(path[i+1]).streetID)
            sum+=turn_penalty;
    }
    sum+=find_street_segment_length(path.back())/walking_speed;
    return sum;
}



std::vector<StreetSegmentIndex> find_path_between_intersections(const IntersectionIndex intersect_id_start, const IntersectionIndex intersect_id_end, const double turn_penalty){
    std::priority_queue<segIntersectionData,std::vector<segIntersectionData>, segIntersectionDataComparator> openSet;
    std::vector<StreetSegmentIndex>cameFrom(getNumStreetSegments());
    std::vector<bool> visited(getNumIntersections(),false);
    std::vector<double> gScore(adjacencyList.size(),DBL_MAX);
    gScore[intersect_id_start]=0;

    openSet.emplace(intersect_id_start,-1,heuristic(intersect_id_start,intersect_id_end));

   // ezgl::renderer * g = appl->get_renderer();
    
    while(!openSet.empty()){
        segIntersectionData current = openSet.top();
        if(current.intersection==intersect_id_end){
            int temp = current.segment;
            std::vector<StreetSegmentIndex> path;
            while(temp != -1){
                path.push_back(temp);
                temp = cameFrom[temp];
            }
            std::reverse(path.begin(),path.end());
            return path;
        }
        visited[current.intersection] = true;
        openSet.pop();
        for(segIntersectionData neighbor : adjacencyList[current.intersection]){
            //drawPathStreetSegment(g,segmentData[neighbor.segment],&ezgl::BLUE);
            //appl->flush_drawing();
            double tentative_gScore = gScore[current.intersection] + get_segment_cost(current.segment,neighbor.segment,turn_penalty);

            if(!visited[neighbor.intersection]&&tentative_gScore < gScore[neighbor.intersection]){
                cameFrom[neighbor.segment] = current.segment;
                gScore[neighbor.intersection]= tentative_gScore;
                neighbor.distance = tentative_gScore + heuristic(current.intersection,neighbor.intersection);
                openSet.push(neighbor);
            }
        }

    }
    return std::vector<StreetSegmentIndex>(0);

}


double heuristic(IntersectionIndex current, IntersectionIndex destination){
    return find_distance_between_two_points(std::make_pair(getIntersectionPosition(current),getIntersectionPosition(destination)));
}
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty){
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
    std::vector<segIntersectionData> parent(getNumStreetSegments()); //the path vector
    std::vector<bool> visited(getNumIntersections(),false);
    segIntersectionData top; //top of the pq
    std::vector<StreetSegmentIndex> walk;
    std::vector<segIntersectionData> drivingStart; //vector to start the driving search from
    
    //ezgl::renderer * g = appl->get_renderer(); //drawing for testing
    
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
                parent[currInter.segment] = top; //keep track of path //modified
                int prevStreet = top.segment;   //modified
                if (prevStreet >= 0){
                    InfoStreetSegment previous = getInfoStreetSegment(prevStreet); 
                    InfoStreetSegment current = getInfoStreetSegment(currInter.segment);

                    if(current.streetID != previous.streetID){
                        dist[currInter.intersection] += turn_penalty;
                    }
                }
                pq.emplace(currInter.intersection, currInter.segment, dist[currInter.intersection]);
            } 
        }
    }
    
    
    std::vector<StreetSegmentIndex> drive;
    
    
    segIntersectionData placeHolder; //This is the beginning of the drive path.
    segIntersectionData x = placeHolder;
    while(x.segment != -1){
        walk.push_back(x.segment);
        x = parent[x.segment];
    }
    std::reverse(walk.begin(), walk.end());
    std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> result(walk, drive);
    return result;
}

double compute_segment_walking_time(StreetSegmentIndex seg, const double walking_speed){
    return find_street_segment_length(seg)/walking_speed;
}



std::vector<StreetSegmentIndex> uber_pool_test(
                          const IntersectionIndex start_intersection, 
                          const IntersectionIndex end_intersection,
                          const double turn_penalty){

    
    std::vector<double> dist(adjacencyList.size(),INT_MAX); 
    std::vector<segIntersectionData> parent(getNumStreetSegments()); //modified
    std::vector<bool> visited(getNumIntersections(),false);
    segIntersectionData top; //top of the pq
    std::vector<StreetSegmentIndex> walk;
    //std::vector<segIntersectionData> drivingStart; //vector to start the driving search from
    
    //ezgl::renderer * g = appl->get_renderer(); //drawing for testing
    
    std::priority_queue < segIntersectionData, std::vector<segIntersectionData>, segIntersectionDataComparator > pq;
    pq.emplace(start_intersection, -1, 0);
    dist[start_intersection] = 0;
    
    while (!pq.empty()){
        top = pq.top(); //break when end is on top
        if (top.intersection == end_intersection) //within walking distance
            break;
        visited[top.intersection] = true;
        //drivingStart.push_back(top);
        pq.pop();
         
        for (int x = 0; x < adjacencyList[top.intersection].size(); x ++){
            segIntersectionData currInter = adjacencyList[top.intersection][x];
            //drawPathStreetSegment(g,segmentData[currInter.segment],&ezgl::BLUE);
            //appl->flush_drawing();
            double weight = find_street_segment_travel_time(currInter.segment);
            if (visited[currInter.intersection] == false && dist[currInter.intersection] > dist[top.intersection] + weight) 
            { 
                // Updating distance of current Intersection 
                dist[currInter.intersection] = dist[top.intersection] + weight; 
                parent[currInter.segment] = top; //keep track of path //modified
                int prevStreet = top.segment;   //modified
                if (prevStreet >= 0){
                    InfoStreetSegment previous = getInfoStreetSegment(prevStreet); 
                    InfoStreetSegment current = getInfoStreetSegment(currInter.segment);

                    if(current.streetID != previous.streetID){
                        dist[currInter.intersection] += turn_penalty;
                    }
                }
                pq.emplace(currInter.intersection, currInter.segment, dist[currInter.intersection]);
            } 
        }
    }
    segIntersectionData x = top; ////////////////////////modifications
    while(x.segment != -1){
        walk.push_back(x.segment);
        x = parent[x.segment];
    }
    std::reverse(walk.begin(), walk.end());
    return walk;
}
