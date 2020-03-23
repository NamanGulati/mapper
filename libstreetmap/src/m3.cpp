#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include <queue>
#include "globals.h"
#include "m1.h"
#include "helpers.h"

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



double heuristic(IntersectionIndex current, IntersectionIndex destination);
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next, IntersectionIndex intersction, const double turn_penalty);
double compute_segment_walking_time(StreetSegmentIndex seg, const double walking_speed);

double compute_path_travel_time(const std::vector<StreetSegmentIndex>& path, const double turn_penalty){

    int sum=0;
    for(int i=0;i<path.size()-1;i++){
        InfoStreetSegment seg = getInfoStreetSegment(path[i]);
        sum+=segLen[i]*seg.speedLimit;
        if(seg.streetID!=getInfoStreetSegment(path[i+1]).streetID)
            sum+=turn_penalty;
    }
    return sum;
}

double compute_path_walking_time(const std::vector<StreetSegmentIndex>& path, const double walking_speed, const double turn_penalty){
    int sum=0;
    for(int i=0;i<path.size()-1;i++){
        InfoStreetSegment seg = getInfoStreetSegment(path[i]);
        sum+=segLen[i]*walking_speed;
        if(seg.streetID!=getInfoStreetSegment(path[i+1]).streetID)
            sum+=turn_penalty;
    }
    return sum;
}


std::vector<StreetSegmentIndex> find_path_between_intersections(const IntersectionIndex intersect_id_start, const IntersectionIndex intersect_id_end, const double turn_penalty){

    highlightedSegs.clear();
    std::priority_queue<segIntersectionData,std::vector<segIntersectionData>, segIntersectionDataComparator> priorityQueue;
    std::vector<bool> visited(getNumIntersections(),false);
    std::vector<StreetSegmentIndex>pathTaken(getNumStreetSegments(),-1);
    int currIntersection=intersect_id_start;
    int lastSeg = -1;
    priorityQueue.push(segIntersectionData(currIntersection,-1));
    std::vector<double> intersectionCost(getNumPointsOfInterest(),-1);
    intersectionCost[0]=0;
    ezgl::renderer * g = appl->get_renderer();
    while(!priorityQueue.empty() && currIntersection!=intersect_id_end){
        currIntersection = priorityQueue.top().intersection;
        lastSeg = priorityQueue.top().intersection;
        visited[currIntersection]=true;
        priorityQueue.pop();

        for(int i=0;i<adjacencyList[currIntersection].size();i++){
            segIntersectionData dat = adjacencyList[currIntersection][i];
            double currentCost = intersectionCost[i] + get_segment_cost(lastSeg,dat.segment,currIntersection,turn_penalty);
            highlightedSegs.push_back(dat.segment);
            appl->refresh_drawing();
            if(!visited[dat.intersection]&&(intersectionCost[dat.intersection]!=-1 || intersectionCost[dat.intersection] > currentCost)){
                intersectionCost[dat.intersection] = currentCost;
                dat.distance = currentCost + heuristic(dat.intersection, intersect_id_end);
                priorityQueue.push(dat);
                pathTaken[dat.segment]=lastSeg;
            }
        }

    }

    std::vector<StreetSegmentIndex> path;
    if(priorityQueue.empty())
        return path;
    
    int temp = lastSeg;
    while(temp != -1){
        path.push_back(temp);
        temp = pathTaken[temp];
    }

    std::reverse(path.begin(),path.end());
    highlightedSegs.clear();
    return path;
}

double heuristic(IntersectionIndex current, IntersectionIndex destination){
    return find_distance_between_two_points(std::make_pair(getIntersectionPosition(current),getIntersectionPosition(destination)));
}
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next, IntersectionIndex intersction, const double turn_penalty){
    double cost =  find_street_segment_travel_time(next);
    TurnType tt = determineDirection(getIntersectionPosition(intersction),getLastCurvePoint(current),getFirstCurvePoint(next));
    if(tt==TurnType::LEFT||tt==TurnType::RIGHT)
        cost+=turn_penalty;
    return cost;
}




std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> //check units
         find_path_with_walk_to_pick_up(
                          const IntersectionIndex start_intersection, 
                          const IntersectionIndex end_intersection,
                          const double turn_penalty,
                          const double walking_speed, 
                          const double walking_time_limit){
    
    std::vector<int> dist(adjacencyList.size(),INT_MAX);
    std::vector<segIntersectionData> parent(adjacencyList.size());
    std::vector<bool> visited(getNumIntersections(),false);
    segIntersectionData top;
    std::vector<StreetSegmentIndex> walk;
    
    
    std::priority_queue < segIntersectionData, std::vector<segIntersectionData>, segIntersectionDataComparator > pq;
    pq.emplace(start_intersection, -1, 0);
    dist[start_intersection] = 0;
    
    while (!pq.empty()){
        top = pq.top(); //break when end is on top
        if (dist[top.intersection] >= walking_time_limit)
            break;
        visited[top.intersection] = true;
        pq.pop();
         
        for (int x = 0; x < adjacencyListWalking[top.intersection].size(); x ++){
            segIntersectionData currInter = adjacencyListWalking[top.intersection][x];
            double walkTime = compute_segment_walking_time(currInter.segment, walking_speed);
            //std::pair<int, double> currInter(segInter.intersection, compute_segment_walking_time(segInter.segment, walking_speed));
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
                //pq.push(std::make_pair(currInter.first, dist[currInter.first]));
            } 
        }
    }
    int x = top.intersection;
    while(parent[x].intersection != -1){
        walk.push_back(parent[x].segment);
        x = parent[x].intersection;
    }
    std::reverse(walk.begin(), walk.end());
    std::vector<StreetSegmentIndex> drive;
    
    
    std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> result(walk, drive);
    return result;
}

double compute_segment_walking_time(StreetSegmentIndex seg, const double walking_speed){
    return find_street_segment_length(seg)/walking_speed * 3.6; //dont know if conversion is needed
}