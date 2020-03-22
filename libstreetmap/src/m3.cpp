#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include <queue>
#include "globals.h"
#include "m1.h"
#include "helpers.h"

typedef std::pair<int, int> intPair;

class pairCompareIntInt
{ 
public: 
    int operator() (const intPair& p1, const intPair& p2) 
    { 
        return p1.second > p2.second; 
    } 
};

double heuristic(IntersectionIndex current, IntersectionIndex destination);
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next);

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

    std::priority_queue<segIntersectionData,std::vector<segIntersectionData>, segIntersectionDataComparator> priorityQueue;
    std::vector<bool> visited(getNumIntersections(),false);
    std::vector<StreetSegmentIndex>pathTaken(getNumStreetSegments(),-1);
    int currIntersection=intersect_id_start;
    int lastSeg = -1;
    priorityQueue.push(segIntersectionData(currIntersection,-1));
    std::vector<double> intersectionCost(getNumPointsOfInterest(),-1);
    intersectionCost[0]=0;

    while(!priorityQueue.empty() && currIntersection!=intersect_id_end){
        currIntersection = priorityQueue.top().intersection;
        lastSeg = priorityQueue.top().intersection;
        visited[currIntersection]=true;
        priorityQueue.pop();

        for(int i=0;i<adjacencyList[currIntersection].size();i++){
            segIntersectionData dat = adjacencyList[i];
            double currentCost = intersectionCost[i] + get_segment_cost(lastSeg,dat.segment);

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
    return path;
}

double heuristic(IntersectionIndex current, IntersectionIndex destination){
    return find_distance_between_two_points(std::make_pair(getIntersectionPosition(current),getIntersectionPosition(destination)));
}
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next){
    return find_street_segment_travel_time(next);
}

std::vector<StreetSegmentIndex> find_path_between_intersections_temp(const IntersectionIndex intersect_id_start, const IntersectionIndex intersect_id_end, const double turn_penalty){
    int nodeLength = adjacencyList.size();
    std::vector<int> dist(adjacencyList.size(),INT_MAX);
    std::vector<int> parent(adjacencyList.size(),-1);
    int top;
    int topWeight;
    std::vector<StreetSegmentIndex> path;
    dist[intersect_id_start] = 0;
    
    std::priority_queue < intPair, std::vector<intPair>, pairCompareIntInt > pq;
    pq.push(std::make_pair(intersect_id_start, dist[intersect_id_start]));
    
    while (!pq.empty()){
        top = pq.top().first; //break when end is on top
        if (top == intersect_id_end)
            break;
        topWeight = pq.top().second;
        pq.pop();
         
        for (int x = 0; x < adjacencyList[top].size(); x ++){
            int interId = adjacencyList[top][x].first;
            std::pair<int, int> currInter(interId, find_street_segment_travel_time(adjacencyList[top][x].second));
            if (dist[currInter.first] > dist[top] + currInter.second) 
            { 
                // Updating distance of current Intersection 
                dist[currInter.first] = dist[top] + currInter.second; 
                parent[currInter.first] = top; //keep track of path
                pq.push(std::make_pair(currInter.first, dist[currInter.first]));
            } 
        }
        
    }
    int x = top;
    while(parent[x] != -1){
        path.push_back(parent[x]);
        x = parent[x];
    }
    std::reverse(path.begin(), path.end());
    return path;
}