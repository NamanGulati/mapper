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
    int nodeLength = adjacencyList.size();
    int dist[nodeLength], parent[nodeLength];
    int top;
    int topWeight;
    std::vector<StreetSegmentIndex> path;
    
    
    for (int x = 0; x < nodeLength; x ++){
        dist[x] = INT_MAX;
        parent[x] = -1;
    }
    dist[intersect_id_start] = 0;
    
    std::priority_queue < intPair, std::vector<intPair>, pairCompareIntInt > pq;
    pq.push(std::make_pair(intersect_id_start, dist[intersect_id_start]));
    
    while (!pq.empty()){
        top = pq.top().first; //break when end is on top
        if (top = intersect_id_end)
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