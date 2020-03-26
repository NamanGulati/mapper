#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include <queue>
#include "globals.h"
#include "m1.h"
#include "helpers.h"
#include <chrono>
#include <thread>
#include <iostream>

//#define drawAlgos

void delay (int milliseconds) ;

std::vector<StreetSegmentIndex> uber_pool_test(
                          const IntersectionIndex start_intersection, 
                          const IntersectionIndex end_intersection,
                          const double turn_penalty);
double heuristic(IntersectionIndex current, IntersectionIndex destination);
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty);
double compute_segment_walking_time(StreetSegmentIndex seg, const double walking_speed);
double get_segment_walk_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty, const double walking_speed);
std::vector<StreetSegmentIndex> drive(const std::vector<IntersectionIndex> intersect_id_start, const IntersectionIndex intersect_id_end, const double turn_penalty);


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
    std::vector<double> gScore(adjacencyList.size(),DBL_MAX);
    gScore[intersect_id_start]=0;

    openSet.emplace(intersect_id_start,-1,0);

    #ifdef drawAlgos
        ezgl::renderer * g = appl->get_renderer();
    #endif
    
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
        openSet.pop();
        if(current.distance <= gScore[current.intersection]){
            gScore[current.intersection] = current.distance;
            
        
            for(segIntersectionData neighbor : adjacencyList[current.intersection]){

                #ifdef drawAlgos
                    drawPathStreetSegment(g,segmentData[neighbor.segment],&ezgl::BLUE);
                    appl->flush_drawing();
                #endif

                double tentative_gScore = gScore[current.intersection] + get_segment_cost(current.segment,neighbor.segment,turn_penalty);

                if(tentative_gScore < gScore[neighbor.intersection]){
                    cameFrom[neighbor.segment] = current.segment;
                    gScore[neighbor.intersection]= tentative_gScore + (heuristic(neighbor.intersection, intersect_id_end)*3.6/maxSpeedLim);
                    neighbor.distance = tentative_gScore;
                    openSet.push(neighbor);
                }
            }
        }

    }
    return std::vector<StreetSegmentIndex>(0);

}


double heuristic(IntersectionIndex current, IntersectionIndex destination){
    //return find_distance_between_two_points(std::make_pair(getIntersectionPosition(current),getIntersectionPosition(destination)));
   ezgl::point2d cur (LatLonTo2d(getIntersectionPosition(current)));
   ezgl::point2d dest (LatLonTo2d(getIntersectionPosition(destination)));
   return std::hypot(dest.x - cur.x, dest.y - cur.y)*EARTH_RADIUS_METERS;
}
double get_segment_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty){
    if(current==-1)
        return find_street_segment_travel_time(next);
    return  find_street_segment_travel_time(next) + ((getInfoStreetSegment(current).streetID!=getInfoStreetSegment(next).streetID)?turn_penalty:0);
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

    
    std::priority_queue<segIntersectionData,std::vector<segIntersectionData>, segIntersectionDataComparator> openSet;
    std::vector<StreetSegmentIndex>cameFrom(getNumStreetSegments());
    std::vector<StreetSegmentIndex>cameFromInter(getNumIntersections());
    std::vector<double> gScore(adjacencyListWalking.size(),DBL_MAX);
    std::vector <IntersectionIndex> drivingStart;
    gScore[start_intersection]=0;

    openSet.emplace(start_intersection,-1,0);

    #ifdef drawAlgos
        ezgl::renderer * g = appl->get_renderer();
    #endif
    segIntersectionData current;
    while(!openSet.empty()){
        current = openSet.top();
        drivingStart.push_back(current.intersection);
        if(current.distance >= walking_time_limit || current.intersection == end_intersection){
            break; //Used code below to recall path
            
        }
        
        openSet.pop();
        if(current.distance <= gScore[current.intersection]){
            gScore[current.intersection] = current.distance;
            
        
            for(segIntersectionData neighbor : adjacencyListWalking[current.intersection]){

                #ifdef drawAlgos
                    drawPathStreetSegment(g,segmentData[neighbor.segment],&ezgl::BLUE);
                    appl->flush_drawing();
                #endif

                double tentative_gScore = gScore[current.intersection] + get_segment_walk_cost(current.segment,neighbor.segment,turn_penalty, walking_speed);

                if(tentative_gScore < gScore[neighbor.intersection]){
                    cameFrom[neighbor.segment] = current.segment;
                    cameFromInter[neighbor.intersection] = neighbor.segment; 
                    gScore[neighbor.intersection]= tentative_gScore;
                    neighbor.distance = tentative_gScore;
                    openSet.push(neighbor);
                }
            }
        }

    }
    
    std::vector<StreetSegmentIndex> drivePath = drive(drivingStart, end_intersection, turn_penalty);
    int pathStart;
    if (drivePath.size() > 0){
        if (drivePath.size() > 1){
            if (getInfoStreetSegment(drivePath[0]).from == getInfoStreetSegment(drivePath[1]).from || getInfoStreetSegment(drivePath[0]).from == getInfoStreetSegment(drivePath[1]).to){
                pathStart = getInfoStreetSegment(drivePath[0]).to;
            }
            else{
                pathStart = getInfoStreetSegment(drivePath[0]).from;
            }
        }
        else{
            if (getInfoStreetSegment(drivePath[0]).from == end_intersection){
                pathStart = getInfoStreetSegment(drivePath[0]).to;
            }
            else{
                pathStart = getInfoStreetSegment(drivePath[0]).from;
            }
        }
    }
    pathStart = cameFromInter[pathStart];
    std::vector<StreetSegmentIndex> path;
    while(pathStart != -1){
        path.push_back(pathStart);
        pathStart = cameFrom[pathStart];
    }
    std::reverse(path.begin(),path.end());
    
//    if (current.intersection == end_intersection){
//        int temp = current.segment;
//        
//        if (get_segment_walk_cost(cameFrom[temp],temp, turn_penalty, walking_speed) >  walking_time_limit){ //watch out for turn penalty
//            drivingStart.pop_back();
//            std::vector<segIntersectionData> startInter = drive(drivingStart, end_intersection, turn_penalty);
//            if (drivingStart.back().intersection == start_intersection){
//                for (int x = 0; x < startInter.size(); x ++){
//                    drivePath.push_back(startInter[x].segment);
//                }    
//            }
//            else{
//                for (int x = 1; x < startInter.size(); x ++){
//                    drivePath.push_back(startInter[x].segment);
//                }    
//            }
//            
//        }
//        else{
//            while(temp != -1){
//                path.push_back(temp);
//                temp = cameFrom[temp];
//            }
//            std::reverse(path.begin(),path.end()); 
//        }
//        
//        
//    }
//    else{
//        std::cout << "Current distance is:  " << current.distance << std::endl;
//        if (current.distance > walking_time_limit)
//            drivingStart.pop_back();
//        std::cout << "Driving Start is:  " << drivingStart[0].intersection << std::endl;
//        std::vector<segIntersectionData> startInter = drive(drivingStart, end_intersection, turn_penalty);
//        if (!startInter.empty()){
//            
//            if (current.intersection != start_intersection && drivingStart.size() != 1){ 
//                
//                for (int x = 1; x < startInter.size(); x ++){
//                    drivePath.push_back(startInter[x].segment);
//                }
//                if (drivingStart.size() != 1){
//                    int temp = startInter[0].segment;
//                    while(temp != -1){
//                        path.push_back(temp);
//                        temp = cameFrom[temp];
//                    }
//                    std::reverse(path.begin(),path.end());
//                }
//                
//            }
//            else{
//                for (int x = 0; x < startInter.size(); x ++){
//                    drivePath.push_back(startInter[x].segment);
//                }
//            }
//            
//        }
//
//        
//    }
    
    std::cout << "Walking path size is " << path.size() << std::endl;
    std::cout << "Driving path size is " << drivePath.size() << std::endl;
   
    std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> result(path, drivePath);
    return result;
}

std::vector<StreetSegmentIndex> drive(const std::vector<IntersectionIndex> intersect_id_start, const IntersectionIndex intersect_id_end, const double turn_penalty){
    std::priority_queue<segIntersectionData,std::vector<segIntersectionData>, segIntersectionDataComparator> openSet;
    std::vector<StreetSegmentIndex>cameFrom(getNumStreetSegments());
    std::vector<double> gScore(adjacencyList.size(),DBL_MAX);
    for(int x = 0; x < intersect_id_start.size(); x ++){
        gScore[intersect_id_start[x]]=0;

        openSet.emplace(intersect_id_start[x],-1,0);    
    }
    

    #ifdef drawAlgos
        ezgl::renderer * g = appl->get_renderer();
    #endif
    
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
        openSet.pop();
        if(current.distance <= gScore[current.intersection]){
            gScore[current.intersection] = current.distance;
            
        
            for(segIntersectionData neighbor : adjacencyList[current.intersection]){

                #ifdef drawAlgos
                    drawPathStreetSegment(g,segmentData[neighbor.segment],&ezgl::BLUE);
                    appl->flush_drawing();
                #endif

                double tentative_gScore = gScore[current.intersection] + get_segment_cost(current.segment,neighbor.segment,turn_penalty);

                if(tentative_gScore < gScore[neighbor.intersection]){
                    cameFrom[neighbor.segment] = current.segment;
                    gScore[neighbor.intersection]= tentative_gScore + (heuristic(neighbor.intersection, intersect_id_end)*3.6/maxSpeedLim);
                    neighbor.distance = tentative_gScore;
                    openSet.push(neighbor);
                }
            }
        }

    }
    return std::vector<StreetSegmentIndex>(0);

}

//std::vector<segIntersectionData> drive(const std::vector<segIntersectionData> intersect_id_start, const IntersectionIndex intersect_id_end, const double turn_penalty){
//    std::priority_queue<segIntersectionData,std::vector<segIntersectionData>, segIntersectionDataComparator> openSet;
//    std::vector<segIntersectionData>cameFrom(getNumStreetSegments());
//    std::vector<double> gScore(adjacencyList.size(),DBL_MAX);
//    for(int x = 0; x < intersect_id_start.size(); x ++){
//        gScore[intersect_id_start[x].intersection]=0;
//
//        openSet.emplace(intersect_id_start[x].intersection,intersect_id_start[x].segment,0); 
//    }
//    
//
//    #ifdef drawAlgos
//        ezgl::renderer * g = appl->get_renderer();
//    #endif
//    
//    while(!openSet.empty()){
//        segIntersectionData current = openSet.top();
//        if(current.intersection==intersect_id_end){
//            segIntersectionData temp = current;
//            std::vector<segIntersectionData> path;
//            while(temp.segment != -1){
//                path.push_back(temp);
//                temp = cameFrom[temp.segment];
//            }
//            std::reverse(path.begin(),path.end());
//            return path;
//        }
//        openSet.pop();
//        if(current.distance <= gScore[current.intersection]){
//            gScore[current.intersection] = current.distance;
//            
//        
//            for(segIntersectionData neighbor : adjacencyList[current.intersection]){
//
//                #ifdef drawAlgos
//                    drawPathStreetSegment(g,segmentData[neighbor.segment],&ezgl::BLUE);
//                    appl->flush_drawing();
//                #endif
//
//                double tentative_gScore = gScore[current.intersection] + get_segment_cost(current.segment,neighbor.segment,turn_penalty);
//
//                if(tentative_gScore < gScore[neighbor.intersection]){
//                    cameFrom[neighbor.segment] = current;
//                    gScore[neighbor.intersection]= tentative_gScore + (heuristic(neighbor.intersection, intersect_id_end)*3.6/maxSpeedLim);
//                    neighbor.distance = tentative_gScore;
//                    openSet.push(neighbor);
//                }
//            }
//        }
//
//    }
//    std::vector<segIntersectionData> nothing;
//    return nothing;
//
//}

double compute_segment_walking_time(StreetSegmentIndex seg, const double walking_speed){
    return find_street_segment_length(seg)/walking_speed;
}

double get_segment_walk_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty, const double walking_speed){
    if(current==-1)
        return compute_segment_walking_time(next, walking_speed);
    return  compute_segment_walking_time(next, walking_speed) + ((getInfoStreetSegment(current).streetID!=getInfoStreetSegment(next).streetID)?turn_penalty:0);
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
            double weight = get_segment_cost(top.segment, currInter.segment, turn_penalty);
            if (visited[currInter.intersection] == false && dist[currInter.intersection] > dist[top.intersection] + weight) 
            { 
                // Updating distance of current Intersection 
                dist[currInter.intersection] = dist[top.intersection] + weight; 
                parent[currInter.segment] = top; //keep track of path //modified           
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
