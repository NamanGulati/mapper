#include "m4.h"
#include <vector>
#include <map>
#include "StreetsDatabaseAPI.h"
#include "globals.h"
#include "m3.h"
#include "m1.h"
#include <queue>
#include <list>
#include <set>
#include <iostream>
#include "helpers.h"
#include "courier_verify.h"
#include <chrono>

//#define drawAlgos
std::unordered_map<IntersectionIndex, std::unordered_map<IntersectionIndex, PathData>> travelTimes;

struct pickDrop{
        int packageIndex=-1;
        IntersectionIndex intersection=-1;
        bool pickOrDrop=true; //true = pickup
        float itemWeight;
};

double get_seg_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty);
double computeTravelTime(std::vector<pickDrop> deliveryOrder);
bool isLegal(std::vector<pickDrop> deliveryOrder, const float truck_capacity);
std::vector<pickDrop> twoOptSwap(std::vector<pickDrop> deliveryOrder, int first, int second);
std::vector<pickDrop> simpleSwap(std::vector<pickDrop> deliveryOrder, int first, int second);

std::vector<CourierSubpath> traveling_courier(const std::vector<DeliveryInfo> &deliveries, const std::vector<int> &depots, const float turn_penalty, const float truck_capacity)
{
    auto startTime = std::chrono::high_resolution_clock::now();
    bool timeOut = false;
    
    std::vector<int> deliveryPoints;
    for (int i = 0; i < deliveries.size(); i++)
    {
        deliveryPoints.push_back(deliveries[i].dropOff);
        deliveryPoints.push_back(deliveries[i].pickUp);
    }
    deliveryPoints.insert(deliveryPoints.end(), depots.begin(), depots.end());

    for (int intersect_id_start : deliveryPoints)
    {

        std::priority_queue<segIntersectionData, std::vector<segIntersectionData>, segIntersectionDataComparator> openSet;
        std::vector<StreetSegmentIndex> cameFrom(getNumStreetSegments());
        std::vector<double> gScore(adjacencyList.size(), DBL_MAX);
        gScore[intersect_id_start] = 0;

        openSet.emplace(intersect_id_start, -1, 0);
        #ifdef drawAlgos
            ezgl::renderer * g = appl->get_renderer();
        #endif

        while (!openSet.empty())
        {
            segIntersectionData current = openSet.top();

            if (std::count(deliveryPoints.begin(), deliveryPoints.end(), current.intersection))
            {
                int temp = current.segment;
                std::vector<StreetSegmentIndex> path;
                while (temp != -1)
                {
                    path.push_back(temp);
                    temp = cameFrom[temp];
                }
                std::reverse(path.begin(), path.end());
                travelTimes[intersect_id_start][current.intersection] = {path,compute_path_travel_time(path, turn_penalty)};
            }

            openSet.pop();
            if (current.distance <= gScore[current.intersection])
            {
                gScore[current.intersection] = current.distance;

                for (segIntersectionData neighbor : adjacencyList[current.intersection])
                {

                    #ifdef drawAlgos
                        //drawPathStreetSegment(g, segmentData[neighbor.segment], &ezgl::BLUE);
                        //appl->flush_drawing();
                    #endif

                    double tentative_gScore = gScore[current.intersection] + get_seg_cost(current.segment, neighbor.segment, turn_penalty);

                    if (tentative_gScore < gScore[neighbor.intersection])
                    {
                        cameFrom[neighbor.segment] = current.segment;
                        gScore[neighbor.intersection] = tentative_gScore;
                        neighbor.distance = tentative_gScore;
                        openSet.push(neighbor);
                    }
                }
            }
        }
    }

    int minDepot = depots[0];
    double minDepotTravelTime = DBL_MAX;
    int minPackage = 0;
    for (int i = 0; i < depots.size(); i++)
    {
        int minPackageInternal = 0;
        double minPackageTravelTime = DBL_MAX;
        for (int j = 0; j < deliveries.size(); j++)
        {
            DeliveryInfo d = deliveries[j];
            
            double time = travelTimes[depots[i]][d.pickUp].travelTime;
            if (time < minPackageTravelTime)
            {
                minPackageInternal = j;
                minPackageTravelTime = time;
            }
        }
        if (minPackageTravelTime < minDepotTravelTime)
        {
            minDepotTravelTime = minPackageTravelTime;
            minDepot = depots[i];
            minPackage = minPackageInternal;
        }
    }


    std::vector<pickDrop> picksAndDrops; //true  = pickup
    picksAndDrops.push_back({minPackage,deliveries[minPackage].pickUp,true, deliveries[minPackage].itemWeight});
    std::list<int> canPickUp;
    std::vector<int> packagesCompleted;
    std::list<int> canDropOff;
    canDropOff.push_back(minPackage);

    for (size_t i = 0; i < deliveries.size(); i++)
    {
        if (i != minPackage)
            canPickUp.push_back(i);
    }
    float truckWeight = deliveries[minPackage].itemWeight;
    while (packagesCompleted.size() < deliveries.size())
    {
        int nextPackage = 0;
        bool pickOrDrop = true;
        double nextTravelTime = DBL_MAX;
        IntersectionIndex intersect=-1;
        for (int package : canPickUp)
        {
            if (truckWeight + deliveries[package].itemWeight > truck_capacity)
                continue;
            double time = travelTimes[picksAndDrops.back().intersection][deliveries[package].pickUp].travelTime;
            if (time < nextTravelTime)
            {
                nextPackage = package;
                pickOrDrop = true;
                nextTravelTime = time;
                intersect = deliveries[package].pickUp;
            }
        }
        for (int package : canDropOff)
        {

            double time = travelTimes[picksAndDrops.back().intersection][deliveries[package].dropOff].travelTime;
            if (time < nextTravelTime)
            {
                nextPackage = package;
                pickOrDrop = false;
                nextTravelTime = time;
                intersect=deliveries[package].dropOff;
            }
        }
        if (pickOrDrop)
        {
            for (auto i = canPickUp.begin();i!=canPickUp.end(); i++)
            {
                if (*i == nextPackage)
                {
                    canPickUp.erase(i);
                    break;
                }
            }
            canDropOff.push_back(nextPackage);
            truckWeight += deliveries[nextPackage].itemWeight;
        }
        else
        {
            for (auto i = canDropOff.begin(); i != canDropOff.end(); i++)
            {
                if (*i == nextPackage)
                {
                    canDropOff.erase(i);
                    break;
                }
            }
            packagesCompleted.push_back(nextPackage);
            truckWeight -= deliveries[nextPackage].itemWeight;
        }

        picksAndDrops.push_back({nextPackage,intersect, pickOrDrop, deliveries[nextPackage].itemWeight});
    }

    int prev = 0;
    int curr = 1;
    std::vector<pickDrop> newRoute;
    std::vector<pickDrop> bestRoute;
    double newTime;
    double bestTime = computeTravelTime(picksAndDrops);
    while(!timeOut){        
        for(int i = 1; i < picksAndDrops.size()-1; i++){
            for(int k = i+1; k < picksAndDrops.size(); k++){
                newRoute = simpleSwap(picksAndDrops, i, k);
                newTime = computeTravelTime(newRoute);
                if(newTime < bestTime && isLegal(newRoute, truck_capacity)){
                    bestTime = newTime;
                    picksAndDrops = newRoute;
                    curr++;
                }
            }
        }
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);
        
        if(wallClock.count() > 40.5)
            timeOut = true;
        prev++;
    }
    
    int dropOffDepot = 0;
    double depotDist = DBL_MAX;
    for (int depot : depots)
    {

        double time = travelTimes[picksAndDrops.back().intersection][depot].travelTime;
        if (time < depotDist)
        {
            depotDist = time;
            dropOffDepot = depot;
        }
    }



    std::vector<CourierSubpath> result;
    #ifdef drawAlgos
        intersectionsData[minDepot].isHighlighted = true;
        highlightedSegs.clear();
        highlightedWalkingSegs.clear();
        highlightedWalkingSegs = find_path_between_intersections(minDepot,picksAndDrops.front().intersection,turn_penalty);
        for(int i=0;i<picksAndDrops.size()-1;i++){
            intersectionsData[picksAndDrops[i].intersection].isHighlighted=true;
            std::vector<StreetSegmentIndex> path = find_path_between_intersections(picksAndDrops[i].intersection,picksAndDrops[i+1].intersection,turn_penalty);
            highlightedSegs.insert(highlightedSegs.end(),path.begin(),path.end());
        }
        intersectionsData[picksAndDrops.back().intersection].isHighlighted = true;
        intersectionsData[dropOffDepot].isHighlighted = true;
        std::vector<StreetSegmentIndex> path = find_path_between_intersections(picksAndDrops.back().intersection,dropOffDepot,turn_penalty);
        highlightedWalkingSegs.insert(highlightedWalkingSegs.end(),path.begin(),path.end());
        //appl->refresh_drawing();
    #endif
    #ifndef drawAlgos
        CourierSubpath sub = {minDepot,picksAndDrops.front().intersection,find_path_between_intersections(minDepot,picksAndDrops.front().intersection,turn_penalty),std::vector<unsigned>(0)};
        result.push_back(sub);  
        for(int i=0;i<picksAndDrops.size()-1;i++){
            CourierSubpath subpth;
            subpth.start_intersection=picksAndDrops[i].intersection;
            subpth.end_intersection= picksAndDrops[i+1].intersection;
            subpth.subpath=find_path_between_intersections(subpth.start_intersection,subpth.end_intersection,turn_penalty);
            int j=i;
            // while(j<picksAndDrops.size()&&picksAndDrops[j].intersection==subpth.start_intersection&&picksAndDrops[j].pickOrDrop){
            //     std::cout<<subpth.start_intersection<<std::endl;
            //     subpth.pickUp_indices.push_back(picksAndDrops[j].packageIndex);
            //     i=j;
            //     j++;
            // }
            if(picksAndDrops[i].pickOrDrop)
                subpth.pickUp_indices.push_back(picksAndDrops[j].packageIndex);
            result.push_back(subpth);
        }
        sub={picksAndDrops.back().intersection,dropOffDepot,find_path_between_intersections(picksAndDrops.back().intersection,dropOffDepot,turn_penalty),std::vector<unsigned>(0)};
        result.push_back(sub);  

    #endif

    //PETRUBATE
    
    return result;
}

double get_seg_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty)
{
    if (current == -1)
        return find_street_segment_travel_time(next);
    return find_street_segment_travel_time(next) + ((getInfoStreetSegment(current).streetID != getInfoStreetSegment(next).streetID) ? turn_penalty : 0);
}

std::vector<pickDrop> swap(std::vector<pickDrop> couriers, int index1, int index2){
    //Swap the tings
    //check if legal
    //reverse until legal
    //return the mandem

}

std::vector<pickDrop> twoOptSwap(std::vector<pickDrop> deliveryOrder, int first, int second){
    std::vector<pickDrop> newPath;
    
    for(int i = 0; i < first; i++)
        newPath.push_back(deliveryOrder[i]);
    
    for(int i = second; i >=first; i--)
        newPath.push_back(deliveryOrder[i]);
    
    for(int i = second+1; i < deliveryOrder.size(); i++)
        newPath.push_back(deliveryOrder[i]);
    
    return newPath;
}

std::vector<pickDrop> simpleSwap(std::vector<pickDrop> deliveryOrder, int first, int second){
    auto temp = deliveryOrder[first];
    deliveryOrder[first] = deliveryOrder[second];
    deliveryOrder[second] = temp;
    return deliveryOrder;
}

bool isLegal(std::vector<pickDrop> deliveryOrder, const float truck_capacity){
    float curr_truck_wgt = 0;
    std::vector<int> pickedUp;
    std::vector<int> droppedOff;
    int completedPackages = 0;
    
    for(int idx = 0; idx < deliveryOrder.size(); idx++){
        if(deliveryOrder[idx].pickOrDrop){
            pickedUp.push_back(deliveryOrder[idx].packageIndex);
            curr_truck_wgt += deliveryOrder[idx].itemWeight;
            if(curr_truck_wgt > truck_capacity) return false;
        }
        else if(std::count(pickedUp.begin(), pickedUp.end(), deliveryOrder[idx].packageIndex)){
            droppedOff.push_back(deliveryOrder[idx].packageIndex);
            completedPackages++;
            curr_truck_wgt -= deliveryOrder[idx].itemWeight;
        }  
    }
    if(completedPackages == deliveryOrder.size()/2) return true;
    else return false;
}

double computeTravelTime(std::vector<pickDrop> deliveryOrder){
    double travelTime = 0;
    for(int i = 1; i < deliveryOrder.size(); i++){
        travelTime += travelTimes[deliveryOrder[i-1].intersection][deliveryOrder[i].intersection].travelTime;
    }
    return travelTime;
}
