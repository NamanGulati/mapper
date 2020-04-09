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
#include <chrono>
#include <cmath>

//#define drawAlgos
std::unordered_map<IntersectionIndex, std::unordered_map<IntersectionIndex, PathData>> travelTimes;
int illegal = 0;
int legal = 0;

struct pickDrop{
        int packageIndex=-1;
        IntersectionIndex intersection=-1;
        bool pickOrDrop=true; //true = pickup
        float itemWeight;
};

double get_seg_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty);
double computeTravelTime(std::vector<pickDrop>& deliveryOrder);
bool isLegal(std::vector<pickDrop>& deliveryOrder, const float truck_capacity);
std::vector<pickDrop> twoOptSwap(std::vector<pickDrop> deliveryOrder, int first, int second);
std::pair<double,std::vector<pickDrop>> simpleSwap(std::vector<pickDrop> deliveryOrder, int first, int second, const float truck_capacity);
double multiDestDijkstra(int intersect_id_start, std::vector<int>deliveryPoints, float turn_penalty);
template<typename T,typename Y, typename Z>
void combineVectors(std::vector<pickDrop> & dest, std::pair<T,T> v1, std::pair<Y,Y> v2, std::pair<Z,Z> v3);
std::pair<double,std::vector<pickDrop>> anothaTwoOptSwap(std::vector<pickDrop> & deliveryOrder, int first, int second, float truck_capacity);

std::vector<CourierSubpath> traveling_courier(const std::vector<DeliveryInfo> &deliveries, const std::vector<int> &depots, const float turn_penalty, const float truck_capacity)
{
    if (truck_capacity == 0){
        return std::vector<CourierSubpath>();
    }
    travelTimes.clear();
    auto startTime = std::chrono::high_resolution_clock::now();
    bool timeOut = false;
        
    std::vector<int> deliveryPoints;
    for (int i = 0; i < deliveries.size(); i++)
    {
        deliveryPoints.push_back(deliveries[i].dropOff);
        deliveryPoints.push_back(deliveries[i].pickUp);
        if (deliveries[i].itemWeight > truck_capacity){
            return std::vector<CourierSubpath>();
        }
    }
    deliveryPoints.insert(deliveryPoints.end(), depots.begin(), depots.end());

    

    #pragma omp parallel for
    for (int i =0;i<deliveryPoints.size();i++)
    {
        multiDestDijkstra(deliveryPoints[i],deliveryPoints,turn_penalty);        
    }

    /*int minDepot = depots[0];
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
    }*/

    std::vector<pickDrop> picksAndDrops;
    double pdTime=DBL_MAX;

    #pragma omp parallel for
    for(int minPackage=0;minPackage<deliveries.size();minPackage++){
        std::vector<pickDrop> picksAndDropsLocal; //true  = pickup
        picksAndDropsLocal.push_back({minPackage,deliveries[minPackage].pickUp,true, deliveries[minPackage].itemWeight});
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
                double time = travelTimes[picksAndDropsLocal.back().intersection][deliveries[package].pickUp].travelTime;
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

                double time = travelTimes[picksAndDropsLocal.back().intersection][deliveries[package].dropOff].travelTime;
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

            picksAndDropsLocal.push_back({nextPackage,intersect, pickOrDrop, deliveries[nextPackage].itemWeight});
        }
        double time = computeTravelTime(picksAndDropsLocal);
        #pragma omp critical
        if(time<pdTime){
            pdTime=time;
            picksAndDrops = picksAndDropsLocal;
        }
    }
 
    
    std::vector<pickDrop> bestRoute;
    double bestTime = computeTravelTime(picksAndDrops);
    #pragma omp parallel sections
    {

        #pragma omp section
        {
            int prev = 0;
            int curr = 1;

            while(!timeOut&&prev!=curr){
                auto startTimeInternal = std::chrono::high_resolution_clock::now();
                for(int i = 1; i < picksAndDrops.size()-1; i++){
                    for(int k = i+1; k < picksAndDrops.size(); k++){
                        std::pair<double,std::vector<pickDrop>> res = anothaTwoOptSwap(picksAndDrops,i,k,truck_capacity);
                        #pragma omp critical
                        if(res.first>0&&res.first<bestTime){
                            bestTime = res.first;
                            picksAndDrops = res.second;
                            curr++;
                        }
                    }

                    auto currentTime = std::chrono::high_resolution_clock::now();
                    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);
                    if(wallClock.count() > 45){
                        #pragma omp critical
                        timeOut = true;
                        i=picksAndDrops.size();
                    }
                }

                prev++;
            }
        }
        #pragma omp section
        {
            int prev = 0;
            int curr = 1;
            while(!timeOut&&prev!=curr){
                //#pragma omp parallel for 
                for(int i = 1; i < picksAndDrops.size()-1; i++){
                 //   #pragma omp parallel for 
                    for(int k = i+1; k < picksAndDrops.size(); k++){
                        std::pair<double,std::vector<pickDrop>> res = simpleSwap(picksAndDrops,i,k, truck_capacity);
                        #pragma omp critical
                        if(res.first>0&&res.first<bestTime) {
                            bestTime = res.first;
                            picksAndDrops = res.second;
                            curr++;
                        }
                    }
                    
                    auto currentTime = std::chrono::high_resolution_clock::now();
                    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);
                    if(wallClock.count() > 45){
                        #pragma omp critical
                        timeOut = true;
                        i=picksAndDrops.size();
                    }
                }
                prev++;
            }
        }
    }
    //std::cout << "legal: "<< legal << std::endl;
    //std::cout << "illegal: " << illegal << std::endl;
    
    
    int dropOffDepot = 0;
    double depotDist = DBL_MAX;
    int minDepot=0;
    double minDepotTravelTime = DBL_MAX;
    for (int depot : depots)
    {

        double time = travelTimes[picksAndDrops.back().intersection][depot].travelTime;
        if (time < depotDist)
        {
            depotDist = time;
            dropOffDepot = depot;
        }
        time = travelTimes[depot][picksAndDrops.front().intersection].travelTime;
        if(time<minDepotTravelTime){
            minDepotTravelTime = time;
            minDepot = depot;
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
        CourierSubpath sub = {minDepot,picksAndDrops.front().intersection,travelTimes[minDepot][picksAndDrops.front().intersection].path,std::vector<unsigned>(0)};
        result.push_back(sub);  
        for(int i=0;i<picksAndDrops.size()-1;i++){
            CourierSubpath subpth;
            subpth.start_intersection=picksAndDrops[i].intersection;
            subpth.end_intersection= picksAndDrops[i+1].intersection;
            subpth.subpath=travelTimes[subpth.start_intersection][subpth.end_intersection].path;
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
        sub={picksAndDrops.back().intersection,dropOffDepot,travelTimes[picksAndDrops.back().intersection][dropOffDepot].path};//,///std::vector<unsigned>(0)};
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

std::pair<double,std::vector<pickDrop>> anothaTwoOptSwap(std::vector<pickDrop> & deliveryOrder, int first, int second, float truck_capacity){
    auto startTime = std::chrono::high_resolution_clock::now();
    std::vector<pickDrop> part1 (deliveryOrder.begin(),deliveryOrder.begin()+first+1);
    std::vector<pickDrop> part2 (deliveryOrder.begin()+first+1,deliveryOrder.begin()+second+1);
    std::vector<pickDrop> part3 (deliveryOrder.begin()+second+1,deliveryOrder.end());

   
    //std::vector<std::pair<double,std::vector<pickDrop>>> solutions;
    std::vector<std::pair<std::vector<pickDrop>::iterator,std::vector<pickDrop>::iterator>> iterators;
    iterators.push_back(std::make_pair(part1.begin(),part1.end()));
    iterators.push_back(std::make_pair(part2.begin(),part2.end()));
    iterators.push_back(std::make_pair(part3.begin(),part3.end()));
    std::vector<std::pair<std::vector<pickDrop>::reverse_iterator,std::vector<pickDrop>::reverse_iterator>> rIterators;
    rIterators.push_back(std::make_pair(part1.rbegin(),part1.rend()));
    rIterators.push_back(std::make_pair(part2.rbegin(),part2.rend()));
    rIterators.push_back(std::make_pair(part3.rbegin(),part3.rend()));
    double best_time=DBL_MAX;
    
    std::pair<double,std::vector<pickDrop>> best;
    
    for(int i=0;i<5;i++){
        for(int j=0;j<6;j++){
            if(i==j||abs(i-j)==3)
                continue;
            for(int k=0;k<6;k++){
                if(k==3)
                    continue;
                if(k==i||k==j||abs(i-k)==3||abs(j-k)==3)
                    continue;
                if(i==0&&j==1&&k==2)
                    continue;
                std::vector<pickDrop> result;
                if(i<3){
                    if(j<3){
                        if(k<3)
                            combineVectors(result, iterators[i],iterators[j],iterators[k]);
                        else
                            combineVectors(result,iterators[i],iterators[j],rIterators[k-3]);
                    }
                    else{
                        if(k<3)
                            combineVectors(result,iterators[i],rIterators[j-3],iterators[k]);
                        else
                            combineVectors(result,iterators[i],rIterators[j-3],rIterators[k-3]);
                    }
                }else{
                    if(j<3){
                        if(k<3)
                            combineVectors(result,rIterators[i-3],iterators[j],iterators[k]);
                        else
                            combineVectors(result,rIterators[i-3],iterators[j],rIterators[k-3]);
                    }
                    else{
                        if(k<3)
                            combineVectors(result,rIterators[i-3],rIterators[j-3],iterators[k]);
                        else
                            combineVectors(result,rIterators[i-3],rIterators[j-3],rIterators[k-3]);
                    }
                }
                if(isLegal(result,truck_capacity)){
                    double time = computeTravelTime(result);
                    if(time<best_time){
                        best_time = time;
                        best = std::make_pair(time,result);
                    }
                    //legal++;
                }//else illegal++;
            }
        }
    }
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);
    //std::cout<<wallClock.count()<<std::endl;
    if(best_time == DBL_MAX)
        return std::make_pair(-1, std::vector<pickDrop>(0));
    else{
        //std::cout<<"returning valid, size: "<<best.second.size()<<std::endl;
        return best;
    }
}

template<typename T,typename Y, typename Z>
void combineVectors(std::vector<pickDrop> & dest, std::pair<T,T> v1,
                                    std::pair<Y,Y> v2,
                                    std::pair<Z,Z> v3){
    dest.clear();
    dest.insert(dest.end(),v1.first,v1.second);
    dest.insert(dest.end(),v2.first,v2.second);
    dest.insert(dest.end(),v3.first, v3.second);
}

std::pair<double,std::vector<pickDrop>> simpleSwap(std::vector<pickDrop> deliveryOrder, int first, int second, const float truck_capacity){
    auto temp = deliveryOrder[first];
    deliveryOrder[first] = deliveryOrder[second];
    deliveryOrder[second] = temp;

    double time = computeTravelTime(deliveryOrder);
    
    if(isLegal(deliveryOrder, truck_capacity)) return std::make_pair(time,deliveryOrder);
    else return std::make_pair(-1, std::vector<pickDrop>(0));
}

bool isLegal(std::vector<pickDrop>& deliveryOrder, const float truck_capacity){
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

double computeTravelTime(std::vector<pickDrop>& deliveryOrder){
    double travelTime = 0;
    for(int i = 1; i < deliveryOrder.size(); i++){
        travelTime += travelTimes[deliveryOrder[i-1].intersection][deliveryOrder[i].intersection].travelTime;
    }
    return travelTime;
}

double multiDestDijkstra(int intersect_id_start, std::vector<int>deliveryPoints, float turn_penalty){
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
                #pragma omp critical
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