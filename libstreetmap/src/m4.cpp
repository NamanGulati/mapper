#include "m4.h"
#include <vector>
#include <map>
#include "StreetsDatabaseAPI.h"
#include "globals.h"
#include "m3.h"
#include "m1.h"
#include <queue>

//#define drawAlgos

double get_seg_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty);

std::vector<CourierSubpath> traveling_courier(const std::vector<DeliveryInfo> &deliveries, const std::vector<int> &depots, const float turn_penalty, const float truck_capacity)
{
    std::vector<int> deliveryPoints;
    for (int i = 0; i < deliveries.size(); i++)
    {
        deliveryPoints.push_back(deliveries[i].dropOff);
        deliveryPoints.push_back(deliveries[i].pickUp);
    }
    deliveryPoints.insert(deliveryPoints.end(), depots.begin(), depots.end());
    std::unordered_map<IntersectionIndex, std::unordered_map<IntersectionIndex, double>> travelTimes;

    for (int intersect_id_start : deliveryPoints)
    {

        std::priority_queue<segIntersectionData, std::vector<segIntersectionData>, segIntersectionDataComparator> openSet;
        std::vector<StreetSegmentIndex> cameFrom(getNumStreetSegments());
        std::vector<double> gScore(adjacencyList.size(), DBL_MAX);
        gScore[intersect_id_start] = 0;

        openSet.emplace(intersect_id_start, -1, 0);
        #ifdef drawAlgos
                ezgl::renderer *g = appl->get_renderer();
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
                travelTimes[intersect_id_start][current.intersection] = compute_path_travel_time(path, turn_penalty);
            }

            openSet.pop();
            if (current.distance <= gScore[current.intersection])
            {
                gScore[current.intersection] = current.distance;

                for (segIntersectionData neighbor : adjacencyList[current.intersection])
                {

                    #ifdef drawAlgos
                        drawPathStreetSegment(g, segmentData[neighbor.segment], &ezgl::BLUE);
                        appl->flush_drawing();
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



    std::vector<CourierSubpath> result;
    return result;
}

double get_seg_cost(StreetSegmentIndex current, StreetSegmentIndex next, const double turn_penalty)
{
    if (current == -1)
        return find_street_segment_travel_time(next);
    return find_street_segment_travel_time(next) + ((getInfoStreetSegment(current).streetID != getInfoStreetSegment(next).streetID) ? turn_penalty : 0);
}
