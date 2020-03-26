/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "helpers.h"
#include "globals.h"
#include "ezgl/point.hpp"
#include "ezgl/control.hpp"
#include "ezgl/camera.hpp"
#include "ezgl/canvas.hpp"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m1.h"
#include "m3.h"
#include <cmath>
#include <string>
#include <boost/algorithm/string.hpp>
#include <bits/stdc++.h>
#include <sstream>
#include <boost/format.hpp>

#define RENDER_POIS_STREET 1


std::unordered_map< std::string, ezgl::surface*> iconImgs;
//converts a pair of LatLon points to a pair of Cartesian points
std::pair<Cartesian, Cartesian>  convertLatLonToCartesian(std::pair<LatLon, LatLon> points){
    
    std::pair<Cartesian, Cartesian> convertedPoints;
    double lon1 = points.first.lon()*DEGREE_TO_RADIAN;
    double lon2 = points.second.lon()*DEGREE_TO_RADIAN;
    double lat1 = points.first.lat()*DEGREE_TO_RADIAN;
    double lat2 = points.second.lat()*DEGREE_TO_RADIAN;
    //double lat_avg = (lat1 + lat2)/2;
    convertedPoints.first.xCoord = lon1*cos(lat_avg);
    convertedPoints.second.xCoord = lon2*cos(lat_avg);
    convertedPoints.first.yCoord = lat1;
    convertedPoints.second.yCoord = lat2;
    
    return convertedPoints; 
}

/**
 * lat lon to xy conversion
 * @param point input latlon
 **/
ezgl::point2d LatLonTo2d(LatLon point){
    

    double lon= point.lon()*DEGREE_TO_RADIAN;
    double lat = point.lat()*DEGREE_TO_RADIAN;
    double x = lon*cos(lat_avg);
    double y = lat;
    
    return ezgl::point2d(x,y); 
}

/**
 * x coordinate from longitude
 * @param lon input longitude
 * @return x coordinate based on world projection
 **/
float x_from_lon(float lon){
    return lon*DEGREE_TO_RADIAN*cos(lat_avg);
}

/**
 * x coordinate from latitude
 * @param lat input latitude
 * @return y coordinate based on world projection
 **/
float y_from_lat(float lat){
    return lat*DEGREE_TO_RADIAN;
}


float lon_from_x(float x){
    return x/DEGREE_TO_RADIAN/cos(lat_avg);
}

float lat_from_y(float y){
    return y/DEGREE_TO_RADIAN;
}

/**
 * get bounds of world
 **/
void getBounds(){
    
    float currLon, currLat;
    min_lon = getIntersectionPosition(0).lon();
    max_lon = min_lon;
    min_lat = getIntersectionPosition(0).lat();
    max_lat = min_lat;    
    
    for(int i = 1; i < getNumIntersections(); i++){
        currLon = getIntersectionPosition(i).lon();
        currLat = getIntersectionPosition(i).lat();
        
        if(currLon < min_lon)
            min_lon = currLon;
        
        else if(currLon > max_lon)
            max_lon = currLon;
        
        if(currLat < min_lat)
            min_lat = currLat;
        
        else if(currLat > max_lat)
            max_lat = currLat;
    }

    lat_avg = DEGREE_TO_RADIAN*(min_lat + max_lat)/2;
    
    max_x = x_from_lon(max_lon);
    max_y = y_from_lat(max_lat);
    min_x = x_from_lon(min_lon);
    min_y = y_from_lat(min_lat);
}

bool compareOSMID(OSMID id1, OSMID id2){
 return id1<id2;   
}

//Used to format strings for comparison in the finding street index from partial name function
std::string removeSpaceAndConcat(std::string remove){
    std::string newString = "";
    remove.erase(std::remove_if(remove.begin(), remove.end(), isspace), remove.end());
    for (int x = 0; x < remove.length(); x ++){
        newString.push_back(tolower(remove[x]));
    }
    return newString;
}

std::string removeSpaceAndConcatAndDash(std::string remove){
    std::string newString = removeSpaceAndConcat(remove);
    while(newString.find("-") != std::string::npos){
        newString.erase(newString.find("-"), 1);
    }
    
    return newString;
}

//Used for the sort function to sort a vector of pairs
bool pairCompareStringInt(std::pair<std::string, int> item1, std::pair<std::string, int> item2){
    return item1.first < item2.first;
}

/**
 * parse input entered into search box and extract 2 streets that make up the intersection
 * @returns vector of identified streets
 */
std::vector<std::string> parse2Streets(std::string textInput){
    std::vector<std::string> the2Streets;
    std::string s1, s2, toDrop;
    if (textInput.find(" and ") != std::string::npos){
        the2Streets.push_back(textInput.substr(0, textInput.find(" and ")));
        the2Streets.push_back(textInput.substr(textInput.find(" and ") + 5));
        
    }
    else if(textInput.find("&") != std::string::npos){
        boost::split(the2Streets, textInput, boost::is_any_of("&"));
    }
    else{
        the2Streets.push_back(textInput);
    }
    
    return the2Streets;
}

std::vector<std::string> parse4Streets(std::string textInput){
    std::vector<std::string> the4Streets;
    std::vector<std::string> twoStreetOne;
    std::vector<std::string> twoStreetTwo;
    if (textInput.find(" to ") != std::string::npos){
        twoStreetOne = parse2Streets(textInput.substr(0, textInput.find(" to ")));
        twoStreetTwo = parse2Streets(textInput.substr(textInput.find(" to ") + 4));
        the4Streets.push_back(twoStreetOne[0]);
        the4Streets.push_back(twoStreetOne[1]);
        the4Streets.push_back(twoStreetTwo[0]);
        the4Streets.push_back(twoStreetTwo[1]);
    }
    else{
        the4Streets = parse2Streets(textInput);
    }
    
    return the4Streets;
}


bool sortFeatures(FeatureData first, FeatureData second){
    return (first.area > second.area);
}

/**
 * change application view to look onto specified intersection
 * @param idx index of intersection
 **/
void zoomOnIntersection(ezgl::application *app, int idx){
    float x_2d = x_from_lon(getIntersectionPosition(idx).lon());
    float y_2d = y_from_lat(getIntersectionPosition(idx).lat());
    ezgl::rectangle region({x_2d-diff_x/500, y_2d-diff_y/500}, {x_2d+diff_x/500, y_2d+diff_y/500});
    zoomLevel = 9;
    std::string main_canvas_id = app->get_main_canvas_id();
    auto canvas = app->get_canvas(main_canvas_id);
    ezgl::zoom_fit(canvas, region);
}

/**
 * clear highlightes intersections
 **/
void clearHighlights(){
    if(highlighted.size() == 0)
        return;
    for(int i = 0; i < highlighted.size(); i++){
        intersectionsData[highlighted[i]].isHighlighted = false;
    }
    highlighted.clear();
}

//char * castToCharArray(std::string s){
//    int a = s.length();
//    char charArr[a + 1]; 
//    strcpy(charArr, s.c_str());
//    return;
//}

std::string toLower(std::string s){
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

/**
 * format path from string entered in application search box
 * @param s string entered in search box
 **/
std::string createMapPath(std::string s){
    std::string checker = removeSpaceAndConcatAndDash(s);
    std::cout << checker << std::endl;
    std::string locat, replace;
    for (int x = 0; x < 19; x ++){
        if (removeSpaceAndConcatAndDash(locos[x]).find(checker)!= std::string::npos && removeSpaceAndConcatAndDash(locos[x]).find(checker) == 0){
            if(locos[x].find(",") != std::string::npos){
                replace = locos[x];
                replace.replace(locos[x].find(","), 1, "_");
                locat = replace;
            }
            else{
                locat = locos[x];
            }
            return (mapPathPre + locat + mapPathSuf);
        }
    }
    
    return "DNE";
    
}


void loadImages(ezgl::renderer *g){
    for(auto type: poiTypes)
        iconImgs.emplace(type, g->load_png(("libstreetmap/resources/Icons/"+type+"_icon.png").c_str()));
}

/**
 * parse information received by API
 * @param ss string stream of input
 **/
std::string parseTransitInfo(std::stringstream& ss){
    char header[200];
    char result[300];
    std::string stopName="";
    std::getline(ss, stopName, '|');
    stopName="\nStop Name: "+stopName;
    std::string mode="";
    std::getline(ss, mode, '|');
    mode="Transit Mode:"+ mode;
    std::string name="";
    std::getline(ss, name, '|');
    name = "Route: "+name;
    std::string agency="";
    std::getline(ss, agency, '|');
    agency="Agency: "+agency;
    std::string time="";
    std::getline(ss, time, '|');
    time = "Time: "+time;
    std::cout<<stopName<<" "<<mode<<" "<<name<<" "<<agency<<" "<<time<<" "<<std::endl;
    //std::sprintf(header,"\n|%*s|%*s|%*s|%*s|%*s|",strlen(stopName.c_str()),"Stop Name",strlen(mode.c_str()),"Mode",
   // strlen(name.c_str()),"Route Name",strlen(agency.c_str()),"Transit Agency",strlen(time.c_str()),"Time");
    
    //std::sprintf(result,"|%*s|%*s|%*s|%*s|%*s|",9,stopName.c_str(),4,mode.c_str(),
    //10,name.c_str(),14,agency.c_str(),4,time.c_str());
    //std::cout<<result<<std::endl;
    return (stopName + "\n" + mode + "\n" + name + "\n" + agency + "\n" + time);
    //return std::string(header)+"\n"+std::string(result);
}

/**
 * load popup window with transit data
 * @param transitInfo   information of transit
**/
void infoPopup(ezgl::application *app, std::vector<int> interId, std::string transitInfo){
    GObject *window;
    GtkWidget *content_area;
    GtkWidget *label1;
    GtkWidget *label2;
    GtkWidget *dialog;
    std::string names = "Name(s): ";
    window = app->get_object(app->get_main_window_id().c_str());
    dialog = gtk_dialog_new_with_buttons(
    "Intersection Information",
    (GtkWindow*) window,
    GTK_DIALOG_MODAL,
    ("OK"),
    GTK_RESPONSE_ACCEPT,
    ("CANCEL"),
    GTK_RESPONSE_REJECT,
    NULL);

    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    for (int x = 0; x < interId.size(); x ++){
        if (x > 0){
            names += ", ";
        }
        names += getIntersectionName(interId[x]);
    }
    label1 = gtk_label_new(names.c_str()); //castToCharArray(names)
    gtk_container_add(GTK_CONTAINER(content_area), label1);
    label2 = gtk_label_new(("Upcoming Transit Departures:\n " + transitInfo).c_str());
    gtk_container_add(GTK_CONTAINER(content_area), label2);

    gtk_widget_show_all(dialog);

    g_signal_connect(
    GTK_DIALOG(dialog),
    "response",
    G_CALLBACK(onDialogResponse),
    NULL
    );
}

void onDialogResponse(GtkDialog *dialog, gint response_id, gpointer user_data){
    std::cout << "response is ";
    switch(response_id) {
        case GTK_RESPONSE_ACCEPT:
            std::cout << "GTK_RESPONSE_ACCEPT ";
            break;
        case GTK_RESPONSE_DELETE_EVENT:
            std::cout << "GTK_RESPONSE_DELETE_EVENT (i.e. ’X’ button) ";
            break;
        case GTK_RESPONSE_REJECT:
            std::cout << "GTK_RESPONSE_REJECT ";
            break;
        default:
            std::cout << "UNKNOWN ";
            break;
    }
    
    std::cout << "(" << response_id << ")\n";
    // This will cause the dialog to be destroyed and close
    // without this line the dialog remains open unless the
    // response_id is GTK_RESPONSE_DELETE_EVENT which
    // automatically closes the dialog without the following line.
    gtk_widget_destroy(GTK_WIDGET (dialog));
}

TurnType determineDirection(LatLon O, LatLon from, LatLon to){
    ezgl::point2d pointA = LatLonTo2d(to);
    ezgl::point2d pointB = LatLonTo2d(from);
    ezgl::point2d origin = LatLonTo2d(O);
    pointA.x -= origin.x;
    pointA.y -= origin.y;
    pointB.x -= origin.x;
    pointB.y -= origin.y;

    double result = pointB.x * pointA.y - pointA.x * pointB.y;

    if(result > 0) return TurnType::RIGHT;
    else if(result < 0) return TurnType::LEFT;
    else return TurnType::STRAIGHT_DIFF_STREET;

}
LatLon getFirstCurvePoint(StreetSegmentIndex idx, InfoStreetSegment seg){
    if(getInfoStreetSegment(idx).curvePointCount>0)
        return getStreetSegmentCurvePoint(0, idx);
    return getIntersectionPosition(seg.to);
}
LatLon getLastCurvePoint(StreetSegmentIndex idx, InfoStreetSegment seg){
    if(getInfoStreetSegment(idx).curvePointCount>0)
        return getStreetSegmentCurvePoint(getInfoStreetSegment(idx).curvePointCount-1, idx);
    return getIntersectionPosition(seg.from);
}

TurnType findTurnType(StreetSegmentIndex first, StreetSegmentIndex second){

    InfoStreetSegment seg1 = getInfoStreetSegment(first);
    InfoStreetSegment seg2 = getInfoStreetSegment(second);

    if(seg1.streetID == seg2.streetID)
        return TurnType::STRAIGHT_SAME_STREET;
    else if(seg1.from == seg2.from)
        return determineDirection(getIntersectionPosition(seg1.from), 
            getFirstCurvePoint(first, seg1), getFirstCurvePoint(second, seg2));
    else if(seg1.from == seg2.to)
        return determineDirection(getIntersectionPosition(seg1.from), 
            getFirstCurvePoint(first, seg1), getLastCurvePoint(second, seg2));
    else if(seg1.to == seg2.from)
        return determineDirection(getIntersectionPosition(seg1.to), 
            getLastCurvePoint(first, seg1), getFirstCurvePoint(second, seg2));
    else if(seg1.to == seg2.to)
        return determineDirection(getIntersectionPosition(seg1.to), 
            getLastCurvePoint(first, seg1), getLastCurvePoint(second, seg2));
    else
        return TurnType::RIGHT;    

}

IntersectionIndex findIntersectionOfSegments(StreetSegmentIndex first, StreetSegmentIndex second){
    InfoStreetSegment seg1 = getInfoStreetSegment(first);
    InfoStreetSegment seg2 = getInfoStreetSegment(second);

    if(seg1.from == seg2.from || seg1.from == seg2.to)
        return seg1.from;
    else if(seg1.to == seg2.from || seg1.to == seg2.to)
        return seg1.to;
    else
        return 0;
}

int getTotalPathDistance(std::vector<StreetSegmentIndex> path){
    int totalDist = 0;
    for(int i = 0; i < path.size(); i++){
        totalDist += segLen[path[i]];
    }
    return totalDist;
}

void drawPathStreetSegment(ezgl::renderer * g, StreetSegmentData& segDat, const ezgl::color * color = nullptr){
            g->set_color(ezgl::WHITE);

            double lineWidth = 4;
            int CITY_ROAD_ADJUST = 3;
            int RESIDENTIAL_ADJUST = 6;
            if(segDat.type==StreetType::EXPRESSWAY){
                    lineWidth=((segDat.lanes!=-1)&&zoomLevel>=RENDER_POIS_STREET?segDat.lanes:5)*zoomLevel;
                //g->set_line_width(20);
                g->set_color(ezgl::YELLOW);
            }
            else if(segDat.type == StreetType::CITY_ROAD){
                
                lineWidth=((segDat.lanes!=-1)&&zoomLevel>=RENDER_POIS_STREET?segDat.lanes:4)*(zoomLevel/CITY_ROAD_ADJUST);
            }
            else {//if(segDat.type==StreetType::RESIDENTIAL){
                //if(zoomLevel<6)
                //    return;
                lineWidth=((segDat.lanes!=-1)?segDat.lanes:2)*(zoomLevel-RESIDENTIAL_ADJUST);
            }

           if(color!=nullptr)
                g->set_color(*color);
            g->set_line_width(5);
            g->set_line_cap(ezgl::line_cap::round);
            for (int k = 0; k < segDat.curvePts.size() - 1; k++)
            {   
                //g->draw_line(LatLonTo2d(segDat.curvePts[k]), LatLonTo2d(segDat.curvePts[k + 1]));
                g->draw_line(segDat.convertedCurvePoints[k],segDat.convertedCurvePoints[k+1]);
            }
}

void printDirections(std::vector<StreetSegmentIndex> walkPath /*IntersectionIndex start, IntersectionIndex end/*std::vector<StreetSegmentIndex> drivePath*/){
    
    //std::vector<StreetSegmentIndex> walkPath = find_path_with_walk_to_pick_up(end, start, 15);
    //std::vector<StreetSegmentIndex> drivePath = find_path_between_intersection(parameters);
    
//    std::vector<StreetSegmentIndex> walkPath = {149809, 149810, 146888, 183222, 146882, 146883, 146884, 146885,
//                                                146886, 146887, 31051, 31050, 289, 288, 287, 286, 15178, 15177,
//                                                15176, 97581, 4098, 136817, 97737, 52359, 97740, 107073, 46533, 
//                                                46532, 46531, 181005, 46535, 176570, 18199, 1589, 211978, 85375};
                                                
    double totalPathDistance = getTotalPathDistance(walkPath)/* + getTotalPathDistance(drivePath)*/;
    std::string initDist = getLengthStreet(walkPath, getInfoStreetSegment(walkPath[0]).streetID, 0);
    int walkTime =  int(compute_path_walking_time(walkPath, 1.25 /*km/h*/, 15));
    //int driveTime = int(compute_path_travel_time(drivePath, 15));
    int totalTime = walkTime ;//+ driveTime;
    std::string totalDistMsg, totalTimeMsg;
    std::cout << totalPathDistance << std::endl;
    if(totalPathDistance > 1000)
        totalDistMsg = std::to_string(int(round(totalPathDistance/1000))) + " km";
    else
        totalDistMsg = std::to_string(round(totalPathDistance)) + " m";
    
    if(totalTime > 3600)
        totalTimeMsg = std::to_string(totalTime/3600) + " h. and " +std::to_string(totalTime/60-totalTime/3600*60)+ " min.";
    else if(totalTime > 60){
        totalTimeMsg = std::to_string(totalTime/60) + " min.";
    }
    else
        totalTimeMsg = std::to_string(totalTime) + " sec.";
    
    std::cout << "Your trip is " << totalDistMsg << " long and will take " << totalTimeMsg << std::endl;
    
    std::cout << "Go straight on " << getStreetName(getInfoStreetSegment(walkPath[0]).streetID) << " towards " << getIntersectionName(findIntersectionOfSegments(walkPath[0],walkPath[1]))
                << " for " << initDist << std::endl;
    
    for(int i = 1; i < walkPath.size(); i++){
        TurnType turn = findTurnType(walkPath[i-1], walkPath[i]);
        std::string streetName = getStreetName(getInfoStreetSegment(walkPath[i]).streetID);
        int dist = segLen[walkPath[i]];
        
        if(turn == TurnType::STRAIGHT_SAME_STREET || turn == TurnType::NONE){
           ;
        }
        else if(turn == TurnType::STRAIGHT_DIFF_STREET){
            std::cout << "Go straight on " << streetName << "for " << getLengthStreet(walkPath, getInfoStreetSegment(walkPath[i]).streetID, i) <<std::endl;
        }
        else if(turn == TurnType::LEFT){
            std::cout << "Turn left on " << streetName << " and go straight for " << getLengthStreet(walkPath, getInfoStreetSegment(walkPath[i]).streetID, i) <<std::endl;
        }
        else if(turn == TurnType::RIGHT){
            std::cout << "Turn right on " << streetName << " and go straight for " << getLengthStreet(walkPath, getInfoStreetSegment(walkPath[i]).streetID, i) <<std::endl;
        }
    }
    
    std::cout<< "You have arrived at the pickup destination \n Driving Instructions Follow:" << std::endl;
      
//    for(int i = 1; i < drivePath.size(); i++){
//        TurnType turn = findTurnType(drivePath[i-1], drivePath[i]);
//        std::string streetName = getStreetName(getInfoStreetSegment(drivePath[i]).streetID);
//        int dist = segLen[drivePath[i]];
//        
//        if(turn == TurnType::STRAIGHT_SAME_STREET || turn == TurnType::NONE){
//           ;
//        }
//        else if(turn == TurnType::STRAIGHT_DIFF_STREET){
//            std::cout << "Go straight on " << streetName << "for " << getLengthStreet(drivePath, getInfoStreetSegment(drivePath[i]).streetID, i) <<std::endl;
//        }
//        else if(turn == TurnType::LEFT){
//            std::cout << "Turn left on " << streetName << " and go straight for " << getLengthStreet(drivePath, getInfoStreetSegment(drivePath[i]).streetID, i) <<std::endl;
//        }
//        else if(turn == TurnType::RIGHT){
//            std::cout << "Turn right on " << streetName << " and go straight for " << getLengthStreet(drivePath, getInfoStreetSegment(drivePath[i]).streetID, i) <<std::endl;
//        }
//    }
    std::cout << "You have arrived at your destination!";
}

std::vector<std::string> getDirections(std::vector<StreetSegmentIndex> walkPath /*IntersectionIndex start, IntersectionIndex end/*std::vector<StreetSegmentIndex> drivePath*/){
    
    //std::vector<StreetSegmentIndex> walkPath = find_path_with_walk_to_pick_up(end, start, 15);
    //std::vector<StreetSegmentIndex> drivePath = find_path_between_intersection(parameters);
    
//    std::vector<StreetSegmentIndex> walkPath = {149809, 149810, 146888, 183222, 146882, 146883, 146884, 146885,
//                                                146886, 146887, 31051, 31050, 289, 288, 287, 286, 15178, 15177,
//                                                15176, 97581, 4098, 136817, 97737, 52359, 97740, 107073, 46533, 
//                                                46532, 46531, 181005, 46535, 176570, 18199, 1589, 211978, 85375};
    std::vector<std::string> directions;                                          
    double totalPathDistance = getTotalPathDistance(walkPath)/* + getTotalPathDistance(drivePath)*/;
    std::string initDist = getLengthStreet(walkPath, getInfoStreetSegment(walkPath[0]).streetID, 0);
    int walkTime =  int(compute_path_walking_time(walkPath, 1.25 /*m/s*/, 15));
    //int driveTime = int(compute_path_travel_time(drivePath, 15));
    int totalTime = walkTime ;//+ driveTime;
    std::string totalDistMsg, totalTimeMsg;

    if(totalPathDistance > 1000)
        totalDistMsg = std::to_string(int(round(totalPathDistance/1000))) + " km";
    else
        totalDistMsg = std::to_string(int(round(totalPathDistance))) + " m";
    
    if(totalTime > 3600)
        totalTimeMsg = std::to_string(totalTime/3600) + " h. and " +std::to_string(totalTime/60-totalTime/3600*60)+ " min.";
    else if(totalTime > 60){
        totalTimeMsg = std::to_string(totalTime/60) + " min.";
    }
    else
        totalTimeMsg = std::to_string(totalTime) + " sec.";
    
    directions.push_back("Your trip is " +totalDistMsg+ " long and will take " +totalTimeMsg);
    
    std::string initDir = calcDirection(getInfoStreetSegment(walkPath[0]) , getInfoStreetSegment(walkPath[1]));
    
    directions.push_back("Head " +initDir+ " on "+getStreetName(getInfoStreetSegment(walkPath[0]).streetID)+ " towards " +getIntersectionName(findIntersectionOfSegments(walkPath[0],walkPath[1]))
                +" for " +initDist);
    
    for(int i = 1; i < walkPath.size(); i++){
        TurnType turn = findTurnType(walkPath[i-1], walkPath[i]);
        std::string streetName = getStreetName(getInfoStreetSegment(walkPath[i]).streetID);
        int dist = segLen[walkPath[i]];
        
        if(turn == TurnType::STRAIGHT_SAME_STREET || turn == TurnType::NONE){
           ;
        }
        else if(turn == TurnType::STRAIGHT_DIFF_STREET){
            directions.push_back("Go straight on " + streetName + "for " +getLengthStreet(walkPath, getInfoStreetSegment(walkPath[i]).streetID, i));
        }
        else if(turn == TurnType::LEFT){
            directions.push_back("Turn left on " + streetName + " and go straight for " + getLengthStreet(walkPath, getInfoStreetSegment(walkPath[i]).streetID, i));
        }
        else if(turn == TurnType::RIGHT){
            directions.push_back("Turn right on " + streetName + " and go straight for " +getLengthStreet(walkPath, getInfoStreetSegment(walkPath[i]).streetID, i));
        }
    }
    
    directions.push_back("You have arrived at the pickup destination \n Driving Instructions Follow:");
      
//    for(int i = 1; i < drivePath.size(); i++){
//        TurnType turn = findTurnType(drivePath[i-1], drivePath[i]);
//        std::string streetName = getStreetName(getInfoStreetSegment(drivePath[i]).streetID);
//        int dist = segLen[drivePath[i]];
//        
//        if(turn == TurnType::STRAIGHT_SAME_STREET || turn == TurnType::NONE){
//           ;
//        }
//        else if(turn == TurnType::STRAIGHT_DIFF_STREET){
//            std::cout << "Go straight on " << streetName << "for " << getLengthStreet(drivePath, getInfoStreetSegment(drivePath[i]).streetID, i) <<std::endl;
//        }
//        else if(turn == TurnType::LEFT){
//            std::cout << "Turn left on " << streetName << " and go straight for " << getLengthStreet(drivePath, getInfoStreetSegment(drivePath[i]).streetID, i) <<std::endl;
//        }
//        else if(turn == TurnType::RIGHT){
//            std::cout << "Turn right on " << streetName << " and go straight for " << getLengthStreet(drivePath, getInfoStreetSegment(drivePath[i]).streetID, i) <<std::endl;
//        }
//    }
    directions.push_back("You have arrived at your destination!");
    return directions;
}
std::string getLengthStreet(std::vector<StreetSegmentIndex> path, StreetIndex street_id, int idx){
    double length = 0;
    for(int i = idx; i < path.size(); i++){
        if(getInfoStreetSegment(path[i]).streetID == street_id){
            length += find_street_segment_length(path[i]);
        }
    }
    int intLength = (int) length;
    
    if(intLength > 1000)
        return (std::to_string(intLength/1000) + " km.");
    else
        return (std::to_string(intLength - ((intLength % 10)-((intLength%10>=5)?10:0))) + " m.");
}

std::string calcBearing(LatLon A, LatLon B){
    float X = cos(A.lat()) * sin(abs(A.lon()-B.lon()));
    float Y = cos(A.lat())*sin(B.lat()) - sin(A.lat())*cos(B.lat())*cos(abs(A.lon()-B.lon()));
    
    double bearing = atan2(X,Y)/DEGREE_TO_RADIAN;
    if(bearing < 0)
        return "Negatron";
    if(bearing > 360)
        return "Bigga";
    if((bearing>=337.5 && bearing<=360) || (bearing>=0 && bearing<=22.5))
        return "North";
    else if(bearing>=22.5 && bearing<=67.5)
        return "Northeast";
    else if(bearing<=112.5 && bearing>=67.5)
        return "East";
    else if(bearing>=112.5 && bearing<=157.5)
        return "Southeast";
    else if(bearing<=202.5 && bearing>=157.5)
        return "South";
    else if(bearing>=202.5 && bearing<=247.5)
        return "Southwest";
    else if(bearing>=247.5 && bearing<=292.5)
        return "West";
    else if(bearing>=292.5 && bearing <=337.5)
        return "Northwest";
    else
        return "shieeet";
}

std::string calcDirection(InfoStreetSegment seg1, InfoStreetSegment seg2){
    if(seg1.from == seg2.from)
        return calcBearing(getIntersectionPosition(seg1.from),getFirstCurvePoint(seg2.from, seg2));
    else if(seg1.from == seg2.to)
        return calcBearing(getIntersectionPosition(seg1.from),getLastCurvePoint(seg2.from, seg2));
    else if(seg1.to == seg2.from)
        return calcBearing(getIntersectionPosition(seg1.to),getFirstCurvePoint(seg2.from, seg2));
    else if(seg1.to == seg2.to)
        return calcBearing(getIntersectionPosition(seg1.to),getLastCurvePoint(seg2.from, seg2));
}
