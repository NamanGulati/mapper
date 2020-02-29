/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "m2.h"
#include "globals.h"
#include "ezgl/control.hpp"
#include "ezgl/camera.hpp"
#include "ezgl/canvas.hpp"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "helpers.h"
#include <iostream>
#include <map>
#include <vector>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <math.h>
#include <set>
#include <unordered_set>
#include <boost/algorithm/string.hpp>
#include "OSMDatabaseAPI.h"
#include "transit.h"
#include <sstream>
#include <gdk/gdk.h>


#define  HIGH_LEVEL_DRAW_LIM  8.05723e-05 //expressway and small highway
#define  SECONDARY_LEVEL_DRAW_LIM  2.9006e-05 //city road 
#define  TERTIARY_LEVEL_DRAW_LIM  4.8719e-07 //residential and other


void draw_main_canvas(ezgl::renderer *g);
void onClick(ezgl::application *app, GdkEventButton *event, double x, double y);
void onSearch(GtkWidget *widget, ezgl::application *application);
void onSetup(ezgl::application *app, bool new_window);
void onLoadMap(GtkWidget *widget, ezgl::application *application);
void drawStreetSegment(ezgl::renderer * g, StreetSegmentData& segDat);
void drawIntersection(ezgl::renderer * g, IntersectionIndex idx);
void drawPOI(ezgl::renderer *g, POIIndex idx);
ezgl::surface* iconSurface;
void drawStreetName(ezgl::renderer *g,StreetSegmentData segDat);
float diff_x, diff_y;

void draw_map()
{
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
    std::cout << "min_x: "<< min_x << "max_x: "<< max_x <<  "min_x: "<< min_y <<  "max_y: " << max_y <<std::endl;
        
    intersectionsData.resize(getNumIntersections());
    
    diff_x = abs(abs(max_x) - abs(min_x));
    diff_y = abs(abs(max_y) - abs(min_y));
    
    for (int i = 0; i < getNumIntersections(); i++)
    {
        intersectionsData[i].position = getIntersectionPosition(i);
        intersectionsData[i].name = getIntersectionName(i);
    }

    ezgl::application application(settings);

    ezgl::rectangle initial_world({min_x, min_y}, {max_x, max_y});

    application.add_canvas("MainCanvas",
                           draw_main_canvas,
                           initial_world);

    application.run(onSetup, onClick, NULL, NULL);
}

void draw_main_canvas(ezgl::renderer *g)
{   
    if(iconSurface == NULL)
        iconSurface = g->load_png("libstreetmap/resources/small_image.png");

    std::cout<<"area: "<<g->get_visible_world().area()<<std::endl;
    g->set_color(211, 211, 211, 255);
    g->fill_rectangle(g->get_visible_world());

    for (size_t i = 0; i < featureData.size(); i++)
    {
        
        if (g->get_visible_world().height() > 0.3 && featureData[i].points.size() < 40)
            break;
        if (g->get_visible_world().height() > 0.07)
            break;

        //std::transform(featureData[i].points.begin(), featureData[i].points.end(), std::back_inserter(convertedPoints), LatLonTo2d);
        FeatureType fType = featureData[i].type;
        if(fType == Stream){
            g->set_color(149, 217, 255, 255);
            for(size_t p; p < featureData[i].convertedPoints.size()-1; p++)
                g->draw_line(featureData[i].convertedPoints[p],featureData[i].convertedPoints[p+1]);
        }
        else if (fType == Lake || fType == River)
            g->set_color(149, 217, 255, 255);
        else if (fType == Greenspace || fType == Island)
            g->set_color(149, 235, 100, 255);
        else if (fType == Park || fType == Golfcourse)
            g->set_color(149, 235, 100, 255);
        else if (fType == Beach)
            g->set_color(230, 216, 173, 255);
        else if (fType == Building && zoomLevel > 4)
            g->set_color(ezgl::GREY_75);
        else
            break;
        
        std::vector<ezgl::point2d> FDconvertedPoints = featureData[i].convertedPoints;
        if (featureData[i].points.size() > 1 && featureData[i].isClosed)
        {
            g->fill_poly(featureData[i].convertedPoints);
        }
//        else if(zoomLevel > 5){
//            for(size_t p; p < featureData[i].convertedPoints.size()-1; p++)
//                g->draw_line(featureData[i].convertedPoints[p],featureData[i].convertedPoints[p+1]);
//        }
    }
    std::cout<<"ZoomLevel: "<<zoomLevel<<std::endl;

    for (size_t i = 0; i < streetSegData.size(); i++)
    {

        
        for (size_t j = 0; j < streetSegData[i].size(); j++)
        {
            
            drawStreetSegment(g,streetSegData[i][j]);
        }

        for(int j = 0; j< streetSegData[i].size();j++){
            StreetSegmentData segDat = streetSegData[i][j];
            bool b = g->get_visible_world().contains((segDat.convertedCurvePoints[0]+segDat.convertedCurvePoints[segDat.convertedCurvePoints.size()-1])*ezgl::point2d(0.5,0.5));
            if(b&&zoomLevel>=8){
                drawStreetName(g,segDat);
                
            }
        }
    }
    

    
    for (int i = 0; i < intersectionsData.size(); i++)
    {
        if (intersectionsData[i].isHighlighted)
            drawIntersection(g, i);
        //g->fill_arc({x,y},width,0,360);
        //g->fill_rectangle({x,y},{x + width, y + height});
    }
    

    if(zoomLevel > 8){
        for(int k = 0; k < getNumPointsOfInterest(); k++){
            //if(pois[k].)
            drawPOI(g, k);
        }
    }
}

void onSetup(ezgl::application *app, bool new_window){
    new_window;
    GObject *searchEntry = app->get_object("SearchEntry");
    g_signal_connect(searchEntry, "activate", G_CALLBACK(onSearch), app);
    
    GObject *loadMap = app->get_object("LoadMap");
    g_signal_connect(loadMap, "clicked", G_CALLBACK(onLoadMap), app);
    
    //////////////////AUTO COMPLETION
    GtkListStore *completeOptions = (GtkListStore*)app->get_object("AutoCompleteList");
    GtkTreeIter iter;

    gtk_list_store_clear(completeOptions);

    for (int x = 0; x < getNumStreets(); x++) {
        gtk_list_store_append(completeOptions, &iter);
        gtk_list_store_set(completeOptions, &iter, 0, (gchar*)castToCharArray(toLower(getStreetName(x))), -1);
    }

//    for (int i = 0; i < getNumPointsOfInterest(); i++) {
//        gtk_list_store_append(NamesList, &iter);
//        gtk_list_store_set(NamesList, &iter, 0, (gchar*)giveCharStar(toLower(getPointOfInterestName(i))), -1);
//    }
}

void onClick(ezgl::application *app, GdkEventButton *event, double x, double y)
{
    if(event->button == 2)
        return;
    LatLon clickPos(lat_from_y(y),lon_from_x(x));
    IntersectionIndex idx = find_closest_intersection(clickPos);
    //if intersections are circles
    //if(getIntersectionPosition())
    std::cout << "x: "<< x << "y: " << y << "intersection: " << idx << std::endl;
    std::cout << "Lon: "<< clickPos.lon() << "Lat: " << clickPos.lat() << "intersectionLon: " << getIntersectionPosition(idx).lon() << std::endl;
    std::stringstream ss (curlData(clickPos));
    std::string stopName="";
    std::getline(ss, stopName, '|');
    std::string mode="";
    std::getline(ss, mode, '|');
    std::string name="";
    std::getline(ss, name, '|');
    std::string agency="";
    std::getline(ss, agency, '|');
    std::string time="";
    std::getline(ss, time, '|');
    std::cout << stopName << " " << mode << " " << name << " " << agency << " " << time << " " << std::endl;

    std::cout << getIntersectionName(idx) << std::endl;
    clearHighlights();
    intersectionsData[idx].isHighlighted = true;
    highlighted.push_back(idx);
    zoomOnIntersection(app, idx);
    
    app->refresh_drawing();
}

void onSearch(GtkWidget *widget, ezgl::application *application){
    // Get the GtkEntry cast of GtkSearchEntry object
    GtkEntry* search_entry = (GtkEntry *) widget;
            //application->get_object("SearchEntry");
    
    //Retrieve the text from the search entry
    const char* text = gtk_entry_get_text(search_entry);
    
    std::vector<int> streetMatches1, streetMatches2;
    std::pair<int, int> foundStreets;
    std::vector<int> foundIntersects;
    
    std::vector<std::string> toSearch = parse2Streets(text); // fix no and or &
    if (toSearch.size() == 1){
        text = "No Intersections";
    }
    else{
        
        streetMatches1 = find_street_ids_from_partial_street_name(toSearch[0]);
        streetMatches2 = find_street_ids_from_partial_street_name(toSearch[1]);
        bool breakLoop = 0;

        if (!streetMatches1.empty() && !streetMatches2.empty()){
            std::cout << "I have entered" << std::endl;
            for (int x = 0; x < streetMatches1.size(); x ++){
                foundStreets.first = streetMatches1[x];
                for (int y = 0; y < streetMatches2.size(); y ++){
                    foundStreets.second = streetMatches2[y];
                    foundIntersects = find_intersections_of_two_streets(foundStreets);
                    if (!foundIntersects.empty()){
                        breakLoop = 1;
                        break;
                    }

                }
                if (breakLoop)
                    break;
            }
        }
    }
    
    if (foundIntersects.empty())
        std::cout << "No intersection found" << std::endl;

    clearHighlights();
    
    for (int x = 0; x < foundIntersects.size(); x ++){
        std::cout << getIntersectionName(foundIntersects[x]) << std::endl;
        intersectionsData[foundIntersects[x]].isHighlighted = true;
        highlighted.push_back(foundIntersects[x]);
    }
    if(!foundIntersects.empty())
        zoomOnIntersection(application, foundIntersects[0]);
    
    // Update the status bar message
    application->update_message(text);
    // Redraw the graphics
    application->refresh_drawing();
}

void onLoadMap(GtkWidget* widget, ezgl::application* application){
    // Get the GtkEntry cast of GtkSearchEntry object
    GtkEntry* search_entry = (GtkEntry *) application->get_object("SearchEntry");
            //application->get_object("SearchEntry");
    
    std::cout << "object got" << '\n';
    
    //Retrieve the text from the search entry
    std::string newMapInput = gtk_entry_get_text(search_entry);
    
    std::string mapPathPrefix = "/cad2/ece297s/public/maps/";
    std::string mapPathSuffix = ".streets.bin";
    std::string newMapPath = mapPathPrefix + newMapInput + mapPathSuffix;
    
    std::cout << newMapPath << '\n';
    
    close_map();
    load_map(newMapPath);
    
    std::cout << "loaded map" << '\n';
    
    intersectionsData.resize(getNumIntersections());
    
    diff_x = abs(abs(max_x) - abs(min_x));
    diff_y = abs(abs(max_y) - abs(min_y));
    
    for (int i = 0; i < getNumIntersections(); i++)
    {
        intersectionsData[i].position = getIntersectionPosition(i);
        intersectionsData[i].name = getIntersectionName(i);
    }
    
    ezgl::rectangle new_world({min_x, min_y},{max_x, max_y});
    zoomLevel = 1;
    clearHighlights();
    application->change_canvas_world_coordinates("MainCanvas", new_world);
    application->refresh_drawing();
    
}

void drawStreetSegment(ezgl::renderer * g, StreetSegmentData& segDat){
            g->set_color(ezgl::WHITE);
            double area = g->get_visible_world().area();
            double lineWidth = 4;
            if(segDat.type==StreetType::EXPRESSWAY){
                    lineWidth=((segDat.lanes!=-1)&&zoomLevel>=7?segDat.lanes*0.75:4)*zoomLevel;
                //g->set_line_width(20);
                g->set_color(ezgl::YELLOW);
            }
            else if(segDat.type == StreetType::CITY_ROAD){
                
                lineWidth=((segDat.lanes!=-1)&&zoomLevel>=7?segDat.lanes*0.75:3)*(zoomLevel/3);
            }
            else {//if(segDat.type==StreetType::RESIDENTIAL){
                if(zoomLevel<7)
                    return;
                lineWidth=((segDat.lanes!=-1)?segDat.lanes*0.75:1)*(zoomLevel-6);
            }
           // segDat.drawHieght=lineWidth;
            g->set_line_width(lineWidth);
            g->set_line_cap(ezgl::line_cap::round);
            for (int k = 0; k < segDat.curvePts.size() - 1; k++)
            {   
                //g->draw_line(LatLonTo2d(segDat.curvePts[k]), LatLonTo2d(segDat.curvePts[k + 1]));
                g->draw_line(segDat.convertedCurvePoints[k],segDat.convertedCurvePoints[k+1]);
            }
}

void drawIntersection(ezgl::renderer * g, IntersectionIndex idx){
    
    float x = x_from_lon(intersectionsData[idx].position.lon());
    float y = y_from_lat(intersectionsData[idx].position.lat());

    float width = 0.000001;
    float height = width;
    
    g->set_color(ezgl::RED);
    //g->fill_rectangle({x,y},{x + width, y + height});
    g->fill_arc(LatLonTo2d(intersectionsData[idx].position), g->get_visible_world().height()*0.01,0,360);
    
    
}

void drawPOI(ezgl::renderer *g, POIIndex idx){
    g->draw_surface(iconSurface, LatLonTo2d(pois[idx].position));
}
void drawStreetName(ezgl::renderer *g,StreetSegmentData segDat){
    std::string name = getStreetName(segDat.info.streetID);
    if(name=="<unknown>")
        return;
    ezgl::point2d p1(0,0);
    ezgl::point2d p2(0,0);
    g->set_color(ezgl::BLACK);
    if(segDat.convertedCurvePoints.size()>3){
        p1=segDat.convertedCurvePoints[segDat.convertedCurvePoints.size()/2];
        p2=segDat.convertedCurvePoints[(segDat.convertedCurvePoints.size()/2)+1];
    }
    else if(segDat.convertedCurvePoints.size()==3){
        p1=segDat.convertedCurvePoints[0];
        p2=segDat.convertedCurvePoints[2];    
    }else
    {
        p1=segDat.convertedCurvePoints[0];
        p2=segDat.convertedCurvePoints[1];
    }
    
    double angle= atan((p2.y-p1.y)/(p2.x-p1.x))/DEGREE_TO_RADIAN;
    ezgl::point2d centerPt= (p1+p2)*ezgl::point2d(0.5,0.5);
    if(angle<-90) angle+=180;
    else if(angle>90) angle-=180;
    g->set_text_rotation(angle);
    ezgl::point2d diff= segDat.convertedCurvePoints.front()-segDat.convertedCurvePoints.back();
    double segmentLen=sqrt(pow(diff.x,2)+pow(diff.y,2));
    std::string oneWayIndicator="";
    if(segDat.info.oneWay){
        if(atan2(diff.x,diff.y)<0)
            g->draw_text(centerPt," → "+name+" → ",segmentLen,10);
        else
            g->draw_text(centerPt," ← "+name+" ← ",segmentLen,10);
    }
    else
        g->draw_text(centerPt,name,segmentLen,10);
}