/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "m2.h"
#include "globals.h"
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
#include "OSMDatabaseAPI.h"


#define  HIGH_LEVEL_DRAW_LIM  8.05723e-05 //expressway and small highway
#define  SECONDARY_LEVEL_DRAW_LIM  2.9006e-05 //city road 
#define  TERTIARY_LEVEL_DRAW_LIM  4.8719e-07 //residential and other

void draw_main_canvas(ezgl::renderer *g);
void onClick(ezgl::application *app, GdkEventButton *event, double x, double y);
void onSearch(GtkWidget* widget, ezgl::application *application);
void drawStreetSegment(ezgl::renderer * g, StreetSegmentData segDat);
void drawIntersection(ezgl::renderer * g, IntersectionIndex idx);

void draw_map()
{
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";

    intersectionsData.resize(getNumIntersections());

    for (int i = 0; i < getNumIntersections(); i++)
    {
        intersectionsData[i].position = getIntersectionPosition(i);
        intersectionsData[i].name = getIntersectionName(i);
    }

    ezgl::application application(settings);

    ezgl::rectangle initial_world({x_from_lon(min_lon), y_from_lat(min_lat)}, {x_from_lon(max_lon), y_from_lat(max_lat)});

    application.add_canvas("MainCanvas",
                           draw_main_canvas,
                           initial_world);

    application.run(NULL, onClick, NULL, NULL);
}

void draw_main_canvas(ezgl::renderer *g)
{   
    
    std::cout<<"area: "<<g->get_visible_world().area()<<std::endl;
    g->set_color(211, 211, 211, 255);
    g->fill_rectangle(g->get_visible_world());
    for (size_t i = 0; i < intersectionsData.size(); i++)
    {
        if (intersectionsData[i].isHighlighted)
            drawIntersection(g, i);
        //g->fill_arc({x,y},width,0,360);
        //g->fill_rectangle({x,y},{x + width, y + height});
    }

    for (size_t i = 0; i < featureData.size(); i++)
    {
        
        if (g->get_visible_world().height() > 0.3 && featureData[i].points.size() < 40)
            break;
        if (g->get_visible_world().height() > 0.07)
            break;

        //std::transform(featureData[i].points.begin(), featureData[i].points.end(), std::back_inserter(convertedPoints), LatLonTo2d);
        FeatureType fType = featureData[i].type;
        if (fType == Lake || fType == River || fType == Stream)
            g->set_color(149, 217, 255, 255);
        else if (fType == Greenspace || fType == Island)
            g->set_color(149, 235, 100, 255);
        else if (fType == Park || fType == Golfcourse)
            g->set_color(149, 235, 100, 255);
        else if (fType == Building)
            g->set_color(ezgl::GREY_75);
        else if (fType == Beach)
            g->set_color(230, 216, 173, 255);

        std::vector<ezgl::point2d> FDconvertedPoints = featureData[i].convertedPoints;
        if (featureData[i].points.size() > 1 && featureData[i].isClosed)
        {
            g->fill_poly(featureData[i].convertedPoints);
        }
        //        if(featureData[i].isClosed)
        //            g->fill_poly(convertedPoints);
        //        else{
        //            for(size_t j = 0; j < convertedPoints.size()-1; j++)
        //                g->draw_line(convertedPoints[j], convertedPoints[j+1]);
        //        }
    }
    std::cout<<"ZoomLevel: "<<zoomLevel<<std::endl;

    for (size_t i = 0; i < streetSegData.size(); i++)
    {

        
        for (size_t j = 0; j < streetSegData[i].size(); j++)
        {
            StreetSegmentData segDat = streetSegData[i][j];
            //if(g->get_visible_world().contains(LatLonTo2d(segDat.curvePts[0]))||g->get_visible_world().contains(LatLonTo2d(segDat.curvePts[segDat.curvePts.size()-1]))){
                drawStreetSegment(g,segDat);
            //}
        }
    }

    
}

void onClick(ezgl::application *app, GdkEventButton *event, double x, double y)
{
    LatLon clickPos(lat_from_y(y),lon_from_x(x));
    IntersectionIndex intersection = find_closest_intersection(clickPos);
    //if intersections are circles
    //if(getIntersectionPosition())
    std::cout << "x: "<< x << "y: " << y << "intersection: " << intersection<< std::endl;
    std::cout << "Lon: "<< clickPos.lon() << "Lat: " << clickPos.lat() << "intersectionLon: " << getIntersectionPosition(intersection).lon() << std::endl;
    std::cout << getIntersectionName(intersection) << std::endl;
    intersectionsData[intersection].isHighlighted = true;
    app->refresh_drawing();
}

void onSearch(ezgl::application *application){
    // Get the GtkEntry cast of GtkSearchEntry object
    GtkEntry* search_entry = (GtkEntry *) application->get_object("SearchEntry");
    
    //Retrieve the text from the search entry
    const char* text = gtk_entry_get_text(search_entry);
    
    // Update the status bar message
    application->update_message("Searched");
    // Redraw the graphics
    application->refresh_drawing();
}

void drawStreetSegment(ezgl::renderer * g, StreetSegmentData segDat){
            g->set_color(ezgl::WHITE);
            double area = g->get_visible_world().area();
            int lineWidth = 4;
            if(segDat.type==StreetType::EXPRESSWAY){
               lineWidth=(segDat.lanes!=-1?segDat.lanes:3)*zoomLevel;
                //g->set_line_width(20);
                g->set_color(ezgl::YELLOW);
            }
            else if(segDat.type == StreetType::CITY_ROAD){
                if(zoomLevel<3)
                    return;
                
                lineWidth=(segDat.lanes!=-1?segDat.lanes:2)*(zoomLevel-2);
            }
            else {//if(segDat.type==StreetType::RESIDENTIAL){
                if(zoomLevel<7)
                    return;
                lineWidth=(segDat.lanes!=-1?segDat.lanes:1)*(zoomLevel-6);
            }



            // if (segDat.type == StreetType::CITY_ROAD && area>SECONDARY_LEVEL_DRAW_LIM)
            //      return;
            // else if (segDat.type == StreetType::EXPRESSWAY)
            // {
                 
            // }
            // else if (segDat.type == StreetType::RESIDENTIAL&& area>TERTIARY_LEVEL_DRAW_LIM)
            //      return;
            // else if (segDat.type == StreetType::OTHER && area > TERTIARY_LEVEL_DRAW_LIM)
            //     return;
            // else if(segDat.lanes!=-1)
            //     lineWidth = segDat.lanes*2*zoomlevel;//g->set_line_width((segDat.lanes)*3);

            g->set_line_width(lineWidth);
            g->set_line_cap(ezgl::line_cap::round);
            for (size_t k = 0; k < segDat.curvePts.size() - 1; k++)
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