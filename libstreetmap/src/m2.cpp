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


void draw_main_canvas (ezgl::renderer *g);

float max_lat, min_lat, max_lon, min_lon;
    
void draw_map () {
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
    
    intersectionsData.resize(getNumIntersections());
    
    for(int i = 0; i < getNumIntersections(); i++){
        intersectionsData[i].position = getIntersectionPosition(i);
        intersectionsData[i].name = getIntersectionName(i);
    }
    
    getBounds(min_lon, max_lon, min_lat, max_lat);    
    lat_avg = DEGREE_TO_RADIAN*(min_lat + max_lat)/2;
     
    ezgl::application application(settings);
  
    ezgl::rectangle initial_world({x_from_lon(min_lon),y_from_lat(min_lat)},{x_from_lon(max_lon),y_from_lat(max_lat)});
    
    application.add_canvas("MainCanvas",
                            draw_main_canvas,
                            initial_world);
    
    
    application.run(NULL, NULL, NULL, NULL);
}

void draw_main_canvas (ezgl::renderer *g) {
    
    g->set_color(211,211,211,255);
    g->fill_rectangle(g->get_visible_world());
    g->set_color(ezgl::WHITE);
    for(size_t i = 0; i < intersectionsData.size(); i++){
        float x = x_from_lon(intersectionsData[i].position.lon());
        float y = y_from_lat(intersectionsData[i].position.lat());
        
        float width = 0.000001;
        float height = width;
        
        if(intersectionsData[i].isHighlighted)  
            g->set_color(ezgl::RED);
        
        //g->fill_arc({x,y},width,0,360);
        //g->fill_rectangle({x,y},{x + width, y + height});
    }

    for(size_t i = 0; i < streetSegData.size(); i++){
        
        g->set_color(ezgl::WHITE);
        for(size_t j = 0; j < streetSegData[i].size(); j++){
            StreetSegmentData segDat=streetSegData[i][j];
                if(segDat.type==StreetType::CITY_ROAD)
                    g->set_line_width(3);
                else if(segDat.type==StreetType::EXPRESSWAY){
                    g->set_line_width(7);
                    g->set_color(ezgl::YELLOW);
                }
                else if(segDat.type==StreetType::RESIDENTIAL)
                    g->set_line_width(1);
                else if(segDat.type==StreetType::SMALL_HIGHWAY)
                    g->set_line_width(5);
                else if(segDat.type==StreetType::OTHER)
                    g->set_line_width(2);
                else
                    g->set_line_width(1);
                
            if(segDat.curvePts.size() == 0){
                g->draw_line(LatLonTo2d(getIntersectionPosition(segDat.info.from)),LatLonTo2d(getIntersectionPosition(segDat.info.to)));
            }
            else{         
                for(size_t k = 0; k < segDat.curvePts.size()-1; k++){
                    g->draw_line(LatLonTo2d(segDat.curvePts[k]), LatLonTo2d(segDat.curvePts[k+1]));
                }
            }
        }
    }
    
    
    for(size_t i = 0; i < featureData.size(); i++){
        std::vector<ezgl::point2d> convertedPoints;
        if(g->get_visible_world().height() > 0.3 && featureData[i].points.size() < 40) break;
        if(g->get_visible_world().height() > 0.07) break;
        
        //std::transform(featureData[i].points.begin(), featureData[i].points.end(), std::back_inserter(convertedPoints), LatLonTo2d);
        FeatureType fType = featureData[i].type;
        if(fType == Lake || fType == River || fType == Stream)
            g->set_color(149,217,255,255);
        else if(fType == Greenspace || fType == Island)
            g->set_color(149,235,100,255);
        else if(fType == Park || fType == Golfcourse)
            g->set_color(149,235,100,255);
        else if(fType == Building)
            g->set_color(ezgl::GREY_75);
        else if(fType == Beach)
            g->set_color(230,216,173,255);
            
        
        if(featureData[i].points.size()>1&&featureData[i].isClosed){
            std::transform(featureData[i].points.begin(), featureData[i].points.end(), std::back_inserter(convertedPoints), LatLonTo2d);
            g->fill_poly(convertedPoints);
        }
//        if(featureData[i].isClosed)
//            g->fill_poly(convertedPoints);
//        else{
//            for(size_t j = 0; j < convertedPoints.size()-1; j++)
//                g->draw_line(convertedPoints[j], convertedPoints[j+1]);
//        }
    }
    
    
}

void findFeature(ezgl::application *app, GdkEventButton *event, double x, double y){
    
}