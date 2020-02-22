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
    std::cout<<"size:"<<streetSegData.size()<<std::endl;
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
    
//    g->set_color(ezgl::WHITE);
//    for(size_t i = 0; i < intersectionsData.size(); i++){
//        float x = x_from_lon(intersectionsData[i].position.lon());
//        float y = y_from_lat(intersectionsData[i].position.lat());
//        
//        float width = 0.00001;
//        float height = width;
//        
//        if(intersectionsData[i].isHighlighted)  
//            g->set_color(ezgl::RED);
//        
//        g->fill_rectangle({x,y},{x + width, y + height});
//    }
//
//    for(size_t i = 0; i < streetSegData.size(); i++){
//        
//        g->set_color(ezgl::BLACK);
//        g->set_line_width(0.1);
//        for(size_t j = 0; j < streetSegData[i].size(); j++){
//            if(streetSegData[i][j].curvePts.size() == 0){
//                g->draw_line(LatLonTo2d(getIntersectionPosition(streetSegData[i][j].info.from)),LatLonTo2d(getIntersectionPosition(streetSegData[i][j].info.to)));
//            }
//            else{         
//                for(size_t k = 0; k < streetSegData[i][j].curvePts.size()-1; k++){
//                    g->draw_line(LatLonTo2d(streetSegData[i][j].curvePts[k]), LatLonTo2d(streetSegData[i][j].curvePts[k+1]));
//                }
//            }
//        }
//    }
    
    std::vector<ezgl::point2d> convertedPoints;
    
    for(size_t i = 0; i < featureData.size(); i++){
       
        if(g->get_visible_world().height() > 0.3 && featureData[i].points.size() < 40) break;
        if(g->get_visible_world().height() > 0.07) break;
        
        //std::transform(featureData[i].points.begin(), featureData[i].points.end(), std::back_inserter(convertedPoints), LatLonTo2d);
        
        g->set_color(ezgl::BLUE);
        
        if(featureData[i].type == Lake && i > 500000 && featureData[i].isClosed){
            std::cout << i << std::endl;
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