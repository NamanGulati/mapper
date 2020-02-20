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

void draw_map () {
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
    
    float max_lat, min_lat, max_lon, min_lon;
    
    getBounds(min_lon, max_lon, min_lat, max_lat);    
    lat_avg = (min_lat + max_lat)/2;
     
    ezgl::application application(settings);
    
    ezgl::rectangle  initial_world({min_lon,min_lat},
                                   {max_lon,max_lat});
    
    application.add_canvas("MainCanvas",
                            draw_main_canvas,
                            initial_world);
    
    application.run(NULL, NULL, NULL, NULL);
}

void draw_main_canvas (ezgl::renderer *g) {
    
    for(int i = 0; i < intersections.size(); i++){
        float x = x_from_lon(intersections[i].position.lon());
        float y = y_from_lat(intersections[i].position.at());
        
        float width = 0.001;
        float height = width;
        
        g->fill_rectangle({x,y},{x + width, y + height});
    }
}