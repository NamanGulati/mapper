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
#include "m3.h"
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
#define middle_mouse_button 2
#define RENDER_POIS_STREET 7
#define RENDER_STREET_NAME 5
#define HIGH_ZOOM 9
#define BUILDING_ZOOM 5

void draw_main_canvas(ezgl::renderer *g);
void onClick(ezgl::application *app, GdkEventButton *event, double x, double y);
void onSearch(GtkWidget *widget, ezgl::application *application);
void onSetup(ezgl::application *app, bool new_window);
void drawStreetSegment(ezgl::renderer * g, StreetSegmentData& segDat, const ezgl::color * color);
void drawSegments(ezgl::renderer *g);
void drawPOIs(ezgl::renderer *g);
void drawIntersections(ezgl::renderer *g);
void drawIntersection(ezgl::renderer * g, IntersectionIndex idx);
void drawPOI(ezgl::renderer *g, POIData p);
void getDiff(float &diffX, float &diffY);
void drawFeatures(ezgl::renderer *g); 
ezgl::surface* iconSurface;
bool drawStreetName(ezgl::renderer *g,StreetSegmentData segDat);
void drawPOIText(ezgl::renderer * g,POIIndex idx);
float diff_x, diff_y;
void loadPNGs(ezgl::renderer *g);

ezgl::application * appl;
bool previouslyHighlighted =false;
int lastIntersection = -1;

void draw_map()
{
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
        
    intersectionsData.resize(getNumIntersections());
    
    getDiff(diff_x, diff_y);
    
    for (int i = 0; i < getNumIntersections(); i++)
    {
        intersectionsData[i].position = getIntersectionPosition(i);
        intersectionsData[i].name = getIntersectionName(i);
    }

    ezgl::application * application = new ezgl::application(settings);

    ezgl::rectangle initial_world({min_x, min_y}, {max_x, max_y});

    application->add_canvas("MainCanvas",
                           draw_main_canvas,
                           initial_world);

    appl=application;

    application->run(onSetup, onClick, NULL, NULL);

    delete appl;
}

/**
 * main rendering code that renders street segments, intersections, featues and POIS on map refresh
 **/
void draw_main_canvas(ezgl::renderer *g)
{   
    //loads the pngs for icons
    loadPNGs(g);
    //sets background colour
    g->set_color(211, 211, 211, 255);
    g->fill_rectangle(g->get_visible_world());
    
    drawFeatures(g);
    drawSegments(g);
    drawIntersections(g);
    drawPOIs(g);
    printDirections();
}

/**
 * callback executed on application setup
 **/
void onSetup(ezgl::application *app, bool new_window){
    new_window;
    GObject *searchEntry = app->get_object("SearchEntry");
    g_signal_connect(searchEntry, "activate", G_CALLBACK(onSearch), app);
    

    
    //////////////////AUTO COMPLETION
    GtkListStore *completeOptions = (GtkListStore*)app->get_object("AutoCompleteList");
    GtkTreeIter iter;

    gtk_list_store_clear(completeOptions);

    for (int x = 0; x < getNumStreets(); x++) {
        gtk_list_store_append(completeOptions, &iter);
        gtk_list_store_set(completeOptions, &iter, 0, (gchar*)(toLower(getStreetName(x)).c_str()), -1);
    }  
}

/**
 * callback executed when user clicks on the map- looks for closest intersection to user's click
 * and gets upcoming transit departures
 * @param x x-coordinate of user click
 * @param y y-coordinate of user click
 **/
void onClick(ezgl::application *app, GdkEventButton *event, double x, double y)
{   
    std::cout<<"BUtton: "<<event->button<<std::endl;
    if(event->button == middle_mouse_button)
        return;
    if(event->button == 1){
        LatLon clickPos(lat_from_y(y),lon_from_x(x));
        IntersectionIndex idx = find_closest_intersection(clickPos);
        intersectionsData[idx].isHighlighted = true;
        highlighted.push_back(idx);
        if(previouslyHighlighted){
            std::vector<StreetSegmentIndex> path = find_path_between_intersections(lastIntersection,idx,15);
            for(int seg : path){
                //std::cout<<seg<<std::endl;
                drawPathStreetSegment(app->get_renderer(),segmentData[seg],&ezgl::RED);
            }
            app->flush_drawing();
            previouslyHighlighted=false;
            highlighted.clear();
        }else{
            lastIntersection=idx;
            previouslyHighlighted=true;
        }
        return;
    }
    LatLon clickPos(lat_from_y(y),lon_from_x(x));
    IntersectionIndex idx = find_closest_intersection(clickPos);
    //if intersections are circles
    //if(getIntersectionPosition())
    std::stringstream ss (curlData(clickPos));
    std::string transitInfo = parseTransitInfo(ss);
    std::cout << transitInfo << std::endl;

    std::cout << getIntersectionName(idx) << std::endl;
    clearHighlights();
    intersectionsData[idx].isHighlighted = true;
    highlighted.push_back(idx);
    zoomOnIntersection(app, idx);
    std::vector<int> inter{idx};
    infoPopup(app, inter, transitInfo);
    
    app->refresh_drawing();
}

/**
 * callback when user presses enter in search bar. looks and processes intersections or map names
 **/
void onSearch(GtkWidget *widget, ezgl::application *application){
    // Get the GtkEntry cast of GtkSearchEntry object
    GtkEntry* search_entry = (GtkEntry *) widget;
    
    //Retrieve the text from the search entry
    const char* text = gtk_entry_get_text(search_entry);
    
    std::vector<int> streetMatches1, streetMatches2;
    std::pair<int, int> foundStreets;
    std::vector<int> foundIntersects;
    std::vector<std::string> toSearch = parse2Streets(text);
    
    if (toSearch.size() == 1){
        std::cout << "No Intersections" << std::endl;
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
    
    clearHighlights();
    
    for (int x = 0; x < foundIntersects.size(); x ++){
        std::cout << getIntersectionName(foundIntersects[x]) << std::endl;
        intersectionsData[foundIntersects[x]].isHighlighted = true;
        highlighted.push_back(foundIntersects[x]);
    }
    if(!foundIntersects.empty()){
        LatLon foundPos(getIntersectionPosition(foundIntersects[0]).lat(),getIntersectionPosition(foundIntersects[0]).lon());
        std::stringstream ss (curlData(foundPos));
        std::string transit = parseTransitInfo(ss);
        

        zoomOnIntersection(application, foundIntersects[0]);
        
        infoPopup(application, foundIntersects, transit);
        
        return;
    }    
    else{
        if (toSearch.size() == 2){ //two elements in to search
            // Update the status bar message
            application->update_message("No intersections");
            // Redraw the graphics
            application->refresh_drawing();
            return;
        }
    }
    
    
    
    ///////////////////////////////////////////////////////////Loading Map
    
    std::string path = text;
    std::string newMapPath = createMapPath(path);
    
    std::cout << newMapPath << '\n';
    
    if (newMapPath == "DNE"){
        // Update the status bar message
        application->update_message("Not a valid map");
        // Redraw the graphics
        application->refresh_drawing();
        return;
    }
        
    close_map();
    load_map(newMapPath);
    
    std::cout << "loaded map" << '\n';
    
    intersectionsData.resize(getNumIntersections());
    
    getDiff(diff_x, diff_y);
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

/**
 * draw street segment onto screen by iterating through curve points and assigning
 * different colours and widts to different road types
 * @param g ezgl renderer
 * @param segDat data of street segment to render
 **/
void drawStreetSegment(ezgl::renderer * g, StreetSegmentData& segDat, const ezgl::color * color = nullptr){
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
                if(zoomLevel<6)
                    return;
                lineWidth=((segDat.lanes!=-1)?segDat.lanes:2)*(zoomLevel-RESIDENTIAL_ADJUST);
            }
           // segDat.drawHieght=lineWidth;
           if(color!=nullptr)
                g->set_color(*color);
            g->set_line_width(lineWidth);
            g->set_line_cap(ezgl::line_cap::round);
            for (int k = 0; k < segDat.curvePts.size() - 1; k++)
            {   
                //g->draw_line(LatLonTo2d(segDat.curvePts[k]), LatLonTo2d(segDat.curvePts[k + 1]));
                g->draw_line(segDat.convertedCurvePoints[k],segDat.convertedCurvePoints[k+1]);
            }
}

/**
 * draw intersection onto screen
 * @param g ezgl renderer
 * @param idx intersection id of intersection to render
 **/
void drawIntersection(ezgl::renderer * g, IntersectionIndex idx){

    g->set_color(ezgl::RED);
    //g->fill_rectangle({x,y},{x + width, y + height});
    g->fill_arc(LatLonTo2d(intersectionsData[idx].position), g->get_visible_world().height()*0.01,0,360);
    
    
}

/**
 * Renderes POI onto screen
 * 
 * @param g ezgl renderer
 * @param p data of POI to render
 **/
void drawPOI(ezgl::renderer *g, POIData p){
    
    auto find = iconImgs.find(p.type);
    //ezgl::surface* iconSurface;
   
    if(find == iconImgs.end())
        return;
    else
        iconSurface = find->second;
    
    cairo_surface_set_device_scale(iconSurface, 5, 5);
    
    g->draw_surface(iconSurface, LatLonTo2d(p.position));
}

/**
 * Renders Street Name text onto screen
 * 
 * @param g ezgl renderer
 * @param segDat data of street segment to render
 * @return boolean if the segment was actually drawn to screen (i.e. false if off screen)
 */
bool drawStreetName(ezgl::renderer *g,StreetSegmentData segDat){

    std::string name = getStreetName(segDat.info.streetID);
    if(name=="<unknown>")
        return false;
    ezgl::point2d p1(0,0);
    ezgl::point2d p2(0,0);
    g->set_color(ezgl::BLACK);
    if(segDat.convertedCurvePoints.size()>3){
        p1=segDat.convertedCurvePoints[(segDat.convertedCurvePoints.size()/2)];
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
    bool done=false;
    if(segDat.info.oneWay){
        if(atan2(diff.x,diff.y)<0)
            done = g->draw_text(centerPt," → "+name+" → ",segmentLen,10);
        else
            done = g->draw_text(centerPt," ← "+name+" ← ",segmentLen,10);
    }
    else
        done = g->draw_text(centerPt,name,segmentLen,10);
   
    return done;
}

void getDiff(float &diffX, float &diffY){
    
    if(min_x*max_x > 0 && min_y*max_y > 0){
        diffX = abs(abs(max_x) - abs(min_x));
        diffY = abs(abs(max_y) - abs(min_y));
        return;
    }
    if(min_x < 0 && max_x > 0)
        diffX = max_x - min_x;
    if(min_y < 0 && max_y > 0)
        diffY = max_y - min_y;
}

/**
 * @param g ezgl renderer
 * @param idx index of poi to render
 **/
void drawPOIText(ezgl::renderer * g,POIIndex idx){
    g->set_color(ezgl::BLACK);
    g->set_text_rotation(0);
    g->draw_text(LatLonTo2d(pois[idx].position),pois[idx].name);
}

void drawFeatures(ezgl::renderer *g){
    float MIN_RENDER = 0.07;
    float MAX_RENDER = 0.3;
    int num_points = 40;
    for (size_t i = 0; i < featureData.size(); i++)
    {
        
        if (g->get_visible_world().height() > MAX_RENDER && featureData[i].points.size() < num_points)
            break;
        if (g->get_visible_world().height() > MIN_RENDER)
            break;

        //std::transform(featureData[i].points.begin(), featureData[i].points.end(), std::back_inserter(convertedPoints), LatLonTo2d);
        FeatureType fType = featureData[i].type;
        if(fType == Stream)
            g->set_color(149, 217, 255, 255);
        else if (fType == Lake || fType == River)
            g->set_color(149, 217, 255, 255);
        else if (fType == Greenspace || fType == Island)
            g->set_color(149, 235, 100, 255);
        else if (fType == Park || fType == Golfcourse)
            g->set_color(149, 235, 100, 255);
        else if (fType == Beach)
            g->set_color(230, 216, 173, 255);
        else if (fType == Building && zoomLevel > BUILDING_ZOOM)
            g->set_color(ezgl::GREY_75);
        else
            break;
        
        std::vector<ezgl::point2d> FDconvertedPoints = featureData[i].convertedPoints;
        if (featureData[i].points.size() > 1 && featureData[i].isClosed)
        {
            g->fill_poly(featureData[i].convertedPoints);
        }else if(featureData[i].points.size() > 1 && zoomLevel > 8){
            g->set_line_width(1);
            for(size_t p = 0; p < featureData[i].convertedPoints.size()-1; p++)
                g->draw_line(featureData[i].convertedPoints[p],featureData[i].convertedPoints[p+1]);
        }
    }
}

void drawSegments(ezgl::renderer *g){
    for (size_t i = 0; i < streetSegData.size(); i++)
    {

        
        for (size_t j = 0; j < streetSegData[i].size(); j++)
        {
            
            drawStreetSegment(g,streetSegData[i][j]);
        }

        bool done =false;
        for(int j = 0; (j< streetSegData[i].size())&&!done;j++){
            StreetSegmentData segDat = streetSegData[i][j];
            bool b = g->get_visible_world().contains((segDat.convertedCurvePoints[0]+segDat.convertedCurvePoints[segDat.convertedCurvePoints.size()-1])*ezgl::point2d(0.5,0.5));
            if(b&&zoomLevel>RENDER_STREET_NAME){
                done=drawStreetName(g,segDat);
            }
        }
    }
    for(int i=0;i<highlightedSegs.size();i++){
        std::cout<<highlightedSegs[i]<<std::endl;
    }
}

void drawIntersections(ezgl::renderer *g){
    for (int i = 0; i < intersectionsData.size(); i++)
    {
        if (intersectionsData[i].isHighlighted)
            drawIntersection(g, i);
    }
}

void drawPOIs(ezgl::renderer *g){
    if(zoomLevel > RENDER_POIS_STREET){
        for(int k = 0; k < pois.size(); k++){
            if(zoomLevel > HIGH_ZOOM)
                drawPOI(g, pois[k]);
            drawPOIText(g,k);
        }
    }
}

void loadPNGs(ezgl::renderer *g){
//    if(iconImgs.empty())
//        for(auto type: poiTypes)
//            iconImgs.emplace(type, g->load_png(("libstreetmap/resources/"+type+".png").c_str()));
}