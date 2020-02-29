#include "transit.h"
#include <curl/curl.h>
#include <iostream>
#include "LatLon.h"
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <cstdio>

std::string exec(const char* cmd) ;
std::string curlData(LatLon pos){
    /*CURLcode res = curl_global_init(CURL_GLOBAL_ALL);
    if (res != CURLE_OK){
        std::cout << "ERROR: Unable to initialize libcurl" << std::endl;
        std::cout << curl_easy_strerror(res) << std::endl;
        return 1;
    }
    CURL *curlHandle = curl_easy_init();
    if(!curlHandle){
        std::cout<<"ERROR: unable to get handle"<<std::endl;
    }
    else{
        //curl_easy_setopt(curlHandle,CURLOPT_GET,true);
       std::cout<<"http://ec2-3-82-130-183.compute-1.amazonaws.com/upcoming?lat="+std::to_string(pos.lat())+"&lon="+std::to_string(pos.lon())<<std::endl;
       //res = curl_easy_setopt(curlHandle,CURLOPT_URL,"http://ec2-3-82-130-183.compute-1.amazonaws.com/upcoming");
       const char* str = curl_easy_escape(curlHandle,("http://ec2-3-82-130-183.compute-1.amazonaws.com/upcoming?lat="+std::to_string(pos.lat())+"&lon="+std::to_string(pos.lon())).c_str(),0);
       //res = curl_easy_setopt(curlHandle,CURLOPT_POSTFIELDS,"lat="+std::to_string(pos.lat())+"&lon="+std::to_string(pos.lon()));
       //res = curl_easy_setopt(curlHandle,CURLOPT_POSTFIELDS,str);
       curl_easy_setopt(curlHandle,CURLOPT_URL,str);
       res = curl_easy_setopt(curlHandle, CURLOPT_CUSTOMREQUEST, "GET");
       curl_easy_setopt(curlHandle,CURLOPT_VERBOSE,true);
      

       
    }

    if (res != CURLE_OK) {
        std::cout << "ERROR: Unable to set libcurl option" << std::endl;
        std::cout << curl_easy_strerror(res) << std::endl;
    } else {// Perform web transfer request
        res = curl_easy_perform(curlHandle);
    }
    std::cout << std::endl << std::endl;

    if (res == CURLE_OK) {
        std::cout << "All good! res == CURLE_OK!" << std::endl;
    } else {
        std::cout << "ERROR: res == " << res << std::endl;
        std::cout << curl_easy_strerror(res) << std::endl;
        std::cout << "See https://curl.haxx.se/libcurl/c/libcurl-errors.html for error codes" << std::endl;
    }
    curl_easy_cleanup(curlHandle);
    curlHandle = nullptr;
    return 0;*/
    return exec(("curl \"http://ec2-3-82-130-183.compute-1.amazonaws.com/upcoming?lat="+std::to_string(pos.lat())+"&lon="+std::to_string(pos.lon())+"\"").c_str());
}
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}
