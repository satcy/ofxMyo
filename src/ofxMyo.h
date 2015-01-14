#pragma once

#include "ofThread.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>
#include "DeviceCollector.h"

class ofxMyo : public ofThread{
private:
    DeviceCollector collector;
public:
    void setup(){
        startThread();
    }
    
    
    void stop(){
        stopThread();
    }
    
    vector<Device*> getDevices(){
        return collector.devices;
    }
    
    void threadedFunction()
    {
        while(isThreadRunning())
        {
            if(lock())
            {
                myo::Hub hub("net.satcy.ofxMyo");
                
                hub.addListener(&collector);
                
                while(1){
                    hub.run(1);
                }
                
                unlock();
                
            }
            else
            {
                ofLogWarning("threadedFunction()") << "Unable to lock mutex.";
            }
        }
    }
};