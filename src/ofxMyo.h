#pragma once

#include "ofThread.h"


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