#pragma once

#include "ofThread.h"


#include <myo/myo.hpp>
#include "DeviceCollector.h"

class ofxMyo : public ofThread{
private:
    int duration;
    string identifier;
    DeviceCollector collector;
    
public:
    void setup(int dur = 1, string ident = "net.satcy.ofxMyo"){
        duration = dur;
        identifier = ident;
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
                try {
                    myo::Hub hub(identifier);
                    
                    hub.addListener(&collector);
                    
                    while(1){
                        hub.run(duration);
                    }
                } catch(const std::exception& e) {
                    
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