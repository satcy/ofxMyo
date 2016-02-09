#pragma once

#include "ofThread.h"


#include <myo/myo.hpp>
#include "DeviceCollector.h"

namespace ofxMyo {
class Myo : public ofThread{
private:
    int duration;
    string identifier;
    DeviceCollector collector;
public:
    void setup(int dur = 1, string ident = "net.satcy.ofxMyo") ;
    void stop();
    vector<Device*> getDevices();
    void threadedFunction();
};

}