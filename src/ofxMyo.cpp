#include "ofxMyo.h"

namespace ofxMyo {
    
void Myo::setup(int dur, string ident) {
    duration = dur;
    identifier = ident;
    startThread();
}
    
    
void Myo::stop() {
    stopThread();
}
    
vector<Device*> Myo::getDevices() {
    return collector.devices;
}
    
void Myo::threadedFunction() {
    while(isThreadRunning()) {
        if(lock()) {
            try {
                myo::Hub hub(identifier);
                
                hub.addListener(&collector);
                
                while (1) {
                    hub.run(duration);
                }
            } catch(const std::exception& e) {
                
            }
            unlock();
        } else {
            //ofLogWarning("threadedFunction()") << "Unable to lock mutex.";
        }
    }
}
    
}