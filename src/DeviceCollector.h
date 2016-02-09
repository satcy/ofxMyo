#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include "ofMain.h"
#include <myo/myo.hpp>

namespace ofxMyo{
    
class DeviceCollector;

class Device{
friend DeviceCollector;
    
public:
    
    Device();
    
    void reset();
    
    int getId() ;
    void setId( int v);
    
    ofVec3f getAccel();
    void setAccel(ofVec3f v) ;
    
    ofVec3f getGyro();
    void setGyro(ofVec3f v) ;
    
    ofQuaternion getQuaternion() ;
    void setQuaternion(ofQuaternion v) ;
    
    myo::Pose getPose() ;
    void setPose(myo::Pose v) ;
    
    bool getOnArm() ;
    void setOnArm(bool b) ;
    bool getIsUnlocked() ;
    void setIsUnlocked(bool b) ;
    bool getIsConnect();
    void setIsConnect(bool b);
    
    myo::Arm getWhichArm();
    void setWhichArm(myo::Arm v) ;
    
    std::vector<int> getEmgSamples() ;
    void setEmgSamples(std::vector<int> vals);
    
    float getRoll();
    void setRoll(float v);
    
    float getPitch();
    void setPitch(float v);
    
    float getYaw();
    void setYaw(float v) ;
    
    ofVec3f getLinearAccel();
    void setLinearAccel(ofVec3f v) ;
    
    float getGravity();
    void setGravity(float val);
//    float getLastTimef() { return last_timef; }
//    void setLastTimef(float t) { last_timef = t; }
//    float getTimef() { return timef; }
//    void setTimef(float t) { timef = t; }
    
protected:
    myo::Myo * myo;
    int id;
    
    bool onArm;
    myo::Arm whichArm;
    
    bool isUnlocked;
    bool isConnect;
    
    int roll_w, pitch_w, yaw_w;
    float roll, pitch, yaw;
    myo::Pose currentPose;
    
    ofVec3f accel;
    ofVec3f gyro;
    
    ofQuaternion q;
    
    std::vector<int> emgSamples;
    
    float gravity = 0.98;
    
    ofVec3f linear_accel;
//
//    ofMatrix4x4 currentRotationMatrix;
//    float last_timef;
//    float timef;
};


class DeviceCollector : public myo::DeviceListener {
    
friend Device;
public:
    DeviceCollector();
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion);
    void onUnpair(myo::Myo* myo, uint64_t timestamp);
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);
    void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel);
    void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro);
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat);
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection);
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp);
    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion);
    void onDisconnect(myo::Myo* myo, uint64_t timestamp) ;
    void onUnlock(myo::Myo* myo, uint64_t timestamp);
    void onLock(myo::Myo* myo, uint64_t timestamp);
    Device * findDevice(myo::Myo* myo);
    
    
    std::vector<Device*> devices;
};
}
