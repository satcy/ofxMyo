#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include <myo/myo.hpp>

class DeviceCollector;

class Device{
    
public:
    friend DeviceCollector;
    
    Device() : onArm(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
        emgSamples.resize(8);
    }
    
    void reset(){
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        onArm = false;
        isUnlocked = false;
        emgSamples.resize(8);
    }
    
    int getId() { return id; }
    
    ofVec3f getAccel(){ return accel; }
    
    ofVec3f getGyro(){ return gyro; }
    
    ofQuaternion getQuaternion() { return q; }
    
    myo::Pose getPose() { return currentPose; }
    
    bool getOnArm() { return onArm; }
    bool getIsUnlocked() { return isUnlocked; }
    bool getIsConnect() { return isConnect; }
    
    myo::Arm getWhichArm(){ return whichArm; }
    
    std::vector<int> getEmgSamples() { return emgSamples; }
    
    float getRoll() { return roll; }
    
    float getPitch() { return pitch; }
    
    float getYaw() { return yaw; }
    
    
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
};


class DeviceCollector : public myo::DeviceListener {
public:
    DeviceCollector()
    {
    }
    
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
    {
        myo->setStreamEmg(myo::Myo::streamEmgEnabled);
        if ( !findDevice(myo) ) {
            Device * device = findDevice(myo);
            device = new Device();
            device->myo = myo;
            device->id = devices.size();
            devices.push_back(device);
            std::cout << "Paired with " << device->id << "." << std::endl;
        }
    }
    
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        Device * device = findDevice(myo);
        if ( device ) {
            device->reset();
        }
    }
    
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
    {
        Device * device = findDevice(myo);
        if ( device ) {
            for (int i = 0; i < 8; i++) {
                device->emgSamples[i] = emg[i];
            }
        }
    }
    
	void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel)
	{
        Device * device = findDevice(myo);
        if ( device ) {
            device->accel.x = accel.x();
            device->accel.y = accel.y();
            device->accel.z = accel.z();
        }
	}
    
    
	void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro)
	{
        
        Device * device = findDevice(myo);
        if ( device ) {
            device->gyro.x = gyro.x();
            device->gyro.y = gyro.y();
            device->gyro.z = gyro.z();
        }
        
	}
    
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        Device * device = findDevice(myo);
        if ( device ) {
            using std::atan2;
            using std::asin;
            using std::sqrt;
            
            // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
            float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                               1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
            float pitch = asin(2.0f * (quat.w() * quat.y() - quat.z() * quat.x()));
            float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                              1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
            
            device->q.set(quat.x(), quat.y(), quat.z(), quat.w());
                          
            device->roll = roll;
            device->pitch = pitch;
            device->yaw = yaw;
            
            
            // Convert the floating point angles in radians to a scale from 0 to 20.
            device->roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
            device->pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
            device->yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
        }
    }
    
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        Device * device = findDevice(myo);
        if ( device ) {
            device->currentPose = pose;
//            if (pose == myo::Pose::fist) {
//                myo->vibrate(myo::Myo::vibrationShort);
//            }
            
        }
    }
    
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        Device * device = findDevice(myo);
        if ( device ) {
            device->onArm = true;
            device->whichArm = arm;
        }
    }
    
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        
        Device * device = findDevice(myo);
        if ( device ) {
            device->onArm = false;
        }
    }
    
    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
        myo->setStreamEmg(myo::Myo::streamEmgEnabled);
        Device * device = findDevice(myo);
        if ( device ) {
            device->isConnect = true;
        }
    }
    
    void onDisconnect(myo::Myo* myo, uint64_t timestamp) {
        Device * device = findDevice(myo);
        if ( device ) {
            device->isConnect = false;
        }
    }
    
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        Device * device = findDevice(myo);
        if ( device ) {
            device->isUnlocked = true;
        }
    }
    
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        Device * device = findDevice(myo);
        if ( device ) {
            device->isUnlocked = false;
        }
    }
    
    
    
    Device * findDevice(myo::Myo* myo){
        for (int i = 0; i < devices.size(); ++i) {
            if (devices[i]->myo == myo) {
                return devices[i];
            }
        }
        return 0;
    }
    
    
    std::vector<Device*> devices;
};
