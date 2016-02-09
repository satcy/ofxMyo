#define _USE_MATH_DEFINES
#include "DeviceCollector.h"

namespace ofxMyo {

    
#pragma mark - Device

Device::Device() : onArm(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose() {
    emgSamples.resize(8);
}

void Device::reset(){
    roll_w = 0;
    pitch_w = 0;
    yaw_w = 0;
    onArm = false;
    isUnlocked = false;
    emgSamples.resize(8);
}

int Device::getId() { return id; }
void Device::setId( int v) { id = v; }

ofVec3f Device::getAccel(){ return accel; }
void Device::setAccel(ofVec3f v) { accel.set(v); }

ofVec3f Device::getGyro(){ return gyro; }
void Device::setGyro(ofVec3f v) { gyro.set(v); }

ofQuaternion Device::getQuaternion() { return q; }
void Device::setQuaternion(ofQuaternion v) { q.set(v); }

myo::Pose Device::getPose() { return currentPose; }
void Device::setPose(myo::Pose v) { currentPose = v; }

bool Device::getOnArm() { return onArm; }
void Device::setOnArm(bool b) { onArm = b; }
bool Device::getIsUnlocked() { return isUnlocked; }
void Device::setIsUnlocked(bool b) { isUnlocked = b; }
bool Device::getIsConnect() { return isConnect; }
void Device::setIsConnect(bool b) { isConnect = b; }

myo::Arm Device::getWhichArm(){ return whichArm; }
void Device::setWhichArm(myo::Arm v) { whichArm = v; }

std::vector<int> Device::getEmgSamples() { return emgSamples; }
void Device::setEmgSamples(std::vector<int> vals) {
    if ( emgSamples.size() == vals.size() ) {
        for ( int i=0; i<emgSamples.size(); i++ ) {
            emgSamples[i] = vals[i];
        }
    }
}

float Device::getRoll() { return roll; }
void Device::setRoll(float v) { roll = v; }

float Device::getPitch() { return pitch; }
void Device::setPitch(float v) { pitch = v; }

float Device::getYaw() { return yaw; }
void Device::setYaw(float v) { yaw = v; }

ofVec3f Device::getLinearAccel(){ return linear_accel; }
void Device::setLinearAccel(ofVec3f v) { linear_accel.set(v); }

float Device::getGravity() { return gravity; }
void Device::setGravity(float val) { gravity = val; }

#pragma mark - DeviceCollector

DeviceCollector::DeviceCollector(){
}

void DeviceCollector::onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
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

void DeviceCollector::onUnpair(myo::Myo* myo, uint64_t timestamp)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->reset();
    }
}

void DeviceCollector::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
{
    Device * device = findDevice(myo);
    if ( device ) {
        for (int i = 0; i < 8; i++) {
            device->emgSamples[i] = emg[i];
        }
    }
}

void DeviceCollector::onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->accel.x = accel.x();
        device->accel.y = accel.y();
        device->accel.z = accel.z();
    }
}


void DeviceCollector::onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro)
{
    
    Device * device = findDevice(myo);
    if ( device ) {
        device->gyro.x = gyro.x();
        device->gyro.y = gyro.y();
        device->gyro.z = gyro.z();
    }
    
}

void DeviceCollector::onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
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
        
        float gyoro_g = sqrt(device->gyro.x*device->gyro.x + device->gyro.y*device->gyro.y + device->gyro.z*device->gyro.z);
        float g = sqrt(device->accel.x*device->accel.x + device->accel.y*device->accel.y + device->accel.z*device->accel.z);
//            cout << gyoro_g << endl;
//            cout << g << endl;
        if ( gyoro_g <= 0.2 ) device->gravity = g;
        
        ofQuaternion q;
        q.set(quat.x(), quat.z(), quat.y(), quat.w());
        ofVec3f linear_accel;
        ofMatrix4x4 mat;
        mat.translate(ofVec3f(0,device->gravity,0));
        mat.rotate(q);
        ofVec3f trans = mat.getTranslation();
        
        linear_accel = device->getAccel();
        linear_accel.x = linear_accel.x - trans.x;
        linear_accel.y = linear_accel.y - trans.z;
        linear_accel.z = linear_accel.z - trans.y;
        
        device->linear_accel.set(linear_accel);
//            cout << device->getAccel() << endl;
//            cout << mat.getTranslation() << endl;
//            cout << linear_accel << endl;
    }
}

void DeviceCollector::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->currentPose = pose;
//            if (pose == myo::Pose::fist) {
//                myo->vibrate(myo::Myo::vibrationShort);
//            }
        
    }
}

void DeviceCollector::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->onArm = true;
        device->whichArm = arm;
    }
}

void DeviceCollector::onArmUnsync(myo::Myo* myo, uint64_t timestamp)
{
    
    Device * device = findDevice(myo);
    if ( device ) {
        device->onArm = false;
    }
}

void DeviceCollector::onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
    myo->setStreamEmg(myo::Myo::streamEmgEnabled);
    Device * device = findDevice(myo);
    if ( device ) {
        device->isConnect = true;
    }
}

void DeviceCollector::onDisconnect(myo::Myo* myo, uint64_t timestamp) {
    Device * device = findDevice(myo);
    if ( device ) {
        device->isConnect = false;
    }
}

void DeviceCollector::onUnlock(myo::Myo* myo, uint64_t timestamp)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->isUnlocked = true;
    }
}

void DeviceCollector::onLock(myo::Myo* myo, uint64_t timestamp)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->isUnlocked = false;
    }
}



Device * DeviceCollector::findDevice(myo::Myo* myo){
    for (int i = 0; i < devices.size(); ++i) {
        if (devices[i]->myo == myo) {
            return devices[i];
        }
    }
    return 0;
}

    

}
