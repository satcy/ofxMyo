#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(0);
    myo.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255);
    for ( int i=0; i<myo.getDevices().size(); i++ ) {
        stringstream s;
        s << "id: " << myo.getDevices()[i]->getId() << endl;
        s << "which: " << myo.getDevices()[i]->getWhichArm() << endl;
        s << "pose: " << myo.getDevices()[i]->getPose() << endl;
        s << "accel:          ";
        s << myo.getDevices()[i]->getAccel().x << ",";
        s << myo.getDevices()[i]->getAccel().y << ",";
        s << myo.getDevices()[i]->getAccel().z << endl;
        s << "gyro:           ";
        s << myo.getDevices()[i]->getGyro().x << ",";
        s << myo.getDevices()[i]->getGyro().y << ",";
        s << myo.getDevices()[i]->getGyro().z << endl;
        s << "quaternion:     ";
        s << myo.getDevices()[i]->getQuaternion().x() << ",";
        s << myo.getDevices()[i]->getQuaternion().y() << ",";
        s << myo.getDevices()[i]->getQuaternion().z() << ",";
        s << myo.getDevices()[i]->getQuaternion().w() << endl;
        s << "roll/pitch/yaw: ";
        s << myo.getDevices()[i]->getRoll() << ",";
        s << myo.getDevices()[i]->getPitch() << ",";
        s << myo.getDevices()[i]->getYaw() << endl;
        s << "raw data:       ";
        for ( int j=0; j<8; j++ ) {
            s << myo.getDevices()[i]->getEmgSamples()[j];
            s << ",";
        }
        s << endl;
        ofSetColor(255);
        ofDrawBitmapString(s.str(), 0,12 + i*100);
    }
}

void ofApp::exit(){
    myo.stop();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}