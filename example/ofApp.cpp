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
        s << "id: " << myo.getDevices()[i]->id << endl;
        s << "which: " << myo.getDevices()[i]->whichArm << endl;
        s << "pose: " << myo.getDevices()[i]->currentPose << endl;
        s << "accel:          ";
        s << myo.getDevices()[i]->a_x << ",";
        s << myo.getDevices()[i]->a_y << ",";
        s << myo.getDevices()[i]->a_z << endl;
        s << "gyro:           ";
        s << myo.getDevices()[i]->g_x << ",";
        s << myo.getDevices()[i]->g_y << ",";
        s << myo.getDevices()[i]->g_z << endl;
        s << "quaternion:      ";
        s << myo.getDevices()[i]->q.x() << ",";
        s << myo.getDevices()[i]->q.y() << ",";
        s << myo.getDevices()[i]->q.z() << ",";
        s << myo.getDevices()[i]->q.w() << endl;
        s << "roll/pitch/yaw: ";
        s << myo.getDevices()[i]->roll << ",";
        s << myo.getDevices()[i]->pitch << ",";
        s << myo.getDevices()[i]->yaw << endl;
        s << "raw data:       ";
        for ( int j=0; j<8; j++ ) {
            s << myo.getDevices()[i]->emgSamples[j];
            s << ",";
        }
        s << endl;
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