#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxMidi.h"

#include "ofxImGui.h"

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);


	void drawPointCloud();
	float getNormalizedMovement(int x, int y, int width, int height);
	int getNearestPoint(int x, int y, int width, int height);

	void onTiltAngleChange(int & value);

	
	
	ofEasyCam _easyCam;
	ofxMidiOut _midiOut;
	// kinect
	ofxKinect _kinect;
	ofxCvColorImage _colorImage;
	ofxCvGrayscaleImage _depthImage;
	ofxCvGrayscaleImage _lastDepthImage;
	ofxCvGrayscaleImage _diffDepthImage;
	ofxCvGrayscaleImage _contourDepthImage;
	ofxCvContourFinder _contourFinder; 

	ofParameter<int> _tiltAngle;


	// gui
	ofxImGui::Gui _gui;

};
