#include "./config.h"
#include "ofApp.h"
#include "ImHelpers.h"


void ofApp::setup()
{
	ofSetFrameRate(60);
	ofSetLogLevel(OF_LOG_VERBOSE);

	ofFile file("config.json");
	ofJson config;
	if(file.exists()){
		file >> config;
	}




	// enable depth->video image calibration
	_kinect.setRegistration(true);
	_kinect.init();
	// kinect.init(true); // shows infrared instead of RGB video image
	// kinect.init(false, false); // disable video image (faster fps)

	_kinect.open(); // opens first available kinect
	// kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	// kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #

	// print the intrinsic IR sensor values
	if (_kinect.isConnected())
	{
		ofLogNotice() << "sensor-emitter dist: " << _kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << _kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << _kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << _kinect.getZeroPlaneDistance() << "mm";
	}



	// zero the tilt on startup
	_tiltAngle.set("tiltAngle", 0, 0, 50);
	_tiltAngle.addListener(this, &ofApp::onTiltAngleChange);
	_kinect.setCameraTiltAngle(_tiltAngle);
	_colorImage.allocate(_kinect.width, _kinect.height);
	_depthImage.allocate(_kinect.width, _kinect.height);
	_lastDepthImage.allocate(_kinect.width, _kinect.height);
	_diffDepthImage.allocate(_kinect.width, _kinect.height);
	_contourDepthImage.allocate(_kinect.width, _kinect.height);

	// connect
	_midiOut.listOutPorts();
	// midiOut.openPort(0); // by number
	// midiOut.openPort("IAC Driver Pure Data In"); // by name
	_midiOut.openVirtualPort("dd-kinect"); // open a virtual port


	//gui
	_gui.setup(nullptr, true, ImGuiConfigFlags_ViewportsEnable );

}

void ofApp::update() {
    _kinect.update();
    if (_kinect.isFrameNew()) {
		_colorImage.setFromPixels(_kinect.getPixels());
		_colorImage.flagImageChanged();
		_depthImage.setFromPixels(_kinect.getDepthPixels());
		_depthImage.flagImageChanged();
		_contourDepthImage.setFromPixels(_kinect.getDepthPixels());
        _contourDepthImage.flagImageChanged();
		_diffDepthImage.absDiff(_depthImage, _lastDepthImage);
		_diffDepthImage.flagImageChanged();
		_diffDepthImage.threshold(10); // adjust threshold value as needed

		_contourDepthImage.threshold(100);
		_contourFinder.findContours(_depthImage, 100, (_kinect.width * _kinect.height) / 3, 10, false, true);



		_lastDepthImage = _depthImage;
    }
	_midiOut.sendControlChange(MIDI_CHANNEL, MIDI_CONTROLLER_MOVEMENT, ofMap(getNormalizedMovement(0, 0, _kinect.getWidth(), _kinect.getHeight()), 0, 1, 0, 127));

	// ofLogNotice() << getNearestPoint(0, 0, _kinect.getWidth(), _kinect.getHeight());
	ofLogNotice() << _contourFinder.blobs.size();
}

void ofApp::draw() {
	auto width = 320;
	auto height = 240;
    ofBackground(0);
	_colorImage.draw(0, 0, width, height);
	ofPushMatrix();
    ofTranslate(420, 10); // position the point cloud
    // drawPointCloud();
    ofPopMatrix();
	_depthImage.draw(0, height, width, height);
	_diffDepthImage.draw(width, height, width, height);
	_contourFinder.draw(2*width, height, width, height);


	auto mainSettings = ofxImGui::Settings();
	_gui.begin();
    
    // Show the ImGui test window. Most of the sample code is in ImGui::ShowDemoWindow()
    ImGui::SetNextWindowPos( ofVec2f( ofGetWindowPositionX(), ofGetWindowPositionY()), ImGuiCond_Once);
    ImGui::SetNextWindowSize( ofVec2f(ofGetWidth(), ofGetHeight()), ImGuiCond_Once);

	static bool bCollapse = false;
	if (ofxImGui::BeginWindow("settings", mainSettings, ImGuiWindowFlags_NoCollapse, &bCollapse)){
    	// ImGui::ShowDemoWindow();
		ofxImGui::AddStepper(_tiltAngle);
		ofxImGui::EndWindow(mainSettings);
	}


    //required to call this at end
    _gui.end();
}


void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(auto y = 0; y < h; y += step) {
		for(auto x = 0; x < w; x += step) {
			if(_kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(_kinect.getColorAt(x,y));
				mesh.addVertex(_kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

float ofApp::getNormalizedMovement(int x, int y, int width, int height) {
    // Clamp to image bounds
    x = ofClamp(x, 0, _diffDepthImage.getWidth() - 1);
    y = ofClamp(y, 0, _diffDepthImage.getHeight() - 1);
    width = ofClamp(width, 1, _diffDepthImage.getWidth() - x);
    height = ofClamp(height, 1, _diffDepthImage.getHeight() - y);

    unsigned char* pixels = _diffDepthImage.getPixels().getData();
    int imgWidth = _diffDepthImage.getWidth();

    float movementSum = 0.0f;
    int pixelCount = width * height;

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            int index = (y + j) * imgWidth + (x + i);
            movementSum += pixels[index];
        }
    }

    // Normalize to range [0.0, 1.0]
    return movementSum / (255.0f * pixelCount);
}
int ofApp::getNearestPoint(int x, int y, int width, int height) {
    // Clamp bounds to image size
    x = ofClamp(x, 0, _kinect.getWidth() - 1);
    y = ofClamp(y, 0, _kinect.getHeight() - 1);
    width = ofClamp(width, 1, _kinect.getWidth() - x);
    height = ofClamp(height, 1, _kinect.getHeight() - y);

    int minDistance = std::numeric_limits<int>::max();
    for (int j = y; j < y + height; ++j) {
        for (int i = x; i < x + width; ++i) {
            int dist = _kinect.getDistanceAt(i, j);
            if (dist > 0 && dist < minDistance) {
                minDistance = dist;
            }
        }
    }

    return (minDistance == std::numeric_limits<int>::max()) ? 0 : minDistance;
}


void ofApp::exit()
{
	_kinect.setCameraTiltAngle(0); // zero the tilt on exit
	_kinect.close();
}

void ofApp::keyPressed(int key)
{
	switch (key)
	{
	case 'o':
		_kinect.setCameraTiltAngle(_tiltAngle); // go back to prev tilt
		_kinect.open();
		break;

	case 'c':
		_kinect.setCameraTiltAngle(0); // zero the tilt
		_kinect.close();
		break;

	case OF_KEY_UP:
		_tiltAngle++;
		if (_tiltAngle > 30)
			_tiltAngle = 30;
		_kinect.setCameraTiltAngle(_tiltAngle);
		break;

	case OF_KEY_DOWN:
		_tiltAngle--;
		if (_tiltAngle < -30)
		_tiltAngle = -30;
		_kinect.setCameraTiltAngle(_tiltAngle);
		break;
	}
}

void ofApp::mouseDragged(int x, int y, int button) {}
void ofApp::mousePressed(int x, int y, int button) {}
void ofApp::mouseReleased(int x, int y, int button) {}
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::windowResized(int w, int h) {}

void ofApp::onTiltAngleChange(int & value){
	_kinect.setCameraTiltAngle(_tiltAngle);
}
	