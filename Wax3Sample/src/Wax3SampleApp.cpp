#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Utilities.h"

#include "Accelerometer.h"
#include "Wax3Receiver.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class Wax3SampleApp : public AppNative {
  public:
	void setup();
	void update();
	void draw();
    
    Accelerometer mAccel;
    Wax3Receiver mWax3;
    float mFlash;
};

void Wax3SampleApp::setup()
{
    // Initialize Wax3 receiver with its port name
    // (type this in terminal to find the connected devices: ls /dev/tty.*)
    mWax3.setup("tty.usbmodem");
    mWax3.setDebug(false);
    
    // Initialize accelerometer with the Wax3 receiver as data source
    // make sure this ID is the device ID of the emitter, as set in CoolTerm
    mAccel.setup(5, &mWax3);
    
    mFlash = 0.0f;
}


void Wax3SampleApp::update()
{
    // Update the receiver to get the data
    // (we only need this if we tell it to not be threaded)
    mWax3.update();
    
    // Update the accelerometer to obtain new data from its source
    mAccel.update();
    
    // Process new data to detect spikes
    int newReadings = mAccel.getNumNewReadings();
    for (int i=0; i<newReadings; i++) {
        float acc = mAccel.getAccelMagnitude(i);
        if (acc > 15.0 && mFlash == 0.0f) mFlash = 1.0f;
    }
    
    if(mFlash > 0.0f) mFlash -= 0.01f;
    else mFlash = 0.0f;
}

void Wax3SampleApp::draw()
{
	gl::clear(Color::gray(mFlash));
    gl::enableAlphaBlending();

    if (mAccel.isActive()) {
    
        // graph acceleration history per axis
        float scaleX = 6;
        float scaleY = 10;
        Shape2d red;
        Shape2d green;
        Shape2d blue;
        red.moveTo(0, 125 - mAccel.getAccel().x * scaleY);
        green.moveTo(0, 250 - mAccel.getAccel().y * scaleY);
        blue.moveTo(0, 375 - mAccel.getAccel().z * scaleY);

        for (int i=1; i<mAccel.getHistoryLength(); i++) {
            red.lineTo(i * scaleX, 125 - mAccel.getAccel(i).x * scaleY);
            green.lineTo(i * scaleX, 250 - mAccel.getAccel(i).y * scaleY);
            blue.lineTo(i * scaleX, 375 - mAccel.getAccel(i).z * scaleY);
        }
        ci::gl::color(1, 0, 0);
        ci::gl::draw(red);
        ci::gl::color(0, 1, 0);
        ci::gl::draw(green);
        ci::gl::color(0, 0, 1);
        ci::gl::draw(blue);
        
        // show pitch and roll extracted from acceleration
        gl::enableDepthRead();
        gl::enableDepthWrite();
        
        gl::pushMatrices();{
            gl::translate(getWindowCenter());
            gl::rotate(Vec3f(mAccel.getPitch(), 0, mAccel.getRoll()));
            gl::drawColorCube(Vec3f::zero(), Vec3f(150, 50, 80));
        }gl::popMatrices();
        
        gl::disableDepthRead();
        gl::disableDepthWrite();
    }
    else {
        gl::drawStringCentered("Wax3 not found. Check usb port name and receiver ID", getWindowCenter());
    }
    
    gl::drawString(to_string((int)getAverageFps()), Vec2f(20, 20));
}

CINDER_APP_NATIVE( Wax3SampleApp, RendererGl )
