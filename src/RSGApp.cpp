#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Kinect2.h"
#include "cinder/osc/Osc.h"
#include "cinder/Log.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#define USE_UDP 1

#if USE_UDP
using Sender = osc::SenderUdp;
#else
using sender = osc::SenderTcp;
#endif

const std::string destinationHost = "10.0.1.2";
const uint16_t destinationPort = 10001;
const uint16_t localPort = 10000;

// FRONT CAMERA
class RSGApp : public App {
public:
	RSGApp();
	void setup() override;
	void update() override;
	void mouseDown(MouseEvent event) override;
	void keyDown(KeyEvent event) override;
	void draw() override;
	float mapIt(float s, float a1, float a2, float b1, float b2);
	void shutdown();

	void onSendError(asio::error_code error);

	bool mFullScreen = true;

	gl::Texture2dRef mTexture;
	vector<vec2> mTrail;

	Sender mSender;
	bool	mIsConnected;

private:
	Kinect2::BodyFrame mBodyFrame;
	ci::Channel8uRef mChannelBodyIndex;
	ci::Channel16uRef mChannelDepth;
	Kinect2::DeviceRef mDevice;

	int width;
	int height;
};

RSGApp::RSGApp() : mSender(localPort, destinationHost, destinationPort), mIsConnected(false) {
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame frame) {
		mBodyFrame = frame;
	});
	mDevice->connectBodyIndexEventHandler([&](const Kinect2::BodyIndexFrame frame) {
		mChannelBodyIndex = frame.getChannel();
	});
	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame frame) {
		mChannelDepth = frame.getChannel();
	});
}

void RSGApp::setup()
{
	//cout << "TEST" << endl;
	OutputDebugStringW(L"HERE~~~~~~~~~~~~~~~~~~~\n");
	setFullScreen(mFullScreen);
	width = getWindowWidth();
	height = getWindowHeight();
	auto img = loadImage(loadAsset("TOMS_GP_v7.11.png"));
	mTexture = gl::Texture2d::create(img);

	// set up OSC
	try {
		// Bind the sender to the endpoint. This function may throw. The exception will
		// contain asio::error_code information.
		mSender.bind();
	}
	catch (const osc::Exception &ex) {
		CI_LOG_E("Error binding: " << ex.what() << " val: " << ex.value());
		quit();
	}

#if ! USE_UDP
	mSender.connect(
		// Set up the OnConnectFn. If there's no error, you can consider yourself connected to
		// the endpoint supplied.
		[&](asio::error_code error) {
		if (error) {
			CI_LOG_E("Error connecting: " << error.message() << " val: " << error.value());
			quit();
		}
		else {
			CI_LOG_V("Connected");
			mIsConnected = true;
		}
	});
#else
	// Udp doesn't "connect" the same way Tcp does. If bind doesn't throw, we can
	// consider ourselves connected.
	mIsConnected = true;
#endif
}

// Unified error handler. Easiest to have a bound function in this situation,
// since we're sending from many different places.
void RSGApp::onSendError(asio::error_code error)
{
	cout << "oh no" << endl;
	if (error) {
		cout << "ERROR" << endl;
		CI_LOG_E("Error sending: " << error.message() << " val: " << error.value());
		// If you determine that this error is fatal, make sure to flip mIsConnected. It's
		// possible that the error isn't fatal.
		mIsConnected = false;
		try {
#if ! USE_UDP
			// If this is Tcp, it's recommended that you shutdown before closing. This
			// function could throw. The exception will contain asio::error_code
			// information.
			mSender.shutdown();
#endif
			// Close the socket on exit. This function could throw. The exception will
			// contain asio::error_code information.
			mSender.close();
		}
		catch (const osc::Exception &ex) {
			CI_LOG_EXCEPTION("Cleaning up socket: val -" << ex.value(), ex);
		}
		quit();
	}
}


void RSGApp::mouseDown(MouseEvent event) {
	mFullScreen = !mFullScreen;
	setFullScreen(mFullScreen);
}

void RSGApp::keyDown(KeyEvent event) {
	if (event.getChar() == 'a') {
		mTrail.clear();
	}
}

void RSGApp::update()
{
}

void RSGApp::draw()
{
	gl::clear(Color::white());
	gl::color(Color::white());
	gl::draw(mTexture, Rectf(0, 0, getWindowWidth(), getWindowHeight()));
	int bodyCount = 0;
	for (const Kinect2::Body &body : mBodyFrame.getBodies()) {
		if (body.isTracked()) {
			for (const auto& joint : body.getJointMap()) {
				// if it is the mid spine
				if (joint.first == 1) {
					vec3 pos = joint.second.getPosition();
					// console() << pos.y << endl;
					// distance between start of the grid and the camera (in pixels)
					float offset1 = (width / 19.0) * .45;
					// distance between camera and downstage edge & distance between camera threshold in the x direction and upstage edge
					float offset2 = (height / 15.0) * 4.15;
					// distance between edge of downstage and threshold for camera tracking in x direction
					float offset3 = (height / 15.0) * 1.15;
					float xPos = mapIt(pos.x, -2.45, 2.45, width*.67+50, width*.33+50);
					//cout << "here" << endl;
					//cout << pos.x << endl;
			//	std::wstringstream ss;
				//	ss << "Z pos: " << pos.z << endl;
			//		OutputDebugString(ss.str().c_str());
					float yPos = mapIt(pos.z, -.5, 5.0, height-20, height*.35);
					gl::color(Color(1, 0, 0));
					gl::drawSolidCircle(vec2(xPos, yPos), 25);
					mTrail.push_back(vec2(xPos, yPos));
			    	std::string route = "/pos/front/" + to_string(bodyCount);
					osc::Message msg(route);
					msg.append(xPos/width);
					msg.append(yPos/height);
					mSender.send(msg, std::bind( &RSGApp::onSendError, this, std::placeholders::_1));
				}
			}
			bodyCount++;
		}
	}
	for (vec2& point : mTrail) {
		gl::drawSolidCircle(point, 5);
	}
}

float RSGApp::mapIt(float s, float a1, float a2, float b1, float b2) {
	return b1 + (s - a1)*(b2 - b1) / (a2 - a1);
}

void RSGApp::shutdown() {
	mDevice->stop();
}

CINDER_APP(RSGApp, RendererGl)
