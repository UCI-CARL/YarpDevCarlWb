
#include <iostream>

using namespace std;

#include "CarlWbEPuck.h"
#include "CarlWbEPuckRobot.h"

using namespace yarp::carl;

#include <yarp/os/all.h>

using namespace yarp::os;

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Speaker.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;


#define myTrace   if(m_log <= Log::TraceType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).trace
#define myDebug   if(m_log <= Log::DebugType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).debug
#define myInfo    if(m_log <= Log::InfoType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).info
#define myWarning if(m_log <= Log::WarningType) yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).warning
#define myError   if(m_log <= Log::ErrorType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).error
#define myFatal   if(m_log <= Log::FatalType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).fatal

#define ifLogDebug   if(m_log <= yarp::os::Log::DebugType)	yarp::os::Log().debug()	

CarlWbEPuckBehavior::CarlWbEPuckBehavior(CarlWbEPuckRobot * robot, yarp::carl::CarlWbEPuck * device) : 
	m_robot(robot), m_device(device) {   
};

void CarlWbEPuckBehavior::run() {



}

bool CarlWbEPuckBehavior::threadInit() {
	return true; 
}

void CarlWbEPuckBehavior::threadRelease() {
	
}


// Braitenberg

class BraitenbergBehavior : public CarlWbEPuckBehavior
{
public:
public:
	BraitenbergBehavior(yarp::carl::CarlWbEPuckRobot* robot = nullptr, yarp::carl::CarlWbEPuck* device = nullptr):  
		CarlWbEPuckBehavior(robot, device)  {};

	virtual void run() {

		while (running) {

		}
	};


	virtual bool threadInit() {
		return true; 
	}

};



class BodyBehavior : public CarlWbEPuckBehavior
{
public:

	int steps;
public:
	BodyBehavior(yarp::carl::CarlWbEPuckRobot* robot, yarp::carl::CarlWbEPuck* device, int steps = 0) :CarlWbEPuckBehavior(robot, device), steps(steps) {};


	virtual void run() {

		int dt = 0;

		for (int i = 0; !this->isStopping() && (steps == 0 || i < steps) && dt != -1; i++) {

			m_robot->getSensorInput();
			m_device->transmitSensorInput();  // yarp writing port    

			m_robot->blinkLeds();

			m_robot->setActuators();
			dt = m_robot->step(1);

			this->yield(); 
		};
	};

};



const double CarlWbEPuckRobot::MAX_SPEED = 6.28;

const char* CarlWbEPuckRobot::proximitySensorNames[] = { "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7" };

const char* CarlWbEPuckRobot::lightSensorNames[] = { "ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7" };

const char* CarlWbEPuckRobot::ledNames[] = { "led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9" };


const double CarlWbEPuckRobot::weights[proximitySensorCount][2] = {   // Braitenberg coefficients  controller e-puck_avoid_obstacles.c
	{-1.3, -1.0}, {-1.3, -1.0}, {-0.5, 0.5}, {0.0, 0.0},
	{0.0, 0.0},   {0.05, -0.5}, {-0.75, 0},  {-0.75, 0} };


const double CarlWbEPuckRobot::offsets[2] = { 0.5 * MAX_SPEED, 0.5 * MAX_SPEED };

CarlWbEPuckRobot::CarlWbEPuckRobot(CarlWbEPuck* device): device(device) {

	robot = nullptr;
	accelerometer = nullptr;
	leftWheel = nullptr;
	rightWheel = nullptr;
	tof = nullptr;
	accelerometer = nullptr;
	gyro = nullptr;

	camera = nullptr;
	leftMotor = nullptr;
	rightMotor = nullptr;

	behavior = nullptr;

}



double CarlWbEPuckRobot::getTime() { return robot->getTime(); }

void CarlWbEPuckRobot::runBody(int steps) {

	behavior = new BodyBehavior(this, device, steps);   

	behavior->start(); 
}


void CarlWbEPuckRobot::halt() {

	if (behavior) {

		behavior->stop(); 

		delete behavior;
		behavior = nullptr;
	}

}



void CarlWbEPuckRobot::setSpeed(double left, double right) {

	std::lock_guard<std::mutex> guard(mtx); 

	speeds[LEFT] = left; 
	speeds[RIGHT] = right; 

}


void CarlWbEPuckRobot::setLedValues(const std::vector<int> indeces, const int led[]) {

	std::lock_guard<std::mutex> guard(mtx);

	for(auto iter = indeces.begin(); iter < indeces.end(); iter++) {
		ledValues[*iter] = led[*iter];
	}
}


void CarlWbEPuckRobot::setText(std::string text) {

	std::lock_guard<std::mutex> guard(mtx);

	this->text = text;
}


void CarlWbEPuckRobot::initDevices() {

	// Camera
	camera = robot->getCamera("camera");
	assert(camera);
	camera->enable(wbCameraTimeStep);

	// Accelerometer
	accelerometer = robot->getAccelerometer("accelerometer");
	assert(accelerometer); 
	accelerometer->enable(wbTimeStep);

	// Gyro
	gyro = robot->getGyro("gyro"); 
	assert(gyro); 
	gyro->enable(wbTimeStep); 


	// get the position 
	leftWheel = robot->getPositionSensor("left wheel sensor");
	assert(leftWheel);
	leftWheel->enable(wbTimeStep);
	rightWheel = robot->getPositionSensor("right wheel sensor");
	assert(rightWheel);
	rightWheel->enable(wbTimeStep);


	// Proximity Sensors
	for (int i = 0; i < proximitySensorCount; i++) {
		proximitySensors[i] = robot->getDistanceSensor(proximitySensorNames[i]);
		assert(proximitySensors[i]);
		proximitySensors[i]->enable(wbTimeStep);
	}

	tof = robot->getDistanceSensor("tof");
	assert(tof);
	tof->enable(wbTimeStep);

	// Light Sensors
	for (int i = 0; i < lightSensorCount; i++) {
		lightSensors[i] = robot->getLightSensor(lightSensorNames[i]);
		assert(lightSensors[i]);
		lightSensors[i]->enable(wbTimeStep);
	}


	// Actuators

	// get the motor devices
	leftMotor = robot->getMotor("left wheel motor");
	assert(leftMotor);
	rightMotor = robot->getMotor("right wheel motor");
	assert(rightMotor);

	// set the target position of the motors
	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
	leftMotor->setVelocity(0);
	rightMotor->setVelocity(0);


	// LEDs
	for (int i = 0; i < ledCount; i++) {
		leds[i] = robot->getLED(ledNames[i]);
		assert(leds[i]);
		ledValues[i] = false;
	}


	speaker = robot->getSpeaker("speaker"); 
	assert(speaker); 


}

void CarlWbEPuckRobot::resetActuatorValues() {
	for (int i = 0; i < 2; i++)
		speeds[i] = 0.0;
	for (int i = 0; i < ledCount; i++)
		ledValues[i] = false;
}

void CarlWbEPuckRobot::getSensorInput() {

	// Streaming with controler time step (scalar or 1-dim vectors) 

	for (int i = 0; i < proximitySensorCount; i++) {
		proximitySensorValues[i] = proximitySensors[i]->getValue();
		psValues[i] = (int) proximitySensors[i]->getValue(); // trunc
		double minValue = proximitySensors[i]->getMinValue(); // 4095   // ~ 62
		proximitySensorValues[i] -= minValue;
		proximitySensorValues[i] /= 600.0;
	}

	for (int i = 0; i < lightSensorCount; i++) {
		lsValues[i] = (int)(lightSensors[i]->getValue()); // trunc
	}

	if(tof) {
		tofValue = (int) tof->getValue(); // trunc
	}
	
	whlsValues[LEFT] = (int) leftWheel->getValue();
	whlsValues[RIGHT] = (int) rightWheel->getValue();

	if (accelerometer) {
		const double *d = accelerometer->getValues();
		for (int i = 0; i < 3; i++)
			accelValues[i] = d[i];
	}

	if (gyro) {
		const double* d = gyro->getValues();
		for (int i = 0; i < 3; i++)
			gyroValues[i] = d[i];
	}

}


bool CarlWbEPuckRobot::cliffDetected() {

	if (tof->getValue() < 100)
		return true;

	if (proximitySensorValues[1] > 0.1) {
			ledValues[1] = 0xFF0000;  // RGB 
			return true;
	}

	if (proximitySensorValues[7] > 0.1) {
		ledValues[7] = 0xFF0000;  // RGB 
		return true;
	}

	return false;
}

void CarlWbEPuckRobot::setActuators() {
	for (int i = 0; i < ledCount; i++) {
		leds[i]->set(ledValues[i]);
	}

	leftMotor->setVelocity(speeds[LEFT]);
	rightMotor->setVelocity(speeds[RIGHT]); 

	// consume ressource
	if (!text.empty() && text != ".") {
		speaker->speak(text, 100);
		text = ".";  
	}
}


void CarlWbEPuckRobot::blinkLeds() {
	static int counter = 0;
	counter++;

	if (counter % 20 < 2) {  // 2 = blink length
		ledValues[3] = 0x00FF00;  // RGB 
		ledValues[5] = 0xFF0000;  // RGB 
	}

	if (tof->getValue() < 200)
		ledValues[0] = true;  // top front 
}

void CarlWbEPuckRobot::calculateBraitenberg() {  //coeficients
	for (int i = 0; i < 2; i++) {
		speeds[i] = 0.0;
		for (int j = 0; j < proximitySensorCount; j++) 
			speeds[i] += proximitySensorValues[j] * weights[j][i];

		speeds[i] = offsets[i] + speeds[i] * MAX_SPEED; 
		if (speeds[i] > MAX_SPEED)
			speeds[i] = MAX_SPEED;
		else if (speeds[i] < -MAX_SPEED)
			speeds[i] = -MAX_SPEED;
	
	}
}

void CarlWbEPuckRobot::runBraitenberg(int steps) {

	int turned = 0;
	int dt = 0; 

	for (int i = 0; i < steps && dt != -1; i++) { 

		resetActuatorValues();

		getSensorInput();
		device->transmitSensorInput();  // yarp writing port    

		blinkLeds();
		if (cliffDetected()) {
			goBackwards();
			if (turned++ % 4)
				turnLeft();
			else
				turnRight(); 
		}
		else {
			calculateBraitenberg();
		}
		setActuators();
		dt = step(1);
	};
}


void CarlWbEPuckRobot::goBackwards() {
	leftMotor->setVelocity(-0.25 * MAX_SPEED);
	rightMotor->setVelocity(-0.25 * MAX_SPEED);
	passiveWait(0.2);
}

void CarlWbEPuckRobot::turnLeft() {
	leftMotor->setVelocity(-0.66 * MAX_SPEED);
	rightMotor->setVelocity(0.66 * MAX_SPEED); 
	passiveWait(0.2); 
}

void CarlWbEPuckRobot::turnRight() {
	leftMotor->setVelocity(0.66 * MAX_SPEED);
	rightMotor->setVelocity(-0.66 * MAX_SPEED);
	passiveWait(0.2);
}

int CarlWbEPuckRobot::step(int steps) {
	int dt = 0;  
	for (int i = 0; i < steps && dt >= 0; i++) {
		dt= robot->step(wbTimeStep);
	}
	return dt;
}

void CarlWbEPuckRobot::passiveWait(double sec) {
	double startTime = robot->getTime(); 
	do {
		step();
	} while (startTime + sec > robot->getTime());
}

bool CarlWbEPuckRobot::init() {

	//wb_robot_init();
	robot = new Robot();

	auto name = robot->getName();
	if (name == "e-puck2") {
		printf("e-puck2 robot\n");
		wbTimeStep = 64;
		wbCameraTimeStep = 64;
	}
	else {  // original e-puck
		printf("e-puck robot\n");
		wbTimeStep = 256;
		wbCameraTimeStep = 1024;
	}

	wbBasicTimeStep = robot->getBasicTimeStep();

	resetActuatorValues();   

	initDevices();

	int ok = step();

    return true;
}


void CarlWbEPuckRobot::release() {

	if (robot)
		delete robot;

	
}
