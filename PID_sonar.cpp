#include <Aria.h>
#include <stdio.h>
#include <iostream>
#include <conio.h>

using namespace std;
#include "Highlevel.h"

// PID variables
double dState; // Last position input
double iState; // Integrator state
double iMax, iMin;

// Maximum and minimum allowable integrator state
double iGain, // integral gain
pGain, // proportional gain
dGain; // derivative gain

// Calculates a random integer between min and max inclusive
int randomBetween(int min, int max) {
	return rand() % (max - min + 1) + min;
}

// Pythagoras theorem for distance calcuation for random turning
double distanceBetween(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

}

// Updates the PID values
double updatePID(double error)
{
	double pTerm, dTerm, iTerm;
	pTerm = pGain * error;
	// calculate the proportional term

	// calculate the integral state with appropriate limiting
	iState += error;
	if (iState > iMax) {
		iState = iMax;

	}
	else if (iState < iMin) {
		iState = iMin;

	}

	iTerm = iGain * iState; // calculate the integral term
	dTerm = dGain * (error - dState);
	dState = error;

	return pTerm + iTerm + dTerm;
}

// Constructor
FSM::FSM() : ArAction("FSM")
{
	currentState = FOLLOW_WALL_RIGHT;

	// Zero the starting wander coordinates
	startWanderX = 0;
	startWanderY = 0;

	speed = FULL_SPEED; // 0.5m/s
	deltaHeading = 0; // 0 degrees

					  // Set PID values
	pGain = 0.9;
	iGain = 0.0;
	dGain = 0;
}

boolean obstacleLeft = false;

// Body of action
ArActionDesired * FSM::fire(ArActionDesired d)
{
	// Read sonar readings
	rightSonar = myRobot->getClosestSonarRange(-100, -20);
	frontSonar = myRobot->getClosestSonarRange(-20, 20);
	printf("right: %f", rightSonar);
	printf("front: %f", frontSonar);
	// The current distance we have travelled this cycle from when we started wandering
	double currentDistance = distanceBetween(startWanderX, startWanderY,
		myRobot->getX(), myRobot->getY());

	switch (currentState) {

		case OBSTACLE_INFRONT:
			printf("Obstacle infront \n");
			speed = FULL_SPEED * 0.5;
			error = OBSTACLE_DISTANCE - frontSonar + 200;
			deltaHeading = updatePID(error);
			//deltaHeading = 0.9 * (OBSTACLE_DISTANCE + 250 - frontSonar);
			printf("speed: %d \n", speed);
			printf("!!!!!!!!!!!!!!!!!!!!!!!!front: %f \n", frontSonar);
			printf("delta: %f \n", deltaHeading);

			if (frontSonar >= OBSTACLE_DISTANCE - 100) { // Object no longer in the near distance, return to wandering
				currentState = FOLLOW_WALL_RIGHT;
				error = 0;
				iState = 0;
				dState = 0;
				startWanderX = myRobot->getX();
				startWanderY = myRobot->getY();

			}
		    break;

	

		case FOLLOW_WALL_RIGHT:
			printf("Following wall right \n");
			speed = FULL_SPEED;

			// Find error
			if (rightSonar < (2 * FOLLOW_WALL_DISTANCE)) { // Right < 1m
				error = FOLLOW_WALL_DISTANCE - rightSonar;
				deltaHeading = updatePID(error);

			}
			else {
				// Conner, turn right
				speed = FULL_SPEED * 0.5;
				deltaHeading = updatePID(error);
				//deltaHeading = FULL_SPEED * -0.3;
			}

			// Ensure we are still checking front sonar even when following the wall, incase anything is in front of us at the end of the wall
			if (frontSonar < (OBSTACLE_DISTANCE + 100)) {
				currentState = OBSTACLE_INFRONT;
				iState = 0;
				dState = 0;
				speed = FULL_SPEED * 0.5;
				error = OBSTACLE_DISTANCE - frontSonar + 200;
				deltaHeading = updatePID(error);
				printf("speed: %d \n", speed);
				printf("delta: %f", deltaHeading);

			}
			break;

	}
	desiredState.reset(); // reset the desired state (must be done)
	myRobot->setVel2(speed - deltaHeading, speed + deltaHeading); // set the speed of the robot in the desired state
	//desiredState.setDeltaHeading(deltaHeading); // set the new angle

	return &desiredState; // give the desired state to the robot for actioning
}



int main(int argc, char** argv)
{
	//robot and devices
	ArRobot myRobot;
	ArSonarDevice sonar;
	ArSick sick;               //the laser

	//add laser to robot
	myRobot.addRangeDevice(&sick);

	//Device connection
	ArDeviceConnection *con;
	ArSerialConnection laserCon;
	
	ArSerialConnection *serCon = new ArSerialConnection;
	serCon -> setPort();
	/* serCon->setBaud(38400); */
	con = serCon;
	/* laserCon.setPort(ArUtil::COM3); */

	// set the connection on the robot
	myRobot.setDeviceConnection(con);

	//initialize aria and aria's logging destination and level
	Aria::init();
	ArLog::init(ArLog::StdErr, ArLog::Normal);

	//parser for the command line arguments
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();

	//Connect to the robot, then load parameter files for this robot.
	ArRobotConnector robotConnector(&parser, &myRobot);

	if (!robotConnector.connectRobot())
	{
		//Error!
		ArLog::log(ArLog::Terse, "Error, could not connect to the robot.\n");
		if (parser.checkHelpAndWarnUnparsed())
		{
			//help not given
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	// Trigger argument parsing
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}

	ArLog::log(ArLog::Normal, "Program is Connected.");

	FSM *fsm = new FSM;
	ArActionStallRecover recover;
	
	myRobot.addRangeDevice(&sonar);
	
	myRobot.addAction(&recover, 100);
	myRobot.addAction(fsm, 50);
 
	myRobot.enableMotors();
	//myrobot.comInt(ArCommands::SOUNDTOG, 0);
	myRobot.run(true);
}