#include <Aria.h>
#include <stdio.h>
#include <iostream>
#include <conio.h>
using namespace std;




int main(int argc, char** argv)
{

	Aria::init();
	ArPose pose;
	ArRobot robot;
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		cout << "Robot Connected!" << endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();
	//Laser Connection;
	argParser.addDefaultArgument("-connectLaser");
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);
	if (laserConnector.connectLasers())
		cout << "Laser Connected!" << endl;
	ArLaser *laser = robot.findLaser(1);
	ArSensorReading *sonarSensor[8];
	//**ROBOT SETUP & CONNECTION**

	//********************************************** Add your code **********************************************/
	// add to run
	double laserRange[18];
	double laserAngle[18];
	laser->lockDevice();
	for (int i = 0; i < 18; i++)
	{
		laserRange[i] = laser->currentReadingPolar(10 * i - 90, 10 * (i + 1) - 90, &laserAngle[i]);
	}
	laser->unlockDevice();

	float desired_dis, e = 0, e_i = 0, ep = 0, ed = 0, kp, ki, kd, left = 0, right = 0, PID = 0;
	desired_dis = 500;
	float basVal = 110, reading;

	kp = 0.5;
	ki = 0.00;
	kd = 0.000;
	int m = 1;
	while (true)
	{

		// get sonar readings

		for (int i = 0; i < 18; i++)
			laserRange[i] = laser->currentReadingPolar(10 * i - 90, 10 * (i + 1) - 90, &laserAngle[i]);



		if (laserRange[4] < 1500 | laserRange[1] < 1000)
		{
			m = 0;
			reading = laserRange[4];
			if (reading > 1000)
			{reading = laserRange[1];desired_dis = 588;}
			else
			{reading = laserRange[4];desired_dis = 750;}
			// Perform error calculation – current error, integral error and difference between the current and previous error
			e = desired_dis - reading;
			e_i = e_i + e;
			ed = e - ep;
			ep = e;
			// Calculate PID output – round it to be an integer
			PID = (kp*ep + ki*e_i + kd*ed);
			if (PID > 200)
			PID = 200;
			// Calculate the left and right speed of motors
			left = basVal - PID;
			right = basVal + PID;
	    } // END if PID right


		else if (laserRange[13] < 950 | laserRange[16] < 650)
		{
			reading = laserRange[13];
			if (reading > 1000)
			{
				reading = laserRange[16];desired_dis = 588;
			}
			else
			{
				reading = laserRange[14];desired_dis = 750;
			}
			// Perform error calculation – current error, integral error and difference between the current and previous error
			e = desired_dis - reading;
			e_i = e_i + e;
			ed = e - ep;
			ep = e;
			// Calculate PID output – round it to be an integer
			PID = (kp*ep + ki*e_i + kd*ed);
			if (PID > 200)
				PID = 200;
			// Calculate the left and right speed of motors
			left = basVal + PID;
			right = basVal - PID;
		} // END if PID LEFT
		



		if(laserRange[8]<550 & m)
		{
			if(laserRange[6]>laserRange[10])
			{
				left = 200;right = 0;
			}
			else
			{
				left = 0;right = 200;
			}

		}//obstacle_avoidance

		if(laserRange[0]>2500 & laserRange[17]>1000 & laserRange[8]>1000 & laserRange[5]>2000 & laserRange[11]>1000)
		{
			left = 100;right = 100;
		}

		m = 1;
		robot.setVel2(left, right);
		cout << "left: " << left << ", right:" << right << endl;

	}


	return 0;
}