#include <Aria.h>
#include <stdio.h>
#include <iostream>
#include <conio.h>
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>

using namespace std;
#include "Memberships.h"

int main(int argc, char** argv)
{
	ArRobot robot;
	Aria::init();
	ArPose pose;
	

	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobotConnector robotConnector(&argParser, &robot);
	if(robotConnector.connectRobot())
	     cout << "Robot Connected!" << endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();
	//Laser Connection;
	argParser.addDefaultArgument("-connectLaser");
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);
	if(laserConnector.connectLasers())
	     cout << "Laser Connected!" << endl;
	ArLaser *laser = robot.findLaser(1);
	//**ROBOT SETUP & CONNECTION**

	//***********************************************************************************************/
	
	/* 
	 * classification and initialization 
	 */
	
	// Define the fuzzifier
	Memberships RFS_near(0, 0, 400, 480);
	Memberships RFS_med(400, 480, 480, 520);
	Memberships RFS_far(480, 520, 1500, 1500);
	Memberships RBS_near(0, 0, 300, 380);
	Memberships RBS_med(300, 380, 380, 450);
	Memberships RBS_far(380, 450, 1500, 1500);
	Memberships RightEdge[] = { RFS_near, RFS_med, RFS_far, RBS_near, RBS_med, RBS_far };

	Memberships OAL_near(0, 0, 470, 530);
	Memberships OAL_med(470, 530, 530, 600);
	Memberships OAL_far(530, 600, 1500, 1500);
	Memberships OAR_near(0, 0, 470, 530);
	Memberships OAR_med(470, 530, 530, 600);
	Memberships OAR_far(530, 600, 1500, 1500);
	Memberships OA[] = { OAL_near, OAL_med, OAL_far, OAR_near, OAR_med, OAR_far };

	// Centroid
	short SLOW = 40, MED = 150, FAST = 225, SLOW_OA = 0, FAST_OA = 260;
	
	// Define the rule
	short RE_LMS[] = { SLOW, SLOW, SLOW, SLOW, FAST, MED, FAST, FAST, FAST };
	short RE_RMS[] = { FAST, FAST, FAST, FAST, FAST, FAST, MED, SLOW, SLOW };

	short OA_LMS[] = { SLOW_OA, MED, FAST_OA, SLOW_OA, MED, FAST_OA, SLOW_OA, MED, FAST_OA };
	short OA_RMS[] = { FAST_OA, SLOW_OA, SLOW_OA, FAST_OA, FAST_OA, MED, FAST_OA, FAST_OA, FAST_OA };
	
	// High level control fuzzifier
	Memberships RE_D(0, 530, 1600, 1800);
	Memberships OA_NEAR(0, 0, 430, 700);

	// Crispy input
	short x_RBS, x_RFS, x_OAR, x_OAL, x_minRE, x_minOA;
	// Firing sreingth
	double RE_firingsreingth[9], OA_firingsreingth[9];
	
	double sum_denom = 0, sum_lms = 0, sum_rms = 0;	
	// Results of defuzzification
	int RE_outL, RE_outR, OA_outL, OA_outR;
	// Crispy output
	int left, right;

	double laserRange[18], laserAngle[18];
	laser->lockDevice();
	for (int i = 0; i < 18; i++)
		laserRange[i] = laser->currentReadingPolar(10 * i - 90, 10 * (i + 1) - 90, &laserAngle[i]);
	laser->unlockDevice();

	// For monitor
	short count = 0;
	int status = 0;
	
	// Run 
	while (1)
	{
		for (int i = 0; i < 18; i++)
			laserRange[i] = laser->currentReadingPolar(10 * i - 90, 10 * (i + 1) - 90, &laserAngle[i]);

		x_RBS = laserRange[1];
		x_RFS = laserRange[4];
		x_OAR = laserRange[8];
		x_OAL = laserRange[10];
		
		x_minRE = min(min(laserRange[1], laserRange[2]), min(laserRange[3], laserRange[4]));
		x_minOA = min(min(laserRange[8], laserRange[9]), laserRange[10]);

		if (x_RFS > 1300) x_RFS = 1300;
		if (x_RBS > 1100) x_RBS = 1100;
		if (x_OAL > 1100) x_OAL = 1100;
		if (x_OAR > 1100) x_OAR = 1100;
		if (x_minRE > 1800) x_minRE = 1800;
		if (x_minOA > 1100) x_minOA = 1100;
		
		// Find the firing sreingth of the Right Edge Following. Non-zero results are wanted
		for (int rfs = 0; rfs < 3; rfs++)
			for(int rbs = 3; rbs < 6; rbs++)
				RE_firingsreingth[rfs * 3 + (rbs - 3)] = min(RightEdge[rfs].getValue(x_RFS), RightEdge[rbs].getValue(x_RBS));

		// Find the firing sreingth of the Obstacle Avoidance. Non-zero results are wanted
		for (int oal = 0; oal < 3; oal++)
			for(int oar = 3; oar < 6; oar++)
				OA_firingsreingth[oal * 3 + (oar - 3)] = min(OA[oal].getValue(x_OAL), OA[oar].getValue(x_OAR));

		// Defuzzification of Right Edge Following
		for (int i = 0; i < 9; i++)
			sum_denom += RE_firingsreingth[i];
		for (int j = 0; j < 9; j++)
		{
			sum_lms += RE_LMS[j] * RE_firingsreingth[j];
			sum_rms += RE_RMS[j] * RE_firingsreingth[j];
		}
		RE_outL = sum_lms / sum_denom;
		RE_outR = sum_rms / sum_denom;

		sum_denom = 0;
		sum_rms = 0;
		sum_lms = 0;
		// Defuzzification of Obstacle Avoidance
		for (int i = 0; i < 9; i++)
			sum_denom += OA_firingsreingth[i];
		for (int j = 0; j < 9; j++)
		{
			sum_lms += OA_LMS[j] * OA_firingsreingth[j];
			sum_rms += OA_RMS[j] * OA_firingsreingth[j];
		}
		OA_outL = sum_lms / sum_denom;
		OA_outR = sum_rms / sum_denom;
		
		// HLC
		status = 0;
		
		left = (OA_NEAR.getValue(x_minOA) * OA_outL + RE_D.getValue(x_minRE) * RE_outL) 
		/ (OA_NEAR.getValue(x_minOA) + RE_D.getValue(x_minRE));
		
		right = (OA_NEAR.getValue(x_minOA) * OA_outR + RE_D.getValue(x_minRE) * RE_outR)
		/ (OA_NEAR.getValue(x_minOA) + RE_D.getValue(x_minRE));
		
		if (OA_NEAR.getValue(x_minOA) == 1 && RE_D.getValue(x_minRE) == 1)
		{
			status = 2;
			left = OA_outL;
			right = OA_outR;
		} 
		else if (OA_NEAR.getValue(x_minOA) == 0 && RE_D.getValue(x_minRE) == 0)
		{
			status = 1;
			left = RE_outL;
			right = RE_outR;
		}
		
		// Set speed
		//if (left < 0) left = 0;
		//if (right < 0) right = 0;
		robot.setVel2(left, right);

		sum_denom = 0;
		sum_rms = 0;
		sum_lms = 0;
		
		// Monitor
		if (count > 300)
		{
			cout << "status: " << status << ", left: " << left << ", right: " << right <<endl;
			cout << "OA_outL: " << OA_outL << ", OA_outR: " << OA_outR << ", RE_outL: " << RE_outL << ", RE_outR: " << RE_outR << endl;
			cout << "Min Right: " << x_minRE << ", Min Obstacle: " << x_minOA << endl;
			cout << "-----------------------------------------------------" << endl;
			count = 0;
		}
		count++;
	}

	return 0;
}