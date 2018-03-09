
// Source.cpp

#include "Obstacle_avoidance.h"
#include "Wall_Following.h"
#include "PID.h";
#include <iostream>;
#include <fstream>;
#include <math.h>;
#include "aria.h";

using namespace std;

int main(int argc, char **argv) {

	ArSensorReading *sonarSensor[8];

	//**ROBOT SETUP & CONNECTION**
	Aria::init();
	ArRobot robot;
	ArPose pose;
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	// Initialize the pid variable. 
	// start with negative value
	PID pid;
	double init_Kp = 0.5;
	double init_Ki = 0;
	double init_Kd = 1.1;

	pid.Init(init_Kp, init_Ki, init_Kd);

	// init the classes for right wall following and obstacles
	Wall_Following right_edge;
	Obstacle_Avoidance obstacle;

	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		cout << "Robot Connected!" << endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	while (true)
	{
		// define sensor reading inputs
		double right_edge_front = 0;
		double right_edge_right = 0;
		double obstacle_right = 0;
		double obstacle_mid_right = 0;
		double obstacle_mid_left = 0;
		double obstacle_left = 0;
		double obstacle_mid = 0;
		double pid_right = 0;
		

		int sonarRange[8];
		for (int i = 0; i < 8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}

		// use 2 inputs for wall following
		right_edge_front = sonarRange[6];
		right_edge_right = sonarRange[7];

		// use 3 inputs for obstacle avoidance
		obstacle_right = sonarRange[5];
		obstacle_left = sonarRange[2];

		obstacle_mid_right = sonarRange[4];
		obstacle_mid_left = sonarRange[3];

		if (obstacle_right > 5000) obstacle_right = 5000;
		if (obstacle_left > 5000) obstacle_left = 5000;
		if (obstacle_mid_right > 5000) obstacle_mid_right = 5000;
		if (obstacle_mid_left > 5000) obstacle_mid_left = 5000;

		obstacle_mid = min(obstacle_mid_right, obstacle_mid_left);

		// use min of sensor 6 and 7 for PID
		pid_right = min(right_edge_right, right_edge_front);

		/*
		//////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////// PID CONTROL ///////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////////

		// defined distance for robot
		int const desired_distance = 425;
		// define base velocity
		int const base_velocity = 150;  

		cout << "right_edge_right: " << right_edge_right << std::endl;

		cout << "right_edge_front: " << right_edge_front << std::endl;

		cout << "pid_right: " << pid_right << std::endl;

		// calculate the error
		double cte = desired_distance - pid_right;

		// if cte is positive, u getting closer to the right edge. so need to increase left velocity
		cout << "cte" << cte << std::endl;

		pid.UpdateError(cte);
		double steer_value = pid.TotalError();

		int left_velocity = 0;
		int right_velocity = 0;


		left_velocity = base_velocity - steer_value;
		cout << "left_velocity: " << left_velocity << std::endl;
		right_velocity = base_velocity + steer_value;
		cout << "right_velocity: " << right_velocity << std::endl;

		if (left_velocity < 0) {
		left_velocity = base_velocity;
		}
		if (right_velocity < 0) {
		right_velocity = base_velocity;
		}

		robot.setVel2(left_velocity, right_velocity);

		ArUtil::sleep(100);
	
		*/
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////WALLFOLLOWING & OBSTACLE AVOIDANCE/////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		
		int left_obstacle_velocity = 0;
		int right_obstacle_velocity = 0;
		int left_wall_follow_velocity = 0;
		int right_wall_follow_velocity = 0;


		// outputs from crispy output
		left_wall_follow_velocity = right_edge.inference_left(right_edge_right, right_edge_front);
		right_wall_follow_velocity = right_edge.inference_right(right_edge_right, right_edge_front);
		left_obstacle_velocity = obstacle.inference_left(obstacle_left, obstacle_mid, obstacle_right);
		right_obstacle_velocity = obstacle.inference_right(obstacle_left, obstacle_mid, obstacle_right);
		
		//*/

		// Wall Following
		//robot.setVel2(left_wall_follow_velocity, right_wall_follow_velocity);

		// Obstacle Avoidance
		//robot.setVel2(left_obstacle_velocity, right_obstacle_velocity);

		//*/

		
		// Subsumption Architecure for Combination of Wall Following And Obstacle Avoidance
		if (obstacle_mid< 650)
		{
			robot.setVel2(left_obstacle_velocity, right_obstacle_velocity);
		}
		else
		{	
			robot.setVel2(left_wall_follow_velocity, right_wall_follow_velocity);
		}
		

	}
	// termination
	robot.lock();
	robot.stop();
	robot.unlock();
	Aria::exit();
	return 0;
}
