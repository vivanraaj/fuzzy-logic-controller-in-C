// for obstacle avoidance

#pragma once

class Obstacle_Avoidance
{
public:
	// 3 inputs which has 3 membership values each (low , medium & high)
	double obstacle_left_low(double x);
	double obstacle_left_mid(double x);
	double obstacle_left_high(double x);

	double obstacle_front_low(double x);
	double obstacle_front_mid(double x);
	double obstacle_front_high(double x);

	double obstacle_right_low(double x);
	double obstacle_right_mid(double x);
	double obstacle_right_high(double x);

	//rules
	// 3 ^ 3 = 27 rules
	double CCC(double x, double y, double z);
	double CCM(double x, double y, double z);
	double CCH(double x, double y, double z);
	double CMC(double x, double y, double z);
	double CMM(double x, double y, double z);
	double CMH(double x, double y, double z);
	double CHC(double x, double y, double z);
	double CHM(double x, double y, double z);
	double CHH(double x, double y, double z);
	double MCC(double x, double y, double z);
	double MCM(double x, double y, double z);
	double MCH(double x, double y, double z);
	double MMC(double x, double y, double z);
	double MMM(double x, double y, double z);
	double MMH(double x, double y, double z);
	double MHC(double x, double y, double z);
	double MHM(double x, double y, double z);
	double MHH(double x, double y, double z);
	double HCC(double x, double y, double z);
	double HCM(double x, double y, double z);
	double HCH(double x, double y, double z);
	double HMC(double x, double y, double z);
	double HMM(double x, double y, double z);
	double HMH(double x, double y, double z);
	double HHC(double x, double y, double z);
	double HHM(double x, double y, double z);
	double HHH(double x, double y, double z);

	//Outputs
	double inference_left(double x, double y, double z);
	double inference_right(double x, double y, double z);


private:
	// output speed as singleton value
	double low_speed = 50;
	double mid_speed = 100;
	double high_speed = 150;
};


