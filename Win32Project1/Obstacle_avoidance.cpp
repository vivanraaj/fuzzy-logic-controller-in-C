// for Obstacle Avoidance

#include "Obstacle_Avoidance.h"


// defining membership shapes for 6 diff membership graphs
// all take shape of either trapezoid or triangle
double Obstacle_Avoidance::obstacle_left_low(double x)
{
	if (x <= 500)
	{return 1;}
	else if (x >= 650)
	{return 0;}
	else if (x > 500 && x < 650)
	// falling edge
	{return((650 - x) / (650 - 500));}
}
double Obstacle_Avoidance::obstacle_left_mid(double x)
{
	if (x <= 500)
	{return 0;
	}
	else if (x >= 900)
	{return 0;}
	else if (x > 500 && x < 650)
	//rising edge
	{return((x - 500) / (650 - 500));}
	else if (x >= 650 && x <= 750)
	{return 1;}
	else if (x > 750 && x < 900)
	// falling edge
	{return((900 - x) / (900 - 750));}
}
double Obstacle_Avoidance::obstacle_left_high(double x)
{
	if (x <= 750)
	{return 0;}
	else if (x >= 5000)
	{return 1;}
	else if (x > 750 && x < 5000)
	// rising edge
	{return((x - 750) / (5000 - 750));}
}


// Membership function front mid sensor

double Obstacle_Avoidance::obstacle_front_low(double x)
{
	if (x <= 600)
	{return 1;}
	else if (x >= 750)
	{return 0;}
	else if (x > 600 && x < 750)
	// falling edge
	{return((750 - x) / (750 - 600));}
}

double Obstacle_Avoidance::obstacle_front_mid(double x)
{
	if (x <= 600)
	{return 0;}
	else if (x >= 1000)
	{return 0;}
	else if (x > 600 && x < 750)
	// rising edge
	{return((x - 600) / (750 - 600));}
	else if (x >= 750 && x <= 850)
	{return 1;}
	else if (x > 850 && x < 1000)
	// falling edge
	{return((1000 - x) / (1000 - 850));}
}

double Obstacle_Avoidance::obstacle_front_high(double x)
{
	if (x <= 850)
	{return 0;}
	else if (x >= 5000)
	{return 1;} 
	else if (x > 850 && x < 5000)
	// rising edge
	{return((x - 850) / (5000 - 850));}
}


double Obstacle_Avoidance::obstacle_right_low(double x)
{
	if (x <= 500)
	{return 1;}
	else if (x >= 650)
	{return 0;}
	else if (x > 500 && x < 650)
	{return((650 - x) / (650 - 500));}
}
double Obstacle_Avoidance::obstacle_right_mid(double x)
{
	if (x <= 500)
	{return 0;}
	else if (x >= 900)
	{return 0;}
	else if (x > 500 && x < 650)
	{return((x - 500) / (650 - 500));}
	else if (x >= 650 && x <= 750)
	{return 1;}
	else if (x > 750 && x < 900)
	{return((900 - x) / (900 - 750));}
}
double Obstacle_Avoidance::obstacle_right_high(double x)
{
	if (x <= 750)
	{return 0;}
	else if (x >= 5000)
	{return 1;}
	else if (x > 750 && x < 5000)
	{return((x - 750) / (5000 - 750));}
	
	/*
	else if (x >= 900)
	{return 1;}
	else if (x > 750 && x < 900)
	{return((x - 750) / (900 - 750));}
	*/

}


//rules

double Obstacle_Avoidance::CCC(double x, double y, double z)
{
	return obstacle_left_low(x)*obstacle_front_low(y)*obstacle_right_low(z);
}
double Obstacle_Avoidance::CCM(double x, double y, double z)
{
	return obstacle_left_low(x)*obstacle_front_low(y)*obstacle_right_mid(z);
}
double Obstacle_Avoidance::CCH(double x, double y, double z)
{
	return obstacle_left_low(x)*obstacle_front_low(y)*obstacle_right_high(z);
}
double Obstacle_Avoidance::CMC(double x, double y, double z)
{
	return obstacle_left_low(x)*obstacle_front_mid(y)*obstacle_right_low(z);
}
double Obstacle_Avoidance::CMM(double x, double y, double z)
{
	return obstacle_left_low(x)*obstacle_front_mid(y)*obstacle_right_mid(z);
}
double Obstacle_Avoidance::CMH(double x, double y, double z)
{
	return obstacle_left_low(x)*obstacle_front_mid(y)*obstacle_right_high(z);
}
double Obstacle_Avoidance::CHC(double x, double y, double z)
{
	return obstacle_left_low(x)*obstacle_front_high(y)*obstacle_right_low(z);
}
double Obstacle_Avoidance::CHM(double x, double y, double z)
{
	return obstacle_left_low(x)*obstacle_front_high(y)*obstacle_right_mid(z);
}
double Obstacle_Avoidance::CHH(double x, double y, double z)
{
	return obstacle_left_low(x)*obstacle_front_high(y)*obstacle_right_high(z);
}


double Obstacle_Avoidance::MCC(double x, double y, double z)
{
	return obstacle_left_mid(x)*obstacle_front_low(y)*obstacle_right_low(z);
}
double Obstacle_Avoidance::MCM(double x, double y, double z)
{
	return obstacle_left_mid(x)*obstacle_front_low(y)*obstacle_right_mid(z);
}
double Obstacle_Avoidance::MCH(double x, double y, double z)
{
	return obstacle_left_mid(x)*obstacle_front_low(y)*obstacle_right_high(z);
}
double Obstacle_Avoidance::MMC(double x, double y, double z)
{
	return obstacle_left_mid(x)*obstacle_front_mid(y)*obstacle_right_low(z);
}
double Obstacle_Avoidance::MMM(double x, double y, double z)
{
	return obstacle_left_mid(x)*obstacle_front_mid(y)*obstacle_right_mid(z);
}
double Obstacle_Avoidance::MMH(double x, double y, double z)
{
	return obstacle_left_mid(x)*obstacle_front_mid(y)*obstacle_right_high(z);
}
double Obstacle_Avoidance::MHC(double x, double y, double z)
{
	return obstacle_left_mid(x)*obstacle_front_high(y)*obstacle_right_low(z);
}
double Obstacle_Avoidance::MHM(double x, double y, double z)
{
	return obstacle_left_mid(x)*obstacle_front_high(y)*obstacle_right_mid(z);
}
double Obstacle_Avoidance::MHH(double x, double y, double z)
{
	return obstacle_left_mid(x)*obstacle_front_high(y)*obstacle_right_high(z);
}




double Obstacle_Avoidance::HCC(double x, double y, double z)
{
	return obstacle_left_high(x)*obstacle_front_low(y)*obstacle_right_low(z);
}
double Obstacle_Avoidance::HCM(double x, double y, double z)
{
	return obstacle_left_high(x)*obstacle_front_low(y)*obstacle_right_mid(z);
}
double Obstacle_Avoidance::HCH(double x, double y, double z)
{
	return obstacle_left_high(x)*obstacle_front_low(y)*obstacle_right_high(z);
}
double Obstacle_Avoidance::HMC(double x, double y, double z)
{
	return obstacle_left_high(x)*obstacle_front_mid(y)*obstacle_right_low(z);
}
double Obstacle_Avoidance::HMM(double x, double y, double z)
{
	return obstacle_left_high(x)*obstacle_front_mid(y)*obstacle_right_mid(z);
}
double Obstacle_Avoidance::HMH(double x, double y, double z)
{
	return obstacle_left_high(x)*obstacle_front_mid(y)*obstacle_right_high(z);
}
double Obstacle_Avoidance::HHC(double x, double y, double z)
{
	return obstacle_left_high(x)*obstacle_front_high(y)*obstacle_right_low(z);
}
double Obstacle_Avoidance::HHM(double x, double y, double z)
{
	return obstacle_left_high(x)*obstacle_front_high(y)*obstacle_right_mid(z);
}
double Obstacle_Avoidance::HHH(double x, double y, double z)
{
	return obstacle_left_high(x)*obstacle_front_high(y)*obstacle_right_high(z);
}


// Inference
// formula for outputs are 

double Obstacle_Avoidance::inference_left(double x, double y, double z)
{
	return (CCC(x, y, z)*low_speed + CCM(x, y, z)*high_speed + CCH(x, y, z)*high_speed + CMC(x, y, z)*low_speed + CMM(x, y, z)*high_speed + CMH(x, y, z)*mid_speed + CHC(x, y, z)*low_speed + CHM(x, y, z)*mid_speed + CHH(x, y, z)*mid_speed + MCC(x, y, z)*low_speed + MCM(x, y, z)*low_speed + MCH(x, y, z)*high_speed + MMC(x, y, z)*low_speed + MMM(x, y, z)*mid_speed + MMH(x, y, z)*mid_speed + MHC(x, y, z)*low_speed + MHM(x, y, z)*high_speed + MHH(x, y, z)*high_speed + HCC(x, y, z)*low_speed + HCM(x, y, z)*low_speed + HCH(x, y, z)*low_speed + HMC(x, y, z)*low_speed + HMM(x, y, z)*high_speed + HMH(x, y, z)*high_speed + HHC(x, y, z)*low_speed + HHM(x, y, z)*high_speed + HHH(x, y, z)*high_speed) / 
		(CCC(x, y, z) + CCM(x, y, z) + CCH(x, y, z) + CMC(x, y, z) + CMM(x, y, z) + CMH(x, y, z) + CHC(x, y, z) + CHM(x, y, z) + CHH(x, y, z) + MCC(x, y, z) + MCM(x, y, z) + MCH(x, y, z) + MMC(x, y, z) + MMM(x, y, z) + MMH(x, y, z) + MHC(x, y, z) + MHM(x, y, z) + MHH(x, y, z) + HCC(x, y, z) + HCM(x, y, z) + HCH(x, y, z) + HMC(x, y, z) + HMM(x, y, z) + HMH(x, y, z) + HHC(x, y, z) + HHM(x, y, z) + HHH(x, y, z));
}
double Obstacle_Avoidance::inference_right(double x, double y, double z)
{
	return (CCC(x, y, z)*high_speed + CCM(x, y, z)*low_speed + CCH(x, y, z)*low_speed + CMC(x, y, z)*high_speed + CMM(x, y, z)*low_speed + CMH(x, y, z)*low_speed + CHC(x, y, z)*high_speed + CHM(x, y, z)*low_speed + CHH(x, y, z)*low_speed + MCC(x, y, z)*high_speed + MCM(x, y, z)*high_speed + MCH(x, y, z)*low_speed + MMC(x, y, z)*mid_speed + MMM(x, y, z)*mid_speed + MMH(x, y, z)*mid_speed + MHC(x, y, z)*high_speed + MHM(x, y, z)*high_speed + MHH(x, y, z)*high_speed + HCC(x, y, z)*high_speed + HCM(x, y, z)*high_speed + HCH(x, y, z)*high_speed + HMC(x, y, z)*mid_speed + HMM(x, y, z)*high_speed + HMH(x, y, z)*high_speed + HHC(x, y, z)*mid_speed + HHM(x, y, z)*high_speed + HHH(x, y, z)*high_speed) / 
		(CCC(x, y, z) + CCM(x, y, z) + CCH(x, y, z) + CMC(x, y, z) + CMM(x, y, z) + CMH(x, y, z) + CHC(x, y, z) + CHM(x, y, z) + CHH(x, y, z) + MCC(x, y, z) + MCM(x, y, z) + MCH(x, y, z) + MMC(x, y, z) + MMM(x, y, z) + MMH(x, y, z) + MHC(x, y, z) + MHM(x, y, z) + MHH(x, y, z) + HCC(x, y, z) + HCM(x, y, z) + HCH(x, y, z) + HMC(x, y, z) + HMM(x, y, z) + HMH(x, y, z) + HHC(x, y, z) + HHM(x, y, z) + HHH(x, y, z));
}