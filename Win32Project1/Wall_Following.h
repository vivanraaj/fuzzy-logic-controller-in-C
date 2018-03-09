// for wall following
// Wall_Following.h
#pragma once


class Wall_Following
{
public:
	// 2 inputs which has 3 membership values each (low , medium & high)
	double right_edge_right_low(double x);
	double right_edge_right_medium(double x);
	double right_edge_right_high(double x);

	double right_edge_front_low(double x);
	double right_edge_front_medium(double x);
	double right_edge_front_high(double x);

	//rules
	// 2 ^ 3 = 9 rules
	double CC(double x, double y);
	double CM(double x, double y);
	double CH(double x, double y);
	double MC(double x, double y);
	double MM(double x, double y);
	double MH(double x, double y);
	double HC(double x, double y);
	double HM(double x, double y);
	double HH(double x, double y);

	//Outputs
	double inference_left(double x, double y);
	double inference_right(double x, double y);

private:
	// output speed as singleton value
	double low_speed = 50;
	double mid_speed = 100;
	double high_speed = 150;
};