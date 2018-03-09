#include "Wall_Following.h"

// defining membership shapes for 6 diff membership graphs
// all take shape of either trapezoid or triangle

// 
double Wall_Following::right_edge_right_low(double x)
{
	if (x <= 500)
	{return 1;}
	else if (x >= 650)
	{return 0;}
	else if (x>500 && x <650)
	// falling edge 
	{return((650 - x) / (650 - 500));}

}
double Wall_Following::right_edge_right_medium(double x)
{
	if (x <= 500)
	{return 0;}
	else if (x >= 900)
	{return 0;}
	else if (x >= 650 && x <= 750)
	{return 1;}
	else if (x>500 && x <650)
	// rising edge
	{return((x - 500) / (650 - 500));}
	else if (x>750 && x <900)
	// falling edge
	{return((900 - x) / (900 - 750));}
}
double Wall_Following::right_edge_right_high(double x)
{
	if (x >= 900)
	{return 1;}
	else if (x > 750 && x < 900)
	// rising edge
	{return((x - 750) / (900 - 750));}
	else if (x <= 750)
	{return 0;}
}

double Wall_Following::right_edge_front_low(double x)
{
	if (x <= 600)
	{return 1;}
	else if (x >= 750)
	{return 0;}
	else if (x>600 && x <750)
	// falling edge
	{return((750 - x) / (750 - 600));}

}
double Wall_Following::right_edge_front_medium(double x)
{
	if (x <= 600)
	{return 0;}
	else if (x >= 1000)
	{return 0;}
	else if (x >= 750 && x <= 850)
	{return 1;}
	else if (x>600 && x <750)
	// rising edge
	{return((x - 600) / (750 - 600));}
	else if (x>850 && x <1000)
	// falling edge
	{return((1000 - x) / (1000 - 850));}
}
double Wall_Following::right_edge_front_high(double x)
{
	if (x >= 1000)
	{return 1;}
	else if (x > 850 && x < 1000)
	// rising edge
	{return((x - 850) / (1000 - 850));}
	else if (x <= 850)
	{return 0;}
}

//rules
double Wall_Following::CC(double x, double y)
{
	return right_edge_right_low(x) * right_edge_front_low(y);
}
double Wall_Following::CM(double x, double y)
{
	return right_edge_right_low(x) * right_edge_front_medium(y);
}
double Wall_Following::CH(double x, double y)
{
	return right_edge_right_low(x) * right_edge_front_high(y);
}
double Wall_Following::MC(double x, double y)
{
	return right_edge_right_medium(x) * right_edge_front_low(y);
}
double Wall_Following::MM(double x, double y)
{
	return right_edge_right_medium(x) * right_edge_front_medium(y);
}
double Wall_Following::MH(double x, double y)
{
	return right_edge_right_medium(x) * right_edge_front_high(y);
}
double Wall_Following::HC(double x, double y)
{
	return right_edge_right_high(x) * right_edge_front_low(y);
}
double Wall_Following::HM(double x, double y)
{
	return right_edge_right_high(x) * right_edge_front_medium(y);
}
double Wall_Following::HH(double x, double y)
{
	return right_edge_right_high(x) * right_edge_front_high(y);
}

// Outputs to wheel speed
double Wall_Following::inference_left(double x, double y)
{
	return (CC(x, y)*low_speed + CM(x, y)*mid_speed + CH(x, y)*high_speed + MC(x, y)*low_speed + MM(x, y)*mid_speed + MH(x, y)*high_speed + HC(x, y)*low_speed + HM(x, y)*mid_speed + HH(x, y)*high_speed) / 
		(CC(x, y) + CM(x, y) + CH(x, y) + MC(x, y) + MM(x, y) + MH(x, y) + HC(x, y) + HM(x, y) + HH(x, y));
}
double Wall_Following::inference_right(double x, double y)
{
	return ((CC(x, y)*high_speed + CM(x, y)*high_speed + CH(x, y)*low_speed + MC(x, y)*mid_speed + MM(x, y)*mid_speed + MH(x, y)*mid_speed + HC(x, y)*high_speed + HM(x, y)*low_speed + HH(x, y)*low_speed) / 
		(CC(x, y) + CM(x, y) + CH(x, y) + MC(x, y) + MM(x, y) + MH(x, y) + HC(x, y) + HM(x, y) + HH(x, y)));
}
