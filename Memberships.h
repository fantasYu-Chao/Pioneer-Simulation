#ifndef MEMBERSHIPS_H
#define MEMBERSHIPS_H

#include<stdlib.h>
#include<iostream>

using namespace std;

class Memberships
{
private:
	int a;
	int b;
	int c;
	int d;

public:
	Memberships() {}; // constructor

	Memberships(int a, int b, int c, int d);

	double getValue(double z);
};

Memberships::Memberships(int a, int b, int c, int d)
{
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;
}

double Memberships::getValue(double z)
{
	double v = 0;

	if (z < a || z >d)
		v = 0;

	// Left shoulder
	if (a == b)
	{
		if (z <= c)
			v = 1;
		else
			v = (d - z) / (d - c);
	}
	// Right shoulder
	else if (c == d)
	{
		if (z >= b)
			v = 1;
		else
			v = (z - a) / (b - a);
	}
	// Trapezoid or Triangle
	else
	{
		if (z < b)
			v = (z - a) / (b - a);
		else if (z > c)
			v = (d - z) / (d - c);
		else
			v = 1;
	}
	return v;
}

#endif