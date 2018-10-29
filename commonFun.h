#pragma once
#include <math.h>
using namespace std;

#define STATIC_UP_T 9.8
#define STATIC_DOWN_T 9.1
int accelerCpt(double* acceler_x, double* acceler_y, double* acceler_z, int length,double* mag/*,double* dir_xy,double* dir_xz*/)
{

	for (int i=0;i<length;i++)
	{
		mag[i] = sqrt(acceler_x[i] * acceler_x[i] + acceler_y[i] * acceler_y[i] + acceler_z[i] * acceler_z[i]);
		//dir_xy[i] = atan2(acceler_x[i], acceler_y[i]);
		//dir_xz[i] = atan2(acceler_x[i], acceler_z[i]);
	}

	return 1;
}



int staticProbabilityMeasure(double* acceler_x, double* acceler_y, double* acceler_z,int length,int* probability)
{

	if (probability==NULL)
	{
		return -1;
	}
	double* mag = new double[length];
	//double* dir_xy = new double[length];
	//double* dir_xz = new double[length];

	accelerCpt(acceler_x, acceler_y, acceler_z, length, mag/*, dir_xy, dir_xz*/);

	for (int i=0;i<length;i++)
	{
		if (i<3)
		{
			if (mag[i]>STATIC_DOWN_T && mag[i]<STATIC_UP_T)
			{
				probability[i] = 1;
			}
			else
			{
				probability[i] = 0;
			}
		}
		else
		{
			int count = 0;;
			for (int j=0;j<3;j++)
			{
				if (mag[i - 2 + j]>STATIC_DOWN_T && mag[i - 2 + j]<STATIC_UP_T)
				{
					count++;
				}
			}
			if (count>1)
			{
				probability[i] = 1;
			} 
			else
			{
				probability[i] = 0;
			}
		}
	}
	delete[] mag;
	//delete[] dir_xy;
	//delete[] dir_xz;
	return 0;
}
