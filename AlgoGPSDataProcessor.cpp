#include <stdio.h>
#include "AlgoGPSDataProcessor.h"
#include "AlgoKalman.h"
#include "commonFun.h"

using namespace std;

int AlgoGPSOptimizer(double* logitude_src, double* latitude_src, int length, 
	double* acceler_x, double* acceler_y, double* acceler_z, 
	double* timeDiff, double* logitude_dst, double* latitude_dst)
{
	if (logitude_src == NULL || latitude_src == NULL || logitude_dst == NULL || latitude_src == NULL)
	{
		return ALGO_INVALID_ADDRESS;
	}
	
	if (acceler_x == NULL || acceler_y == NULL || acceler_z == NULL )
	{
		return ALGO_INVALID_ADDRESS;
	}
	
	if (timeDiff == NULL)
	{
		return ALGO_INVALID_ADDRESS;
	}
	
	if (length <= 0)
	{
		return ALGO_INVALID_LENGTH;
	}

	int* probability = new int[length];
	int err = staticProbabilityMeasure(acceler_x, acceler_y, acceler_z, length, probability);
	if (err != 0)
	{
		return ALGO_ACC_HANDLE_ERR;
	}

	AlgoKalman kalmanFilter;
	int i = 0;
	while (i < length)
	{
		if (i==0)
		{
			//initialize kalman filter
			kalmanFilter.initKalman(logitude_src[0], latitude_src[0], 0, 0);
		}
		else
		{	
			if (err == 0)
			{
				if (probability[i] == 1)
				{
					logitude_dst[i] = 0.0f;
					latitude_dst[i] = 0.0f;
					i++;
					continue;
				}
			}
			kalmanFilter.setCurrentValue(logitude_src[i], latitude_src[i]);
			kalmanFilter.update();
		}
		kalmanFilter.getCorrectValue(&logitude_dst, &latitude_dst, i);
		i++;
	}

	return ALGO_SUCCESS;
}
