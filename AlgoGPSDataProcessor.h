/*************************************************
Copyright:chaoying
Author:Tong
Date:2018-09-29
Description:GPS data optimization
**************************************************/

#pragma once

//error list
#define ALGO_SUCCESS                                0 //success
#define ALGO_INVALID_ADDRESS            -1//input data arrays address are invalid
/*************************************************
Function:       AlgoGPSOptimizer
Description:    get rid of the noise in GPS data and smooth the GPS curve
Input:          @param logitude_src: an array, the longitude of GPS data
@param latitude_src: an array, the latitude of GPS data
@param length: the length of GPS data
@param acceler_x: an array, the speed of x direction
@param acceler_y: an array, the speed of y direction
@param acceler_z: an array, the speed of z direction
@param timeDiff: an array, the time difference of two adjacent points
Output:    @param logitude_dst: an array, the output of longitude after optimization
@param latitude_dst: an array, the output of latitude after optimization
Return: error code
Others:
*************************************************/
 int AlgoGPSOptimizer(double* logitude_src, double* latitude_src, int length, 
	double* acceler_x, double* acceler_y, double* acceler_z,
	double* timeDiff, double* logitude_dst, double* latitude_dst);
