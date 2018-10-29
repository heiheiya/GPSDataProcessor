/*************************************************
Copyright:chaoying
Author:Tong
Date:2018-09-29
Description:Kalman filter implementation
**************************************************/
#pragma once
#include <opencv2/core/mat.hpp>

using namespace cv;

struct KFStruct
{
	//Mat controlMatB; //control matrix B£¬not used
	//Mat controlVecU; //control vector U£¬not used
	Mat initStateX;           //initial state x
	Mat stateTransA;        //transition matrix A
	Mat kalmanGainK;     //kalman gain K
	Mat currentValueZ;   //current state z
	Mat estCovPrioriP;    //post prediction covariance matrix P
	Mat processNoiseCovQ; //prediction noise covariance matrixQ
	Mat measurementH;  //measurement matrix H;
	Mat measurementNoiseCovR; //measurement noise covariance matrixR
};

class AlgoKalman
{
public:
	AlgoKalman();
	~AlgoKalman();

public:
	/*************************************************
	Function:       initKalman
	Description:    initialize kalman filter
	Input:			@param x0 the first point's longitude
	@param y0  the first point's latitude
	@param vx the first point's speed at longitude
	@param vy the first point's speed at latitude
	*************************************************/
	void initKalman(double x0, double y0, double vx, double vy);

	/*************************************************
	Function:       setCurrentValue
	Description:    set current measurement point to kalman filter
	Input:			@param x the current measurement point's longitude
	@param y  the current measurement point's latitude
	*************************************************/
	void setCurrentValue(double x, double y);

	/*************************************************
	Function:       update
	Description:    correct current measurement point
	*************************************************/
	void update();

	/*************************************************
	Function:       getCorrectValue
	Description:    get update measurement point
	Return:			the correct measurement point
	*************************************************/
	void getCorrectValue(double** longitude, double** latitude, const int index);

private:
	KFStruct m_KF;
};
