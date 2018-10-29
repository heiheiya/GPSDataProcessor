#include "AlgoKalman.h"

AlgoKalman::AlgoKalman()
{
	
}

AlgoKalman::~AlgoKalman()
{

}

void AlgoKalman::initKalman(double x0, double y0, double vx, double vy)
{
	m_KF.kalmanGainK = Mat(4, 2, CV_64F);
	m_KF.currentValueZ = Mat(2, 1, CV_64F);
	m_KF.estCovPrioriP = Mat::zeros(4, 4, CV_64F);
	m_KF.initStateX = (Mat_<double>(4, 1) << x0, y0, 0, 0);
	m_KF.stateTransA = (Mat_<double>(4, 4) << 1, 0, 2, 0, 0, 1, 0, 2, 0, 0, 1, 0, 0, 0, 0, 1);
	m_KF.processNoiseCovQ = Mat::diag(Mat::ones(4, 1, CV_64F) * 0.00001);
        //m_KF.processNoiseCovQ = Mat::eye(4, 4, CV_64F) * 0.00001;
	m_KF.measurementH = Mat::eye(2, 4, CV_64F);
	m_KF.measurementNoiseCovR = Mat::diag(Mat::ones(2, 1, CV_64F) * 10);
       // m_KF.measurementNoiseCovR = Mat::eye(2, 2, CV_64F) * 10;
}

void AlgoKalman::setCurrentValue(double x, double y)
{
	m_KF.currentValueZ = (Mat_<double>(2, 1) << x, y);
}

void AlgoKalman::update()
{
	//Xk=Ak * Xk-1
	Mat Xk = m_KF.stateTransA * m_KF.initStateX;

	//Pk=Ak*Pk-1*AkT+Qk
	Mat Pk = m_KF.stateTransA *m_KF.estCovPrioriP * m_KF.stateTransA.t() + m_KF.processNoiseCovQ;

	//K=HkPkHkT(HkPkHkT+Rk)-1
	Mat PHt = Pk *m_KF.measurementH.t();
	m_KF.kalmanGainK = PHt * (m_KF.measurementH * PHt + m_KF.measurementNoiseCovR).inv();

	//Xk'=Xk+K'(Zk-HkXk)
	m_KF.initStateX = Xk + m_KF.kalmanGainK * (m_KF.currentValueZ - m_KF.measurementH * Xk);

	//Pk'=Pk-K'HkPk
	m_KF.estCovPrioriP = Pk - m_KF.kalmanGainK * m_KF.measurementH * Pk;
}

void AlgoKalman::getCorrectValue(double** longitude, double** latitude, const int index)
{
	(*longitude)[index] = m_KF.initStateX.at<double>(0);
	(*latitude)[index] = m_KF.initStateX.at<double>(1);
}

