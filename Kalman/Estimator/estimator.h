#pragma once
#include"ekf.h"
#include"../MSG/msgstate.h"

class Estimator {
public:

	void TaskMain(uint64_t timestamp);
	Eigen::VectorXd GetState();

private:
	Ekf ekf_;
	sampleIMU_t imu;
	sampleGPS_t gps;
	sampleMag_t mag;
	sampleBaro_t baro;

	controlState_t controlsta;

	Eigen::VectorXd state_;
	
	//磁力计积分用
	uint64_t magCountMin_;
	uint64_t magCount_;
	Eigen::Vector3d magSum_;
};