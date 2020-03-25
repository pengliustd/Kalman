#pragma once
#include<fstream>
#include<vector>
#include<cstdint>

#include"../MSG/msgsensor.h"


using std::fstream;
using std::vector;


class sensorTxt{
public:
	sensorTxt() = default;
	bool run(uint64_t);

	void Initialize(fstream& imufile
		, fstream& gpsfile
		, fstream& magfile
		, fstream& barofile);

private:
	bool initialized_ = false;
	uint64_t imuTimeOffset_;
	bool imuTimeOffsetOk_=false;

	vector<sampleIMU_t> imu_;
	vector<sampleGPS_t> gps_;
	vector<sampleMag_t> mag_;
	vector<sampleBaro_t> baro_;

	vector<sampleIMU_t>::iterator itImu_;
	vector<sampleGPS_t>::iterator itGps_;
	vector<sampleMag_t>::iterator itMag_;
	vector<sampleBaro_t>::iterator itBaro_;

};