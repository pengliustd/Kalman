#include"sensor_from_txt.h"
#include"../MSG/message.hpp"
#include<string>
#include<sstream>


using std::string;
using std::istringstream;

void sensorTxt::Initialize(fstream& imufile, fstream& gpsfile, fstream& magfile, fstream& barofile){
	string str;
	double dtemp;
	sampleIMU_t imu;
	sampleGPS_t gps;
	sampleMag_t mag;
	sampleBaro_t baro;

	imu_.clear();
	gps_.clear();
	mag_.clear();
	baro_.clear();

	while (getline(imufile, str)) {
		istringstream imustream(str);
		imustream >> imu.timestamp >>
			imu.gyro(0) >> imu.gyro(1) >> imu.gyro(2)
			>> dtemp
			>> imu.acc(0) >> imu.acc(1) >> imu.acc(2) >> dtemp;
		if (!imuTimeOffsetOk_) {
			imuTimeOffset_ = imu.timestamp;
			imuTimeOffsetOk_ = true;
		}
		else {
			imu.timestamp -= imuTimeOffset_;
			imu_.push_back(imu);
		}
	}
	while (getline(gpsfile, str)) {
		istringstream gpsstream(str);
		gpsstream >> gps.timestamp >>
			gps.pos(0) >> gps.pos(1) >> gps.pos(2)
			>> gps.s_variance_m_s >> gps.eph >> gps.epv
			>> gps.vel(0) >> gps.vel(1) >> gps.vel(2);
		gps.pos(0) /= 1e7;
		gps.pos(1) /= 1e7;
		gps.pos(2) /= 1e3;
		gps.timestamp -= imuTimeOffset_;
		if (gps.timestamp>0) {
			gps_.push_back(gps);
		}
	}
	while (getline(magfile, str)) {
		istringstream magstream(str);
		magstream >> mag.timestamp >>
			mag.magStrenth(0) >> mag.magStrenth(1) >> mag.magStrenth(2) >> dtemp;
		mag.timestamp -= imuTimeOffset_;
		if (mag.timestamp > 0) {
			mag_.push_back(mag);
		}
	}
	while (getline(barofile, str)) {
		istringstream barostream(str);
		barostream >> baro.timestamp >> baro.altitude;
		baro.altitude /= 1e3;
		baro.timestamp-= imuTimeOffset_;
		if (baro.timestamp > 0) {
			baro_.push_back(baro);
		}
	}

	if (!imu_.empty()) {
		itImu_ = imu_.begin();
	}
	if (!gps_.empty()) {
		itGps_ = gps_.begin();
	}
	if (!mag_.empty()) {
		itMag_ = mag_.begin();
	}
	if (!baro_.empty()) {
		itBaro_ = baro_.begin();
	}

	if(!imu_.empty())
	initialized_ = true;

}


bool sensorTxt::run(uint64_t timestamp) {
	if (!initialized_)
		return false;

	//只有在有IMU更新时，才更新其他传感器的数据
	if (itImu_ != imu_.end()&&itImu_->timestamp <= timestamp) {
		message<sampleIMU_t> mesimu;
		mesimu.Publish(*itImu_);
		itImu_++;

		if (!gps_.empty()&& itGps_ != gps_.end())
		if (itGps_->timestamp <= timestamp&&itGps_!=gps_.end()) {
			message<sampleGPS_t> mesgps;
			mesgps.Publish(*itGps_);
			itGps_++;
		}
		if (!mag_.empty()&& itMag_ != mag_.end())
		if (itMag_->timestamp <= timestamp) {
			message<sampleMag_t> mesmag;
			mesmag.Publish(*itMag_);
			itMag_++;
		}
		if (!baro_.empty() && itBaro_ != baro_.end())
		if (itBaro_->timestamp <= timestamp) {
			message<sampleBaro_t> mesbaro;
			mesbaro.Publish(*itBaro_);
			itBaro_++;
		}
	}
	if (itImu_ == imu_.end()) {
		initialized_ = false;
	}
	return true;
}
