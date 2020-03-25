#include"estimator.h"
#include"../MSG/message.hpp"




void Estimator::TaskMain(uint64_t timestamp) {
	bool imu_updated = false;
	bool gps_updated = false;
	bool mag_updated = false;
	bool baro_updated = false;


	message<sampleIMU_t> mesimu;
	message<sampleGPS_t> mesgps;
	message<sampleMag_t> mesmag;
	message<sampleBaro_t> mesbaro;

	message<controlState_t> mescon;

	//只有惯导数据更新了才运行
	if (mesimu.CheckUpdate(imu)) {
		mesimu.CopyTo(imu);
		Eigen::Vector3d gyro_inter;
		Eigen::Vector3d acc_inter;
		gyro_inter = imu.gyro*imu.intergral_time;
		acc_inter = imu.acc*imu.intergral_time;
		ekf_.setIMUData(timestamp, imu.intergral_time, imu.intergral_time, gyro_inter, acc_inter);

		if (mesgps.CheckUpdate(gps)) {
			mesgps.CopyTo(gps);
			ekf_.setGpsData(timestamp, gps);
		}
		if (mesmag.CheckUpdate(mag)) {
			mesmag.CopyTo(mag);
			magSum_ += mag.magStrenth;
			magCount_++;
			if (magCount_ >= magCountMin_) {
				magSum_ /=(double) magCount_;
				ekf_.setMagData(timestamp, magSum_);
				magSum_.setZero();
				magCount_ = 0;
			}
		}
		if (mesbaro.CheckUpdate(baro)) {
			mesbaro.CopyTo(baro);
			ekf_.setBaroData(timestamp,baro.altitude );
		}

		if (ekf_.update()) {
			Eigen::Vector4d vec;
			ekf_.get_state_delayed(state_);
			controlsta.timestamp = timestamp;
			vec = state_.head(4);
			controlsta.q = Quatd(vec);
			controlsta.localpos = state_.segment(7, 3);
			controlsta.euler = Eulerd(Dcm(controlsta.q));
			controlsta.vel = state_.segment(4, 3);
			mescon.Publish(controlsta);
		}
	}
}

Eigen::VectorXd Estimator::GetState() {
	return state_;
}