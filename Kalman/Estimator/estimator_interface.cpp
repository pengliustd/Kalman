
#include"estimator_interface.h"
#include<cmath>

void EstimatorInterface::setIMUData(uint64_t time_usec, double delta_ang_dt, double delta_vel_dt, 
	Eigen::Vector3d& delta_ang, Eigen::Vector3d& delta_vel)
{
	if (!initialised_) {
		init(time_usec);
		initialised_ = true;
	}
	double dt = (time_usec - timeLastImu_us_)*1e-6;
	dt = std::max(dt, 1.0e-4);
	dt =std::min(dt, 0.02);
	timeLastImu_us_ = time_usec;

	if (timeLastImu_us_ > 0) {
		dtImuAvg_ = 0.8 * dtImuAvg_ + 0.2 * dt;
	}

	// copy data
	imuSample_t imu_sample_new = {};
	imu_sample_new.delta_ang = delta_ang;
	imu_sample_new.delta_vel = delta_vel;

	imu_sample_new.delta_ang_dt = delta_ang_dt ;
	imu_sample_new.delta_vel_dt = delta_vel_dt;
	imu_sample_new.time_us = time_usec;
	imuTicks_++;

	// calculate a metric which indicates the amount of coning vibration
	Eigen::Vector3d temp;
	temp<< imu_sample_new.delta_ang.cross(deltaAngPrev_).norm(),
		(imu_sample_new.delta_ang - deltaAngPrev_).norm(),
		(imu_sample_new.delta_vel - deltaVelPrev_).norm();
	vibeMetrics_ = 0.99*vibeMetrics_ + 0.01*temp;

	deltaAngPrev_ = imu_sample_new.delta_ang;
	deltaVelPrev_ = imu_sample_new.delta_vel;
	

	// accumulate and down-sample imu data and push to the buffer when new downsampled data becomes available
	if (collect_imu(imu_sample_new)) {
		imuBuffer_.Push(imu_sample_new);
		imuTicks_ = 0;
		imuUpdated_ = true;

		// get the oldest data from the buffer
		imuSampleDelayed_ = imuBuffer_.GetOldest();

		// calculate the minimum interval between observations required to guarantee no loss of data
		// this will occur if data is overwritten before its time stamp falls behind the fusion time horizon
		minObsInterval_us_ = (imuSampleNew_.time_us - imuSampleDelayed_.time_us) / (obsBufferLength_ - 1);

	}
	else {
		imuUpdated_ = false;

	}
}


void EstimatorInterface::setGpsData(uint64_t time_usec, sampleGPS_t& gps)
{
	if (!initialised_) {
		return;
	}
	if (((time_usec - timeLastGps_us_) > minObsInterval_us_)) {
		gpsSample_t gps_sample_new = {};
		gps_sample_new.time_us = gps.timestamp - params_.gps_delay_ms * 1000;

		gps_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;
		timeLastGps_us_ = time_usec;

		gps_sample_new.time_us = std::max(gps_sample_new.time_us, imuSampleDelayed_.time_us);
		gps_sample_new.vel =gps.vel;
		gps_sample_new.hacc = gps.eph;
		gps_sample_new.vacc = gps.epv;
		gps_sample_new.hgt = gps.pos(2);

		// Only calculate the relative position if the WGS-84 location of the origin is set
		if (collect_gps(time_usec, gps)) {
			double lpos_x = 0.0;
			double lpos_y = 0.0;
			map_projection_project(&posRef_, gps.pos(0), gps.pos(1), &lpos_x, &lpos_y);
			gps_sample_new.pos(0) = lpos_x;
			gps_sample_new.pos(1) = lpos_y;

		}
		else {
			gps_sample_new.pos(0) = 0.0;
			gps_sample_new.pos(1) = 0.0;
		}
		gpsBuffer_.Push(gps_sample_new);
	}
}

void EstimatorInterface::setMagData(uint64_t time_usec, Eigen::Vector3d& mag)
{
	// limit data rate to prevent data being lost
	if (time_usec - timeLastMag_us_ > minObsInterval_us_) {
		magSample_t mag_sample_new = {};
		mag_sample_new.time_us = time_usec - params_.mag_delay_ms * 1000;
		mag_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;
		timeLastMag_us_ = time_usec;
		mag_sample_new.mag =mag;
		magBuffer_.Push(mag_sample_new);
	}
}

void EstimatorInterface::setBaroData(uint64_t time_usec, double data)
{
	if (!initialised_) {
		return;
	}
	// limit data rate to prevent data being lost
	if (time_usec - timeLastBaro_us_ > minObsInterval_us_) {
		baroSample_t baro_sample_new{};
		baro_sample_new.hgt = data;
		baro_sample_new.time_us = time_usec - params_.baro_delay_ms * 1000;
		baro_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;
		timeLastBaro_us_ = time_usec;
		baro_sample_new.time_us = std::max(baro_sample_new.time_us, imuSampleDelayed_.time_us);
		baroBuffer_.Push(baro_sample_new);
	}
}


//gpsSampleDelayed_,magSampleDelayed_,baroSampleDelayed_,outputSampleDelayed_
//imuSampleNew_,R_ToEarthNow_,posRef_,gpsPosPrev_,NED_OriginInitialised_
//没有重新初始化
bool EstimatorInterface::initialise_interface(uint64_t timestamp)
{
	// find the maximum time delay the buffers are required to handle
	uint64_t max_time_delay_ms = std::max(params_.mag_delay_ms, params_.gps_delay_ms);
	imuBufferLength_ = (max_time_delay_ms / FILTER_UPDATE_PERIOD_MS) + 1;
	uint64_t ekf_delay_ms = max_time_delay_ms + static_cast<uint64_t>(max_time_delay_ms * 0.5);
	obsBufferLength_ = (ekf_delay_ms / params_.sensor_interval_min_ms) + 1;
	obsBufferLength_ = std::min(obsBufferLength_, imuBufferLength_);

	minObsInterval_us_ = 0;
	deltaAngPrev_.setZero();
	deltaVelPrev_.setZero();
	vibeMetrics_.setZero();

	if (!(imuBuffer_.Allocate(imuBufferLength_) &&
		gpsBuffer_.Allocate(obsBufferLength_) &&
		magBuffer_.Allocate(obsBufferLength_) &&
		baroBuffer_.Allocate(obsBufferLength_) &&
		outputBuffer_.Allocate(imuBufferLength_) )) {
		unallocate_buffers();
		return false;
	}

	// zero the data in the observation buffers
	for (uint64_t index = 0; index < obsBufferLength_; index++) {
		gpsSample_t gps_sample_init = {};
		gpsBuffer_.Push(gps_sample_init);
		magSample_t mag_sample_init = {};
		magBuffer_.Push(mag_sample_init);
		baroSample baro_sample_init = {};
		baroBuffer_.Push(baro_sample_init);
	}

	// zero the data in the imu data and output observer state buffers
	for (int index = 0; index < imuBufferLength_; index++) {
		imuSample imu_sample_init = {};
		imuBuffer_.Push(imu_sample_init);
		outputSample output_sample_init = {};
		outputBuffer_.Push(output_sample_init);
	}

	dtImuAvg_ = 0.0f;

	imuSampleDelayed_.SetZero();
	imuSampleDelayed_.time_us = timestamp;

	imuTicks_ = 0;

	initialised_ = false;

	timeLastImu_us_ = 0;
	timeLastGps_us_ = 0;
	timeLastMag_us_ = 0;
	timeLastBaro_us_ = 0;

	outputNew_.SetZero();
	imuUpdated_ = false;
	NED_OriginInitialised_ = false;

	return true;
}

void EstimatorInterface::unallocate_buffers()
{
	imuBuffer_.Unallocate();
	gpsBuffer_.Unallocate();
	magBuffer_.Unallocate();
	baroBuffer_.Unallocate();
	outputBuffer_.Unallocate();
}