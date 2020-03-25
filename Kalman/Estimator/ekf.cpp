#include"ekf.h"

#include<fstream>
using namespace std;
fstream output("output/debug.txt",fstream::out);

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	state_.SetZero();

	deltaAngleCorr_.setZero();
	imuDownSampled_.SetZero();
	imuDownSampled_.time_us = timestamp;

	filterInitialised_ = false;

	dt_ekf_avg_ = 0.001 * (double)(FILTER_UPDATE_PERIOD_MS);

	return ret;
}

bool Ekf::update()
{
	if (!filterInitialised_) {
		filterInitialised_ = initialiseFilter();
		if (!filterInitialised_) {
			return false;
		}
	}
	// Only run the filter if IMU data in the buffer has been updated
	if (imuUpdated_) {
		// perform state and covariance prediction for the main filter
		predictState();
		predictCovariance();
		// control fusion of observation data
		controlFusionModes();
	}
	// the output observer always runs
	calculateOutputStates();
	// check for NaN or inf on attitude states
	if (!isfinite(state_.quat_nominal(0)) || !isfinite(outputNew_.quat_nominal(0))) {
		return false;
	}
	// We don't have valid data to output until tilt and yaw alignment is complete
	return true;
}

bool Ekf::initialiseFilter()
{
	// Keep accumulating measurements until we have a minimum of 10 samples for the required sensors

	// Sum the IMU delta angle measurements
	imuSample_t imu_init = imuBuffer_.GetNewest();
	delVelSum_ += imu_init.delta_vel;

	// Sum the magnetometer measurements
	if (magBuffer_.PopFirstOlderThan(imuSampleDelayed_.time_us, &magSampleDelayed_)) {
		if ((magCounter_ == 0) && (magSampleDelayed_.time_us != 0)) {
			// initialise the counter when we start getting data from the buffer
			magCounter_ = 1;
		}
		else if ((magCounter_ != 0) && (magSampleDelayed_.time_us != 0)) {
			// increment the sample count and apply a LPF to the measurement
			magCounter_++;
			// don't start using data until we can be certain all bad initial data has been flushed
			if (magCounter_ == obsBufferLength_ + 1) {
				// initialise filter states
				magFilterState_ = magSampleDelayed_.mag;
			}
			else if (magCounter_ > (uint8_t)(obsBufferLength_ + 1)) {
				// noise filter the data
				magFilterState_ = magFilterState_ * 0.9 + magSampleDelayed_.mag * 0.1;
			}
		}
	}

	// if the user parameter specifies use of GPS/range finder for height we use baro height initially and switch to GPS/range finder
	// later when it passes checks.
	if (baroBuffer_.PopFirstOlderThan(imuSampleDelayed_.time_us, &baroSampleDelayed_)) {
		if ((hgtCounter_ == 0) && (baroSampleDelayed_.time_us != 0)) {
			hgtCounter_ = 1;
		}
		else if ((hgtCounter_ != 0) && (baroSampleDelayed_.time_us != 0)) {
			// increment the sample count and apply a LPF to the measurement
			hgtCounter_++;

			// don't start using data until we can be certain all bad initial data has been flushed
			if (hgtCounter_ == obsBufferLength_ + 1) {
				// initialise filter states
				hgtBaroOffset_ = baroSampleDelayed_.hgt;
			}
			else if (hgtCounter_ > obsBufferLength_ + 1) {
				// noise filter the data
				hgtBaroOffset_ = 0.9f * hgtBaroOffset_ + 0.1* baroSampleDelayed_.hgt;
			}
		}
	}

	// check to see if we have enough measurements and return false if not
	bool hgt_count_fail = hgtCounter_ <= 2 * obsBufferLength_;
	bool mag_count_fail = magCounter_ <= 2 * obsBufferLength_;

	if (hgt_count_fail || mag_count_fail ) {
		return false;
	}
	else {
		// Zero all of the states
		state_.SetZero();

		state_.pos.head(2) = gpsSampleDelayed_.pos;
		state_.pos(2) = -baroSampleDelayed_.hgt;

		// get initial roll and pitch estimate from delta velocity vector, assuming vehicle is static
		double pitch = 0.0;
		double roll = 0.0;

		if (delVelSum_.norm() > 0.001) {
			delVelSum_.normalize();
			pitch = asin(delVelSum_(0));
			roll = atan2(-delVelSum_(1), -delVelSum_(2));
		}
		else {
			return false;
		}

		// calculate initial tilt alignment
		Eulerd euler_init(roll, pitch, 0.0);
		state_.quat_nominal =Quatd(euler_init);
		outputNew_.quat_nominal = state_.quat_nominal;

		// update transformation matrix from body to world frame
		R_ToEarth_ =Dcm(state_.quat_nominal);

		// calculate the averaged magnetometer reading
		Eigen::Vector3d mag_init = magFilterState_;

		// calculate the initial magnetic field and yaw alignment
		 resetMagHeading(mag_init);

		// initialise the state covariance matrix
		initialiseCovariance();

		// reset the essential fusion timeout counters
		timeLastPosFuse_us = timeLastImu_us_;
		timeLastVelFuse_us = timeLastImu_us_;
		timeLastHgtFuse_us = timeLastImu_us_;
		// reset the output predictor state history to match the EKF initial values
		alignOutputFilter();

		return true;
	}
}

void Ekf::predictState()
{
	if (!earthRateInitialised_) {
		if (NED_OriginInitialised_) {
			calcEarthRateNED(EarthRate_NED_, posRef_.lat_rad);
			earthRateInitialised_ = true;
		}
	}

	// apply imu bias corrections
	 correctedDeltaAng_ = imuSampleDelayed_.delta_ang - state_.gyro_bias;
	 correctedDeltaVel_ = imuSampleDelayed_.delta_vel - state_.accel_bias;

	// correct delta angles for earth rotation rate
	correctedDeltaAng_ -= -R_ToEarth_.transpose() * EarthRate_NED_ * imuSampleDelayed_.delta_ang_dt;

	// convert the delta angle to a delta quaternion
	Quatd dq;
	dq.from_axis_angle(correctedDeltaAng_);

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	state_.quat_nominal = dq * state_.quat_nominal;

	// quaternions must be normalised whenever they are modified
	state_.quat_nominal.normalize();

	// save the previous value of velocity so we can use trapzoidal integration
	Eigen::Vector3d vel_last = state_.vel;

	// update transformation matrix from body to world frame
	R_ToEarth_ = Dcm(state_.quat_nominal);

	// Calculate an earth frame delta velocity
	Eigen::Vector3d correctedDeltaVel__ef = R_ToEarth_ * correctedDeltaVel_;

	// calculate the increment in velocity using the current orientation
	state_.vel += correctedDeltaVel__ef;

	// compensate for acceleration due to gravity
	state_.vel(2) += gravity_ * imuSampleDelayed_.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	state_.pos += (vel_last + state_.vel) * imuSampleDelayed_.delta_vel_dt * 0.5;

	// calculate an average filter update time
	double input = 0.5 * (imuSampleDelayed_.delta_vel_dt + imuSampleDelayed_.delta_ang_dt);

	dt_ekf_avg_ = 0.99 * dt_ekf_avg_ + 0.01 * input;
}

void Ekf::predictCovariance() {
	double dtemp;
	Eigen::MatrixXd  mattemp_4(4,3);
	Eigen::MatrixXd mattemp;

	Eigen::MatrixXd kalman_F(kNumStates_, kNumStates_);
	Eigen::MatrixXd kalman_G(kNumStates_, 6);
	kalman_F.setIdentity();
	kalman_G.setZero();
	
	dtemp = params_.gyro_noise*dt_ekf_avg_;
	Eigen::Vector3d delta_ang_bias{ dtemp,dtemp,dtemp };
	dtemp = params_.accel_noise*dt_ekf_avg_;
	Eigen::Vector3d delta_vel_bias{ dtemp,dtemp,dtemp };

	//陀螺仪噪声修正
	correctedDeltaAng_ -= delta_ang_bias;
	correctedDeltaAng_ *= 0.5;
	//F(q_k->q_k+1)
	kalman_F.block(0, 0, 4, 4) <<
		1, -correctedDeltaAng_(0), -correctedDeltaAng_(1), -correctedDeltaAng_(2),
		correctedDeltaAng_(0), 1, correctedDeltaAng_(2), -correctedDeltaAng_(1),
		correctedDeltaAng_(1), -correctedDeltaAng_(2), 1, correctedDeltaAng_(0),
		correctedDeltaAng_(2), correctedDeltaAng_(1), -correctedDeltaAng_(0), 1;
	//F(delta_Angb_k->q_k+1)
	kalman_F.block(0, 10, 4, 3) <<
		state_.quat_nominal(1), state_.quat_nominal(2), state_.quat_nominal(3),
		-state_.quat_nominal(0), state_.quat_nominal(3), -state_.quat_nominal(2),
		-state_.quat_nominal(3), -state_.quat_nominal(0), state_.quat_nominal(1),
		state_.quat_nominal(2), -state_.quat_nominal(1), -state_.quat_nominal(0);
	kalman_F.block(0, 10, 4, 3) *= 0.5;
	//F(q_k->v_k+1)
	mattemp_4 <<
		state_.quat_nominal(0), -state_.quat_nominal(3), state_.quat_nominal(2),
		state_.quat_nominal(1), state_.quat_nominal(2), state_.quat_nominal(3),
		-state_.quat_nominal(2), state_.quat_nominal(1), state_.quat_nominal(0),
		-state_.quat_nominal(3), -state_.quat_nominal(0), state_.quat_nominal(1);
	kalman_F.block(4, 0, 1, 4) = (2.0 * mattemp_4*correctedDeltaVel_).transpose();
	mattemp_4 <<
		state_.quat_nominal(3), state_.quat_nominal(0), -state_.quat_nominal(1),
		state_.quat_nominal(2), -state_.quat_nominal(1), -state_.quat_nominal(0),
		state_.quat_nominal(1), state_.quat_nominal(2), state_.quat_nominal(3),
		state_.quat_nominal(0), -state_.quat_nominal(3), state_.quat_nominal(2);
	kalman_F.block(5, 0, 1, 4)=(2.0*mattemp_4*correctedDeltaVel_).transpose();
	mattemp_4 <<
		-state_.quat_nominal(2), state_.quat_nominal(1), state_.quat_nominal(0),
		state_.quat_nominal(3), state_.quat_nominal(0), -state_.quat_nominal(1),
		-state_.quat_nominal(0), state_.quat_nominal(3), -state_.quat_nominal(2),
		state_.quat_nominal(1), state_.quat_nominal(2), state_.quat_nominal(3);
	kalman_F.block(6, 0, 1, 4) = (2.0*mattemp_4*correctedDeltaVel_).transpose();
	//F(delta_vb_k->v_k+1)
	kalman_F.block(4, 13, 3, 3) = -R_ToEarth_;
	//F(v_k->p_k+1)
	kalman_F.block(7, 4, 3, 3) = Eigen::Matrix3d::Identity()*dt_ekf_avg_;

	//G(delta_vn_k->v_k+1)
	kalman_G.block(0, 0, 4, 3) <<
		state_.quat_nominal(1), state_.quat_nominal(2), state_.quat_nominal(3),
		-state_.quat_nominal(0), state_.quat_nominal(3), -state_.quat_nominal(2),
		-state_.quat_nominal(3), -state_.quat_nominal(0), state_.quat_nominal(1),
		state_.quat_nominal(2), -state_.quat_nominal(1), -state_.quat_nominal(0);
	kalman_G.block(0, 0, 4, 3) *= 0.5;
	//G(delta_vn_k->v_k+1)
	kalman_G.block(4,3,3,3)= -R_ToEarth_;

	mattemp = Eigen::MatrixXd::Zero(6, 6);
	mattemp.diagonal().head(3) = delta_ang_bias.array().square();
	mattemp.diagonal().tail(3) = delta_vel_bias.array().square();
	kalman_P_ = kalman_F * kalman_P_*kalman_F.transpose() + kalman_G * mattemp*kalman_G.transpose();

	//零偏误差的过程噪声
	dtemp = params_.gyro_bias_p_noise*dt_ekf_avg_*dt_ekf_avg_;
	dtemp = pow(dtemp, 2);
	Eigen::Vector3d delta_ang_p_bias{ dtemp,dtemp,dtemp };
	kalman_P_.diagonal().segment(10, 3) += delta_ang_p_bias;

	dtemp=params_.accel_bias_p_noise*dt_ekf_avg_*dt_ekf_avg_;
	dtemp = pow(dtemp, 2);
	Eigen::Vector3d delta_vel_p_bias{ dtemp,dtemp,dtemp };
	kalman_P_.diagonal().segment(13, 3) += delta_vel_p_bias;

}


void Ekf::calcEarthRateNED(Eigen::Vector3d &omega, double lat_rad) const
{
	omega(0) = earthRate_ * cos(lat_rad);
	omega(1) = 0.0;
	omega(2) = -earthRate_ * sin(lat_rad);
}


void Ekf::controlFusionModes()
{
	//计算磁偏角
	calcMagDeclination();
	// check for arrival of new sensor data at the fusion time horizon
	bool gps_data_ready = gpsBuffer_.PopFirstOlderThan(imuSampleDelayed_.time_us, &gpsSampleDelayed_);
	bool mag_data_ready = magBuffer_.PopFirstOlderThan(imuSampleDelayed_.time_us, &magSampleDelayed_);

	uint64_t delta_time_baro_us = baroSampleDelayed_.time_us;
	bool baro_data_ready = baroBuffer_.PopFirstOlderThan(imuSampleDelayed_.time_us, &baroSampleDelayed_);

	// if we have a new baro sample save the delta time between this sample and the last sample which is
	// used below for baro offset calculations
	if (baro_data_ready) {
		delta_time_baro_us_ = baroSampleDelayed_.time_us - delta_time_baro_us_;
	}

	if (mag_data_ready) {
		fuseHeading();
	}
	if (gps_data_ready) {
		fuseGps();
	}
	if (baro_data_ready) {
		fuseBaro();
	}
}

//计算磁偏角
void Ekf::calcMagDeclination()
{
	magDeclination_ = 0;

}

void Ekf::fuseHeading() {

	Eigen::MatrixXd kalman_H(1, kNumStates_);
	kalman_H.setZero();

	Eigen::MatrixXd kalman_K;

	//简化表达式
	Eigen::Vector3d mag = magSampleDelayed_.mag;

	Eigen::Vector3d  mag_earth_pred = R_ToEarth_ * mag;
	// the angle of the projection onto the horizontal gives the yaw angle
	double measured_hdg = -atan2(mag_earth_pred(1), mag_earth_pred(0)) + magDeclination_;


	//简化表达式
	double q0(state_.quat_nominal(0)), q1(state_.quat_nominal(1)),
		q2(state_.quat_nominal(2)), q3(state_.quat_nominal(3));

	//H(q->psi)
	Eigen::Vector3d den;
	double num;
	den(0) = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)*mag(0);
	den(0) += 2.0 * (q1*q2 - q0 * q3)*mag(1);
	den(0) += 2.0*(q0*q2 + q1 * q3)*mag(2);

	num = 2.0*(q0*q3 + q1 * q2)*mag(0);
	num += (q0*q0 - q1 * q1 + q2 * q2 - q3 * q3)*mag(1);
	num += 2.0*(q2*q3 - q0 * q1)*mag(2);

	den(1) = 1.0 + pow(num / den(0), 2);
	den(2) = pow(den(0), 2);

	kalman_H(0) = (q3 * mag(0) + q0 * mag(1) - q1 *mag(2))/den(0);
	kalman_H(0) -= (q0 * mag(0) - q3 * mag(1) + q2 * mag(2))*num / den(2);
	kalman_H(0) /= den(1);

	kalman_H(1) = (q2 * mag(0) - q1 * mag(1) - q0 * mag(2)) / den(0);
	kalman_H(1) -= (q0 * mag(0) + q2 * mag(1) + q3 * mag(2))*num / den(2);
	kalman_H(1) /= den(1);

	kalman_H(2) = (q1 * mag(0) + q2 * mag(1) + q3 * mag(2)) / den(0);
	kalman_H(2) -= (-q2 * mag(0) + q1 * mag(1) + q0 * mag(2))*num / den(2);
	kalman_H(2) /= den(1);

	kalman_H(3) = (q0 * mag(0) - q3 * mag(1) + q2 * mag(2)) / den(0);
	kalman_H(3) -= (-q3 * mag(0) - q0 * mag(1) + q1 * mag(2))*num / den(2);
	kalman_H(3) /= den(1);

	//计算测量噪声协方差
	double heading_innov_var(pow(params_.mag_heading_noise, 2));
	Eigen::MatrixXd kalman_R(1, 1);
	kalman_R(0, 0) = heading_innov_var;

	//计算卡尔曼滤波增益
	kalman_K = kalman_P_ * kalman_H.transpose();
	kalman_K *= (kalman_H*kalman_P_*kalman_H.transpose() + kalman_R).inverse();


	//更新状态
	Eigen::VectorXd kalman_X;
	ConvertStateToVector(kalman_X);
	Eigen::VectorXd kalman_Z(1);
	Eulerd euler(Dcm(state_.quat_nominal));
	kalman_Z(0) = measured_hdg - euler(2);
	if (kalman_Z(0) < -M_PI) {
		kalman_Z(0) += 2 * M_PI;
	}
	if (kalman_Z(0) > M_PI) {
		kalman_Z(0) -= 2 * M_PI;
	}
	kalman_X += kalman_K * kalman_Z;
	undateState(kalman_X);

	

	//更新协方差
	kalman_P_ = (Eigen::MatrixXd::Identity(kNumStates_, kNumStates_) - kalman_K * kalman_H)*kalman_P_;

}

void Ekf::fuseGps() {
	//不融合GPS的高度
	Eigen::MatrixXd kalman_H(5, kNumStates_);
	kalman_H.setZero();


	Eigen::MatrixXd kalman_K;

	kalman_H.block(0, 3, 3, 3).setIdentity();
	kalman_H.block(3, 7, 2, 2).setIdentity();

	//计算测量噪声协方差
	Eigen::MatrixXd kalman_R(5, 5);
	kalman_R.setZero();
	kalman_R(0, 0) = std::max(params_.gps_vel_noise, 0.01);
	kalman_R(0, 0) = std::max(gpsSampleDelayed_.sacc, kalman_R(0, 0));
	kalman_R(0, 0) = kalman_R(0, 0)*kalman_R(0, 0);
	kalman_R(1, 1) = kalman_R(0, 0);
	kalman_R(2, 2) = 2.25*kalman_R(1, 1);
	kalman_R(3, 3) = std::max(params_.gps_pos_noise, 0.01);
	kalman_R(3, 3) = std::max(gpsSampleDelayed_.hacc, kalman_R(3, 3));
	kalman_R(3, 3) = kalman_R(3, 3)*kalman_R(3, 3);
	kalman_R(4, 4) = kalman_R(3, 3);

	//计算卡尔曼滤波增益
	kalman_K = kalman_P_ * kalman_H.transpose();
	kalman_K *= (kalman_H*kalman_P_*kalman_H.transpose() + kalman_R).inverse();

	//更新状态
	Eigen::VectorXd kalman_X;
	ConvertStateToVector(kalman_X);
	Eigen::VectorXd kalman_Z(5);
	kalman_Z.head(3) = gpsSampleDelayed_.vel - state_.vel;
	kalman_Z.tail(2) = gpsSampleDelayed_.pos-state_.pos.head(2);//投影后的位置，即局部位置
	kalman_X += kalman_K * kalman_Z;
	undateState(kalman_X);

	//////
	static int count = 0;
	count++;
	//if (count == 63)
		output << state_.accel_bias(0)<<"\t"<< state_.accel_bias(1)<<"\t"<< state_.accel_bias(2) << endl;
	//更新协方差
	kalman_P_ = (Eigen::MatrixXd::Identity(kNumStates_, kNumStates_) - kalman_K * kalman_H)*kalman_P_;
}


void Ekf::fuseBaro() {
	Eigen::MatrixXd kalman_H(1, kNumStates_);
	kalman_H.setZero();

	Eigen::MatrixXd kalman_K;
	kalman_H(0, 9) = 1;
	//计算测量噪声协方差
	Eigen::MatrixXd kalman_R(1, 1);
	kalman_R(0, 0) = std::max(params_.baro_noise, 0.01);
	kalman_R(0, 0) = kalman_R(0, 0)*kalman_R(0, 0);

	//计算卡尔曼滤波增益
	kalman_K = kalman_P_ * kalman_H.transpose();
	kalman_K *= (kalman_H*kalman_P_*kalman_H.transpose() + kalman_R).inverse();

	//更新状态
	Eigen::VectorXd kalman_X;
	ConvertStateToVector(kalman_X);
	Eigen::VectorXd kalman_Z(1);
	kalman_Z(0) = -baroSampleDelayed_.hgt - state_.pos(2);//气压计的高度是海平面高度
	kalman_X += kalman_K * kalman_Z;
	undateState(kalman_X);

	//更新协方差
	kalman_P_ = (Eigen::MatrixXd::Identity(kNumStates_, kNumStates_) - kalman_K * kalman_H)*kalman_P_;

}

void Ekf::ConvertStateToVector(Eigen::VectorXd& x) {
	
	x.resize(kNumStates_);
	x.segment(0, 4) = state_.quat_nominal;
	x.segment(4, 3) = state_.vel;
	x.segment(7, 3) = state_.pos;
	x.segment(10, 3) = state_.gyro_bias;
	x.segment(13, 3) = state_.accel_bias;
}

void Ekf::undateState(Eigen::VectorXd& x) {
	Eigen::Vector4d vec = x.segment(0, 4);
	Quatd q(vec);
	q.Norm();
	state_.quat_nominal = q;
	state_.vel= x.segment(4, 3);
	state_.pos= x.segment(7, 3);
	state_.gyro_bias= x.segment(10, 3);
	state_.accel_bias= x.segment(13, 3);
}

double Ekf::constrain(double value, double lower, double upper) {
	double temp;
	if (value < lower)
		temp = lower;
	else if (value > upper)
		temp = upper;
	else
		temp = value;
	return temp;
}

//下采样修改了传入的原始数据，并将原始数据实时更新到表示最新数据的缓存当中
bool Ekf::collect_imu(imuSample_t &imu)
{
	// accumulate and downsample IMU data across a period FILTER_UPDATE_PERIOD_MS long

	// copy imu data to local variables
	imuSampleNew_.delta_ang = imu.delta_ang;
	imuSampleNew_.delta_vel = imu.delta_vel;
	imuSampleNew_.delta_ang_dt = imu.delta_ang_dt;
	imuSampleNew_.delta_vel_dt = imu.delta_vel_dt;
	imuSampleNew_.time_us = imu.time_us;

	// accumulate the time deltas
	imuDownSampled_.delta_ang_dt += imu.delta_ang_dt;
	imuDownSampled_.delta_vel_dt += imu.delta_vel_dt;

	// use a quaternion to accumulate delta angle data
	// this quaternion represents the rotation from the start to end of the accumulation period
	Quatd delta_q;
	delta_q.from_axis_angle(imu.delta_ang);
	q_DownSampled_ = q_DownSampled_ * delta_q;
	q_DownSampled_.normalize();

	// rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	Eigen::Matrix3d delta_R =Dcm(delta_q).transpose();
	imuDownSampled_.delta_vel = delta_R * imuDownSampled_.delta_vel;

	// accumulate the most recent delta velocity data at the updated rotation frame
	// assume effective sample time is halfway between the previous and current rotation frame
	imuDownSampled_.delta_vel += (imuSampleNew_.delta_vel + delta_R * imuSampleNew_.delta_vel) * 0.5;

	// if the target time delta between filter prediction steps has been exceeded
	// write the accumulated IMU data to the ring buffer
	double target_dt = (FILTER_UPDATE_PERIOD_MS) / 1000.0;

	if (imuDownSampled_.delta_ang_dt >= target_dt - imu_collection_time_adj_) {

		// accumulate the amount of time to advance the IMU collection time so that we meet the
		// average EKF update rate requirement
		imu_collection_time_adj_ += 0.01 * (imuDownSampled_.delta_ang_dt - target_dt);
		imu_collection_time_adj_ =constrain(imu_collection_time_adj_, -0.5 * target_dt, 0.5* target_dt);

		imu.delta_ang = q_DownSampled_.to_axis_angle();
		imu.delta_vel = imuDownSampled_.delta_vel;
		imu.delta_ang_dt = imuDownSampled_.delta_ang_dt;
		imu.delta_vel_dt = imuDownSampled_.delta_vel_dt;

		imuDownSampled_.delta_ang.setZero();
		imuDownSampled_.delta_vel.setZero();
		imuDownSampled_.delta_ang_dt = 0.0;
		imuDownSampled_.delta_vel_dt = 0.0;
		q_DownSampled_(0) = 1.0;
		q_DownSampled_(1) = q_DownSampled_(2) = q_DownSampled_(3) = 0.0;

		return true;
	}

	return false;
}

bool Ekf::collect_gps(uint64_t time_usec, sampleGPS_t& gps)
{
	// Run GPS checks whenever the WGS-84 origin is not set or the vehicle is not using GPS
	// Also run checks if the vehicle is on-ground as the check data can be used by vehicle pre-flight checks
	if (!NED_OriginInitialised_) {
		// If we have good GPS data set the origin's WGS-84 position to the last gps fix
		double lat = gps.pos(0);
		double lon = gps.pos(1);
		map_projection_init_timestamped(&posRef_, lat, lon, timeLastImu_us_);

		NED_OriginInitialised_ = true;
	}

	return NED_OriginInitialised_;
}

bool Ekf::resetMagHeading(Eigen::Vector3d &mag_init)
{
	// rotate the magnetometer measurement into earth frame
	Eulerd euler(Dcm(state_.quat_nominal));

	// Set the yaw angle to zero and calculate the rotation matrix from body to earth frame
	euler(2) = 0.0f;
	Eigen::Matrix3d R_to_earth= Dcm(euler);

	// rotate the magnetometer measurements into earth frame using a zero yaw angle
	Eigen::Vector3d mag_earth_pred = R_to_earth * magSampleDelayed_.mag;
	// the angle of the projection onto the horizontal gives the yaw angle
	euler(2) = -atan2(mag_earth_pred(1), mag_earth_pred(0)) + magDeclination_;


	// calculate initial quaternion states for the ekf
	// we don't change the output attitude to avoid jumps
	state_.quat_nominal = Quatd(euler);

	// update transformation matrix from body to world frame
	R_ToEarth_ = Dcm(state_.quat_nominal);

	return true;
}

void Ekf::alignOutputFilter()
{
	// calculate the quaternion delta between the output and EKF quaternions at the EKF fusion time horizon
	Quatd quat_inv = state_.quat_nominal.Inversed();
	Quatd q_delta = outputSampleDelayed_.quat_nominal * quat_inv;
	q_delta.normalize();

	// calculate the velocity and posiiton deltas between the output and EKF at the EKF fusion time horizon
	Eigen::Vector3d vel_delta = state_.vel - outputSampleDelayed_.vel;
	Eigen::Vector3d pos_delta = state_.pos - outputSampleDelayed_.pos;

	// loop through the output filter state history and add the deltas
	outputSample outputstate_s = {};
	uint64_t output_length = outputBuffer_.GetLength();

	for (uint64_t i = 0; i < output_length; i++) {
		outputstate_s = outputBuffer_.GetFromIndex(i);
		outputstate_s.quat_nominal *= q_delta;
		outputstate_s.quat_nominal.normalize();
		outputstate_s.vel += vel_delta;
		outputstate_s.pos += pos_delta;
		outputBuffer_.PushToIndex(i, outputstate_s);
	}
}

void Ekf::initialiseCovariance()
{
	kalman_P_.setZero();

	// calculate average prediction time step in sec
	double dt = 0.001*FILTER_UPDATE_PERIOD_MS;

	// define the initial angle uncertainty as variances for a rotation vector
	Eigen::Vector3d rot_vec_var;
	rot_vec_var(2) = rot_vec_var(1) = rot_vec_var(0) = pow(params_.initial_tilt_err,2);

	// update the quaternion state covariances
	initialiseQuatCovariances(rot_vec_var);

	// velocity
	kalman_P_(4,4) = pow(std::max(params_.gps_vel_noise, 0.01),2);
	kalman_P_(5, 5)= kalman_P_(4, 4);
	kalman_P_(6, 6) = 2.25*  kalman_P_(4, 4);

	// position
	kalman_P_(7, 7) = pow(std::max(params_.gps_pos_noise, 0.01),2);
	kalman_P_(8, 8) = kalman_P_(7, 7);
	kalman_P_(9, 9) = pow(std::max(params_.baro_noise, 0.01),2);


	// gyro bias
	kalman_P_(10, 10) = pow(params_.switch_on_gyro_bias * dt,2);
	kalman_P_(11, 11) = kalman_P_(10, 10);
	kalman_P_(12, 12) = kalman_P_(10, 10);

	// accel bias
	kalman_P_(13, 13) =pow(params_.switch_on_accel_bias * dt,2);
	kalman_P_(14, 14) = kalman_P_(13, 13);
	kalman_P_(15, 15) = kalman_P_(13, 13);

	// variances for optional states

	// earth frame and body frame magnetic field
	// set to observation variance
	/*for (uint8_t index = 16; index <= 21; index++) {
		kalman_P_(index, index) =pow(params_.mag_noise,2);
	}*/

}

void Ekf::initialiseQuatCovariances(Eigen::Vector3d & delta_angle) {
	Eigen::MatrixXd q_F(4,3);
	q_F <<
		state_.quat_nominal(1), state_.quat_nominal(2), state_.quat_nominal(3),
		-state_.quat_nominal(0), state_.quat_nominal(3), -state_.quat_nominal(2),
		-state_.quat_nominal(3), -state_.quat_nominal(0), state_.quat_nominal(1),
		state_.quat_nominal(2), -state_.quat_nominal(1), -state_.quat_nominal(0);
	q_F *= 0.5;

	Eigen::Matrix3d euler_cov;
	euler_cov.setZero();
	euler_cov.diagonal() = delta_angle;

	kalman_P_.block(0, 0, 4, 4) = q_F * euler_cov*q_F.transpose();
}

void Ekf::calculateOutputStates()
{
	// Use full rate IMU data at the current time horizon
	imuSample_t imu_new = imuSampleNew_;

	// correct delta angles for bias offsets
	Eigen::Vector3d delta_angle;
	double dt_scale_correction = dtImuAvg_ / dt_ekf_avg_;
	delta_angle = imuSampleNew_.delta_ang - state_.gyro_bias*dt_scale_correction;

	// correct delta velocity for bias offsets
	Eigen::Vector3d delta_vel = imuSampleNew_.delta_vel - state_.accel_bias * dt_scale_correction;

	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	delta_angle += deltaAngleCorr_;

	// convert the delta angle to an equivalent delta quaternions
	Quatd dq;
	dq.from_axis_angle(delta_angle);

	// rotate the previous INS quaternion by the delta quaternions
	outputNew_.time_us = imu_new.time_us;
	outputNew_.quat_nominal = dq * outputNew_.quat_nominal;

	// the quaternions must always be normalised afer modification
	outputNew_.quat_nominal.normalize();

	// calculate the rotation matrix from body to earth frame
	R_ToEarthNow_ = Dcm(outputNew_.quat_nominal);

	// rotate the delta velocity to earth frame
	Eigen::Vector3d delta_vel_NED = R_ToEarthNow_ * delta_vel;

	// corrrect for measured accceleration due to gravity
	delta_vel_NED(2) += gravity_ * imu_new.delta_vel_dt;

	// save the previous velocity so we can use trapezidal integration
	Eigen::Vector3d vel_last = outputNew_.vel;

	// increment the INS velocity states by the measurement plus corrections
	// do the same for vertical state used by alternative correction algorithm
	outputNew_.vel += delta_vel_NED;

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	Eigen::Vector3d delta_pos_NED = (outputNew_.vel + vel_last) * (imu_new.delta_vel_dt * 0.5);
	outputNew_.pos += delta_pos_NED;

	// calculate the average angular rate across the last IMU update
	Eigen::Vector3d ang_rate = imuSampleNew_.delta_ang * (1.0f / imuSampleNew_.delta_ang_dt);


	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	if (imuUpdated_) {
		outputBuffer_.Push(outputNew_);
		imuUpdated_ = false;

		// get the oldest INS state data from the ring buffer
		// this data will be at the EKF fusion time horizon
		outputSampleDelayed_ = outputBuffer_.GetOldest();
		

		// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
		Quatd quat_inv = state_.quat_nominal.Inversed();
		Quatd q_error = outputSampleDelayed_.quat_nominal * quat_inv;
		q_error.normalize();

		// convert the quaternion delta to a delta angle
		Eigen::Vector3d delta_ang_error;
		double scalar;

		if (q_error(0) >= 0.0) {
			scalar = -2.0;
		}
		else {
			scalar = 2.0;
		}

		delta_ang_error(0) = scalar * q_error(1);
		delta_ang_error(1) = scalar * q_error(2);
		delta_ang_error(2) = scalar * q_error(3);

		// calculate a gain that provides tight tracking of the estimator attitude states and
		// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
		double time_delay = 1e-6 * (double)(imuSampleNew_.time_us - imuSampleDelayed_.time_us);
		time_delay = std::max(time_delay, dtImuAvg_);
		double att_gain = 0.5 * dtImuAvg_ / time_delay;

		// calculate a corrrection to the delta angle
		// that will cause the INS to track the EKF quaternions
		deltaAngleCorr_ = delta_ang_error * att_gain;

		// calculate velocity and position tracking errors
		Eigen::Vector3d vel_err = (state_.vel - outputSampleDelayed_.vel);
		Eigen::Vector3d pos_err = (state_.pos - outputSampleDelayed_.pos);

		// collect magnitude tracking error for diagnostics
		outputTrackingError_(0) = delta_ang_error.norm();
		outputTrackingError_(1) = vel_err.norm();
		outputTrackingError_(2) = pos_err.norm();

		/*
		* Loop through the output filter state history and apply the corrections to the velocity and position states.
		* This method is too expensive to use for the attitude states due to the quaternion operations required
		* but becasue it eliminates the time delay in the 'correction loop' it allows higher tracking gains
		* to be used and reduces tracking error relative to EKF states.
		*/

		// Complementary filter gains
		double vel_gain = dt_ekf_avg_ / constrain(params_.vel_Tau, dt_ekf_avg_, 10.0);
		double pos_gain = dt_ekf_avg_ / constrain(params_.pos_Tau, dt_ekf_avg_, 10.0);

		{
			/*
			* Calculate corrections to be applied to vel and pos output state history.
			* The vel and pos state history are corrected individually so they track the EKF states at
			* the fusion time horizon. This option provides the most accurate tracking of EKF states.
			*/

			// calculate a velocity correction that will be applied to the output state history
			velErrInteg_ += vel_err;
			Eigen::Vector3d vel_correction = vel_err * vel_gain + velErrInteg_ * pow(vel_gain,2) * 0.1;

			// calculate a position correction that will be applied to the output state history
			posErrInteg_ += pos_err;
			Eigen::Vector3d pos_correction = pos_err * pos_gain + posErrInteg_ * pow(pos_gain,2) * 0.1;

			// loop through the output filter state history and apply the corrections to the velocity and position states
			outputSample outputstate_s;
			uint64_t max_index = outputBuffer_.GetLength() - 1;
			for (uint64_t index = 0; index <= max_index; index++) {
				outputstate_s = outputBuffer_.GetFromIndex(index);

				// a constant  velocity correction is applied
				outputstate_s.vel += vel_correction;

				// a constant position correction is applied
				outputstate_s.pos += pos_correction;

				// push the updated data to the buffer
				outputBuffer_.PushToIndex(index, outputstate_s);
			}
			// update output state to corrected values
			outputNew_ = outputBuffer_.GetNewest();
		}
	}
}


void Ekf::get_state_delayed(Eigen::VectorXd& state) {
	ConvertStateToVector(state);
}


void Ekf::get_covariances(Eigen::MatrixXd& covariances) {

	covariances = kalman_P_;
}

void Ekf::get_imu_vibe_metrics(Eigen::Vector3d & vibe) {
	vibe = vibeMetrics_;
}