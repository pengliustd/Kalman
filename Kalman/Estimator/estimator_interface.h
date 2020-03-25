#pragma once
#include<cstdint>
#include"../Eigen/Dense"

#include"../DSS_Math/ring_buffer.hpp"
#include"../DSS_Math/geo.h"
#include"../MSG/msgsensor.h"
#include"../DSS_Math/quaternion.hpp"

//四元数，欧拉角的默认构造函数会将其初始化,所以SetZero函数不会改变四元数和欧拉角

typedef struct parameters {
	// measurement source control
	uint64_t sensor_interval_min_ms{ 20 };		// minimum time of arrival difference between non IMU sensor updates. Sets the size of the observation buffers.

	uint64_t min_delay_ms{ 0 };			// used to a set a minimum buffer length independent of specified sensor delays
	uint64_t mag_delay_ms{ 0 };		// magnetometer measurement delay relative to the IMU (msec)
	uint64_t baro_delay_ms{ 0};		// barometer height measurement delay relative to the IMU (msec)
	uint64_t gps_delay_ms{ 110 };		// GPS measurement delay relative to the IMU (msec)
	uint64_t airspeed_delay_ms{ 100};	// airspeed measurement delay relative to the IMU (msec)
	
	// input noise
	double gyro_noise{ 1.5e-2 };		// IMU angular rate noise used for covariance prediction (rad/sec)
	double accel_noise{ 3.5e-1 };		// IMU acceleration noise use for covariance prediction (m/sec/sec)

										// process noise
	double gyro_bias_p_noise{ 1.0e-3 };	// process noise for IMU rate gyro bias prediction (rad/sec**2)
	double accel_bias_p_noise{ 6.0e-3 };	// process noise for IMU accelerometer bias prediction (m/sec**3)
	double mage_p_noise{ 1.0e-3 };		// process noise for earth magnetic field prediction (Guass/sec)
	double magb_p_noise{ 1.0e-4 };		// process noise for body magnetic field prediction (Guass/sec)
	double wind_vel_p_noise{ 1.0e-1 };		// process noise for wind velocity prediction (m/sec/sec)
	
										// initialization errors
	double switch_on_gyro_bias{ 0.1 };	// 1-sigma gyro bias uncertainty at switch on (rad/sec)
	double switch_on_accel_bias{ 0.2 };	// 1-sigma accelerometer bias uncertainty at switch on (m/s**2)
	double initial_tilt_err{ 0.1 };		// 1-sigma tilt error after initial alignment using gravity vector (rad)
	double initial_wind_uncertainty{ 1.0 };	// 1-sigma initial uncertainty in wind velocity (m/s)

											// position and velocity fusion
	double gps_vel_noise{ 5.0e-1 };		// observation noise for gps velocity fusion (m/sec)
	double gps_pos_noise{ 0.5 };		// observation noise for gps position fusion (m)
	double pos_noaid_noise{ 10.0 };		// observation noise for non-aiding position fusion (m)
	double baro_noise{ 2.0 };			// observation noise for barometric height fusion (m)
	double baro_innov_gate{ 5.0 };		// barometric height innovation consistency gate size (STD)
	double posNE_innov_gate{ 5.0 };		// GPS horizontal position innovation consistency gate size (STD)
	double vel_innov_gate{ 5.0 };		// GPS velocity innovation consistency gate size (STD)
	double hgt_reset_lim{ 0.0 };		// The maximum 1-sigma uncertainty in height that can be tolerated before the height state is reset (m)

	// magnetometer fusion
	double mag_heading_noise{ 3.0e-1 };	// measurement noise used for simple heading fusion (rad)
	double mag_noise{ 5.0e-2 };		// measurement noise used for 3-axis magnetoemeter fusion (Gauss)
	double mag_declination_deg{ 0.0 };	// magnetic declination (degrees)
	double heading_innov_gate{ 2.6 };		// heading fusion innovation consistency gate size (STD)
	double mag_innov_gate{ 3.0 };		// magnetometer fusion innovation consistency gate size (STD)
	uint64_t mag_declination_source{  };	// bitmask used to control the handling of declination data
	uint64_t mag_fusion_type{ 0 };		// integer used to specify the type of magnetometer fusion used
	double mag_acc_gate{ 0.5 };		// when in auto select mode, heading fusion will be used when manoeuvre accel is lower than this (m/s**2)
	double mag_yaw_rate_gate{ 0.25 };		// yaw rate threshold used by mode select logic (rad/sec)

											// airspeed fusion
	double tas_innov_gate{ 5.0 };		// True Airspeed Innovation consistency gate size in standard deciation
	double eas_noise{ 1.4 };			// EAS measurement noise standard deviation used for airspeed fusion [m/s]

	
	// these parameters control the strictness of GPS quality checks used to determine if the GPS is
	// good enough to set a local origin and commence aiding
	uint64_t gps_check_mask{ 21 };		// bitmask used to control which GPS quality checks are used
	double req_hacc{ 5.0 };			// maximum acceptable horizontal position error
	double req_vacc{ 8.0 };			// maximum acceptable vertical position error
	double req_sacc{ 1.0};			// maximum acceptable speed error
	uint64_t req_nsats{ 6 };			// minimum acceptable satellite count
	double req_gdop{ 2.0 };			// maximum acceptable geometric dilution of precision
	double req_hdrift{ 0.3 };			// maximum acceptable horizontal drift speed
	double req_vdrift{ 0.5 };			// maximum acceptable vertical drift speed

	// XYZ offset of sensors in body axes (m)
	Eigen::Vector3d imu_pos_body;			// xyz position of IMU in body frame (m)
	Eigen::Vector3d gps_pos_body;			// xyz position of the GPS antenna in body frame (m)
	Eigen::Vector3d rng_pos_body;			// xyz position of range sensor in body frame (m)
	Eigen::Vector3d flow_pos_body;			// xyz position of range sensor focal point in body frame (m)
	Eigen::Vector3d ev_pos_body;			// xyz position of VI-sensor focal point in body frame (m)

	// output complementary filter tuning
	double vel_Tau{ 0.25 };			// velocity state correction time constant (1/sec)
	double pos_Tau{ 0.25 };			// postion state correction time constant (1/sec)

	// accel bias learning control
	double acc_bias_lim{ 0.4 };		// maximum accel bias magnitude (m/s/s)
	double acc_bias_learn_acc_lim{ 25.0 };	// learning is disabled if the magnitude of the IMU acceleration vector is greater than this (m/sec**2)
	double acc_bias_learn_gyr_lim{ 3.0 };	// learning is disabled if the magnitude of the IMU angular rate vector is greater than this (rad/sec)
	double acc_bias_learn_tc{ 0.5 };		// time constant used to control the decaying envelope filters applied to the accel and gyro magnitudes (sec)

	unsigned no_aid_timeout_max{ 1000000 };	// maximum lapsed time from last fusion of measurements that constrain drift before
											// the EKF will report that it is dead-reckoning (usec)
}parameters_t;

typedef struct stateSample {
	void SetZero() {
		vel.setZero();
		pos.setZero();
		gyro_bias.setZero();
		accel_bias.setZero();
	}
	Quatd  quat_nominal; // quaternion defining the rotaton from earth to body frame
	Eigen::Vector3d    vel;	// NED velocity in earth frame in m/s
	Eigen::Vector3d    pos;	// NED position in earth frame in m
	Eigen::Vector3d    gyro_bias;	// delta angle bias estimate in rad
	Eigen::Vector3d    accel_bias;	// delta velocity bias estimate in m/s
}stateSample_t;

typedef struct outputSample {
	void SetZero() {
		vel.setZero();
		pos.setZero();
		time_us = 0;
	}
	Quatd  quat_nominal;	// nominal quaternion describing vehicle attitude
	Eigen::Vector3d    vel;		// NED velocity estimate in earth frame in m/s
	Eigen::Vector3d    pos;		// NED position estimate in earth frame in m/s
	uint64_t    time_us;		// timestamp in microseconds
}outputSample_t;

typedef struct imuSample {
	void SetZero() {
		delta_ang.setZero();
		delta_vel.setZero();
		delta_ang_dt = 0;
		delta_vel_dt = 0;
		time_us = 0;
	}
	Eigen::Vector3d    delta_ang;	// delta angle in body frame (integrated gyro measurements)
	Eigen::Vector3d    delta_vel;	// delta velocity in body frame (integrated accelerometer measurements)
	double       delta_ang_dt;	// delta angle integration period in seconds
	double       delta_vel_dt;	// delta velocity integration period in seconds
	uint64_t    time_us;	// timestamp in microseconds
}imuSample_t;

typedef struct gpsSample {
	void SetZero() {
		pos.setZero();
		hgt = 0;
		vel.setZero();
		hacc = 0;
		vacc = 0;
		sacc = 0;
		time_us = 0;
	}
	Eigen::Vector2d    pos;	// 投影后的位置,NE earth frame gps horizontal position measurement in m
	double       hgt;	// gps height measurement in m
	Eigen::Vector3d    vel;	// NED earth frame gps velocity measurement in m/s
	double	    hacc;	// 1-std horizontal position error m
	double	    vacc;	// 1-std vertical position error m
	double		sacc;  //速度噪声
	uint64_t    time_us;	// timestamp in microseconds
}gpsSample_t;

typedef struct magSample {
	void SetZero() {
		mag.setZero();
		time_us = 0;
	}
	Eigen::Vector3d    mag;	// NED magnetometer body frame measurements
	uint64_t    time_us;	// timestamp in microseconds
}magSample_t;

typedef struct baroSample {
	void SetZero() {
		hgt = 0;
		time_us = 0;
	}
	double       hgt{ 0.0f };	// barometer height above sea level measurement in m
	uint64_t    time_us{ 0 };	// timestamp in microseconds
}baroSample_t;

class EstimatorInterface
{
public:
	EstimatorInterface() = default;
	~EstimatorInterface() = default;

	virtual bool init(uint64_t timestamp) = 0;
	virtual bool update() = 0;

	// gets the innovations of velocity and position measurements
	// 0-2 vel, 3-5 pos
	//virtual void get_vel_pos_innov(Eigen::VectorXd& vel_pos_innov) = 0;

	// gets the innovations of the earth magnetic field measurements
	//virtual void get_mag_innov(Eigen::Vector3d mag_innov) = 0;

	// gets the innovation variances of velocity and position measurements
	// 0-2 vel, 3-5 pos
	//virtual void get_vel_pos_innov_var(Eigen::VectorXd& vel_pos_innov_var) = 0;

	// gets the innovation of airspeed measurement
	//virtual void get_airspeed_innov(double& airspeed_innov) = 0;

	// gets the innovation variances of the earth magnetic field measurements
	//virtual void get_mag_innov_var(Eigen::Vector3d& mag_innov_var) = 0;

	// gets the innovation variance of the airspeed measurement
	//virtual void get_airspeed_innov_var(double& get_airspeed_innov_var) = 0;

	virtual void get_state_delayed(Eigen::VectorXd& state) = 0;

	//virtual void get_wind_velocity(Eigen::VectorXd& wind) = 0;

	virtual void get_covariances(Eigen::MatrixXd& covariances) = 0;

	// gets the variances for the NED velocity states
	//virtual void get_vel_var(Eigen::Vector3d &vel_var) = 0;

	// gets the variances for the NED position states
	//virtual void get_pos_var(Eigen::Vector3d &pos_var) = 0;

	/*
	Returns  following IMU vibration metrics in the following array locations
	0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
	1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
	2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
	*/
	virtual void get_imu_vibe_metrics(Eigen::Vector3d & vibe) = 0;

	// get the ekf WGS-84 origin position and height and the system time it was last set
	// return true if the origin is valid
	//virtual bool get_ekf_origin(uint64_t &origin_time, map_projection_reference_s *origin_pos, double &origin_alt) = 0;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
	//virtual void get_ekf_gpos_accuracy(double & ekf_eph, double& ekf_epv, bool &dead_reckoning) = 0;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
	//virtual void get_ekf_lpos_accuracy(double &ekf_eph, double &ekf_epv, bool &dead_reckoning) = 0;

	// get the 1-sigma horizontal and vertical velocity uncertainty
	//virtual void get_ekf_vel_accuracy(double &ekf_evh, double &ekf_evv, bool &dead_reckoning) = 0;

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	virtual bool collect_gps(uint64_t time_usec, sampleGPS_t  &gps) { return true; }

	// accumulate and downsample IMU data to the EKF prediction rate
	virtual bool collect_imu(imuSample_t &imu) { return true; }

	// set delta angle imu data
	void setIMUData(uint64_t time_usec, double delta_ang_dt, double delta_vel_dt,
		Eigen::Vector3d& delta_ang, Eigen::Vector3d& delta_vel);

	// set magnetometer data
	void setMagData(uint64_t time_usec, Eigen::Vector3d& mag);

	// set gps data
	void setGpsData(uint64_t time_usec,sampleGPS_t& gps);

	// set baro data
	void setBaroData(uint64_t time_usec, double data);

	void copy_quaternion(Quatd quat)
	{
		quat = outputNew_.quat_nominal;
	}

	void get_velocity(Eigen::Vector3d& vel)
	{
		vel = outputNew_.vel;
	}

	void get_position(Eigen::Vector3d& pos) {

		pos = outputNew_.pos;
	}


protected:
	parameters_t params_;		// filter parameters
	uint64_t obsBufferLength_{ 0 };
	uint64_t imuBufferLength_{ 0 };
	uint64_t minObsInterval_us_{ 0 }; // minimum time interval between observations that will guarantee data is not lost (usec)
	const uint64_t FILTER_UPDATE_PERIOD_MS = 12;

	double dtImuAvg_{ 0.0 };	// average imu update period in s

	imuSample_t imuSampleDelayed_{};
	gpsSample_t gpsSampleDelayed_{};
	magSample_t magSampleDelayed_{};
	baroSample_t baroSampleDelayed_{};

	Eigen::Vector3d deltaAngPrev_{};	// delta angle from the previous IMU measurement
	Eigen::Vector3d deltaVelPrev_{};	// delta velocity from the previous IMU measurement
	Eigen::Vector3d vibeMetrics_{};	// IMU vibration metrics

	outputSample_t outputSampleDelayed_{};	// filter output on the delayed time horizon
	outputSample_t outputNew_{};		// filter output on the non-delayed time horizon
	imuSample_t imuSampleNew_{};
	Eigen::Matrix3d R_ToEarthNow_{};

	struct map_projection_reference_s posRef_ {};   // Contains WGS-84 position latitude and longitude (radians) of the EKF origin
	struct map_projection_reference_s gpsPosPrev_ {};   // Contains WGS-84 position latitude and longitude (radians) of the previous GPS message

	uint64_t timeLastImu_us_{ 0 };
	uint64_t timeLastGps_us_{ 0 };
	uint64_t timeLastMag_us_{ 0 };
	uint64_t timeLastBaro_us_{ 0 };

	uint64_t imuTicks_{ 0 };	// counter for imu updates

	bool imuUpdated_{ false };      // true if the ekf should update (completed downsampling process)
	bool initialised_{ false };      // true if the ekf interface instance (data buffering) is initialized

	bool NED_OriginInitialised_{ false };

	// data buffer instances
	RingBuffer<imuSample_t> imuBuffer_;
	RingBuffer<gpsSample_t> gpsBuffer_;
	RingBuffer<magSample_t> magBuffer_;
	RingBuffer<baroSample_t> baroBuffer_;
	RingBuffer<outputSample_t> outputBuffer_;

	// allocate data buffers and intialise interface variables
	bool initialise_interface(uint64_t timestamp);

	// free buffer memory
	void unallocate_buffers();

};