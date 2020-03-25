#pragma once

#include"estimator_interface.h"


class Ekf : public EstimatorInterface {
public:
	Ekf() = default;
	~Ekf() = default;

	bool init(uint64_t timestamp);

	bool update();

	bool collect_imu(imuSample_t &imu);
	bool collect_gps(uint64_t time_usec, sampleGPS_t& gps);

	void get_state_delayed(Eigen::VectorXd& state);

	void get_covariances(Eigen::MatrixXd& covariances);

	void get_imu_vibe_metrics(Eigen::Vector3d & vibe);

private:
	const double  earthRate_{ 0.000072921};
	const double gravity_{9.8};
	const uint64_t kNumStates_{ 16 };

	stateSample_t state_{};
	Eigen::MatrixXd kalman_P_=Eigen::MatrixXd::Zero(16,16);

	//气压计时间
	uint64_t delta_time_baro_us_;

	Eigen::Vector3d deltaAngleCorr_;
	
	bool filterInitialised_{false};
	double dt_ekf_avg_;

	//imu下采样时使用
	imuSample_t imuDownSampled_;
	Quatd q_DownSampled_;
	double imu_collection_time_adj_;

	Eigen::Matrix3d R_ToEarth_;

	uint64_t timeLastPosFuse_us{ 0 };
	uint64_t timeLastVelFuse_us{ 0 };
	uint64_t timeLastHgtFuse_us{ 0 };
	
	//互补滤波时使用
	Eigen::Vector3d correctedDeltaAng_;
	Eigen::Vector3d correctedDeltaVel_;
	Eigen::Vector3d velErrInteg_;	// integral of velocity tracking error
	Eigen::Vector3d posErrInteg_;

	//初始化滤波器使用
	Eigen::Vector3d delVelSum_;
	uint64_t magCounter_;
	Eigen::Vector3d magFilterState_;
	uint64_t hgtCounter_;
	double hgtBaroOffset_;

	//地球自转角速率
	Eigen::Vector3d EarthRate_NED_{0,0,0};
	bool earthRateInitialised_{false};
	//磁偏角
	double magDeclination_;

	Eigen::Vector3d outputTrackingError_;

	void calculateOutputStates();

	bool initialiseFilter(void);

	void initialiseCovariance();

	void initialiseQuatCovariances(Eigen::Vector3d &);

	void predictState();

	void predictCovariance();

	void fuseHeading();

	void fuseGps();

	void fuseBaro();

	void controlFusionModes();

	bool resetMagHeading(Eigen::Vector3d& );

	void alignOutputFilter();

	void calcEarthRateNED(Eigen::Vector3d &omega, double lat_rad) const;

	void calcMagDeclination();

	void ConvertStateToVector(Eigen::VectorXd& x);

	void undateState(Eigen::VectorXd& x);

	double constrain(double value,double lower, double upper);
};