#pragma once
/* This file is about Message structures of basic sensors. */
#include <cstdint>
#include "../Eigen/Dense"


typedef struct sampleIMU {
	sampleIMU() :
		timestamp(0),
		gyro(),
		acc(),
		intergral_time(0)
	{
		gyro.setZero();
		acc.setZero();
	}
	uint64_t timestamp;
	/* Measuremet from gyroscope. */
	Eigen::Vector3d gyro;
	/* Measuremet from accelerometer. */
	Eigen::Vector3d acc;
	/* The time interval between two meassurement. */
	double intergral_time;
}sampleIMU_t;

typedef struct sampleGPS {
	sampleGPS() :
		timestamp(0),
		pos(),
		altitude_ellipsoid(0),
		s_variance_m_s(0),
		c_variance_rad(0),
		eph(100),
		epv(100),
		hdop(100),
		vdop(100),
		vel(),
		intergral_time(0)
	{
		pos.setZero();
		vel.setZero();
	}
	uint64_t timestamp;
	/* pos(0)=latitude,pos(1)=longotude,pos(2)=altitude_MSL. */
	Eigen::Vector3d pos;
	/* WGS84 height. */
	double altitude_ellipsoid; 
	/* GPS speed accuracy estimate, (metres / sec). */
	double s_variance_m_s;
	/* GPS course accuracy estimate, (radians). */
	double c_variance_rad;
	/* GPS horizontal position accuracy (metres). */
	double eph;
	/* GPS vertical position accuracy (metres). */
	double epv;
	/* Horizontal dilution of precision. */
	double hdop;
	/* Vertical dilution of precision. */
	double vdop;
    /* vel(0)=North velocity,vel(1)=East velocity, vel(2)=Down velocity. */
	Eigen::Vector3d vel;
	double intergral_time;
}sampleGPS_t;

typedef struct sampleMag {
	sampleMag() :
		timestamp(0),
		magStrenth(),
		intergral_time(0)
	{
		magStrenth.setZero();
	}
	uint64_t timestamp;
	/* Magnetic strenth. */
	Eigen::Vector3d magStrenth;
	double intergral_time;
}sampleMag_t;

typedef struct sampleBaro {
	sampleBaro() :
		timestamp(0),
		altitude(0), 
		temperature(0),
		intergral_time(0)
	{}
	uint64_t timestamp;
	/* Height above mean sea level. */
	double altitude;
	double temperature;
	double intergral_time;
}sampleBaro_t;

typedef struct sampleBattery {
	sampleBattery() :timestamp(0),
		voltage(0), voltage_filtered(0),
		current(0), current_filtered(0),
		intergral_time(0)
	{}
	uint64_t timestamp;
	double voltage;
	double voltage_filtered;
	double current;
	double current_filtered;
	double intergral_time;
}sampleBattery_t;


typedef struct SensorCal{
	SensorCal() :
		cal_acc_xoff(-0.056),
		cal_acc_xsacle(0.997),
		cal_acc_yoff(-0.163),
		cal_acc_ysacle(0.998),
		cal_acc_zoff(-0.065),
		cal_acc_zsacle(0.991),

		cal_gyro_xoff(-0.004),
		cal_gyro_xsacle(1.000),
		cal_gyro_yoff(0.065),
		cal_gyro_ysacle(1.000),
		cal_gyro_zoff(-0.014),
	    cal_gyro_zsacle(1.000)
	{}

	double cal_acc_xoff;
	double cal_acc_xsacle;
	double cal_acc_yoff;
	double cal_acc_ysacle;
	double cal_acc_zoff;
	double cal_acc_zsacle;

	double cal_gyro_xoff;
	double cal_gyro_xsacle;
	double cal_gyro_yoff;
	double cal_gyro_ysacle;
	double cal_gyro_zoff;
	double cal_gyro_zsacle;
}SensorCal_t;

typedef struct errEstimation{
	errEstimation()
	{
		err_vel.setZero();
		err_attitude.setZero();
		err_pos.setZero();
		err_acc.setZero();
		err_gyro.setZero();
	}

	Eigen::Vector3d err_vel;
	Eigen::Vector3d err_attitude;
	Eigen::Vector3d err_pos;
	Eigen::Vector3d err_acc;
	Eigen::Vector3d err_gyro;
}errEstimation_t;

typedef struct imu_var {
	imu_var() :
		timestamp(0),
		acc_var(),
		gyro_var(),
		intergral_time(0)
	{
		acc_var.setZero();
		gyro_var.setZero();
	}
	uint64_t timestamp;
	Eigen::Vector3d acc_var;
	Eigen::Vector3d gyro_var;
	double intergral_time;
}imu_var_t;