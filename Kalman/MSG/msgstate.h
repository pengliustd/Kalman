#pragma once
/* This file is about data structure of system state. 
* In simulation, the true state of system is known.
*/
#include <cstdint>
#include "../Eigen/Dense"
#include"../DSS_Math/quaternion.hpp"
#include"../DSS_Math/euler.hpp"

typedef struct controlState {
	controlState() :
		timestamp(0),
		pos(),
		localpos(),
		vel(),
		wbody(),
		q(),
		euler(),
		acc(),
		intergral_time(0)
	{
		pos.setZero();
		localpos.setZero();
		vel.setZero();
		wbody.setZero();
		acc.setZero();
	}
	uint64_t timestamp;
	/* pos(0)=latitude,pos(1)=longotude,pos(2)=altitude_MSL. */
	Eigen::Vector3d pos;
	/* Local position in NED frame. */
	Eigen::Vector3d localpos;
	/* vel(0)=North velocity, vel(1)=East velocity, vel(2)=Down velocity. */
	Eigen::Vector3d vel;
	/* Body Angular velocity described in body frame. */
	Eigen::Vector3d wbody;
	/* Attitude is in quaternion. */
	Quatd q;
	/* Attitude is in euler. */
	Eulerd euler;
	/* Acc in NED frame. */
	Eigen::Vector3d acc;
	double intergral_time;
}controlState_t;

typedef struct systemState {
	systemState() :
		timestamp(0),
		pos(),
		vel(),
		wbody(),
		q(),
		euler(),
		mass(0),
		J(),
		intergral_time(0)
	{
		pos.setZero();
		vel.setZero();
		wbody.setZero();
		J.setZero();
	}
	uint64_t timestamp;
	/* pos(0)=latitude,pos(1)=longitude,pos(2)=altitude_MSL. */
	Eigen::Vector3d pos;
	/* vel(0)=North velocity, vel(1)=East velocity, vel(2)=Down velocity. */
	Eigen::Vector3d vel;
	/* Body Angular velocity described in body frame. */
	Eigen::Vector3d wbody;
	/* Attitude is in quaternion. */
	Quatd q;
	/* Attitude is in euler. */
	Eulerd euler;

	double mass;

	Eigen::Matrix3d J;

	double intergral_time;
}systemState_t;


typedef struct windState {

	windState() :
		timestamp(0),
		vel(),
		density(1.25),
		intergral_time(0)
	{
		vel.setZero();
	}
	uint64_t timestamp;
	/* vel(0)=North velocity, vel(1)=East velocity, vel(2)=Down velocity. */
	Eigen::Vector3d vel;
	/* Air density. */
	double density;
	double intergral_time;
}windState_t;

typedef struct windEstimate {

	windEstimate() :
		timestamp(0),
		vel(),
		intergral_time(0)
	{
		vel.setZero();
	}
	uint64_t timestamp;
	/* vel(0)=North velocity, vel(1)=East velocity, vel(2)=Down velocity. */
	Eigen::Vector3d vel;
	double intergral_time;
}windEstimate_t;