#include"time_simulation.h"

namespace DSS {
	/* Used for automatically initializing clockID. */
	uint64_t TimeGlobal_us::countAuto_ = 0;
	uint64_t TimeGlobal_ms::countAuto_ = 0;

	/*************************************************/
	double TimeGlobal_us::GetTime_s()const {
		double us_s = 1e-6;
		return time_ * us_s;
	}

	double TimeGlobal_us::GetTime_ms()const {
		double us_ms = 1e-3;
		return time_ * us_ms;
	}

	uint64_t TimeGlobal_us::GetClock_ID()const {
		return clock_ID_;
	}

	void TimeGlobal_us::IncrementTime_s(const double& t) {
		double s_us = 1e6;
		time_ += static_cast<uint64_t>(s_us*t);
	}

	void TimeGlobal_us::SetTime_s(const double& t) {
		double s_us = 1e6;
		time_ = static_cast<uint64_t>(s_us*t);
	}
	/**************************************************/

	/**************************************************/
	double TimeGlobal_ms::GetTime_s()const {
		double us_ms = 1e-3;
		return time_ * us_ms;
	}

	double TimeGlobal_ms::GetTime_ms()const {
		return static_cast<double>(time_);
	}

	uint64_t TimeGlobal_ms::GetClock_ID()const {
		return clock_ID_;
	}

	void TimeGlobal_ms::IncrementTime_s(const double& t) {
		double s_ms = 1e3;
		time_ += static_cast<uint64_t>(s_ms*t);
	}

	void TimeGlobal_ms::SetTime_s(const double& t) {
		double s_ms = 1e3;
		time_ = static_cast<uint64_t>(s_ms*t);
	}
	/*************************************************/

	/*************************************************/
	double Time_us::GetTime_s()const {
		double us_s = 1e-6;
		return time_ * us_s;
	}

	double Time_us::GetTime_ms()const {
		double us_ms = 1e-3;
		return time_ * us_ms;
	}

	uint64_t Time_us::GetClock_ID()const {
		return 0;
	}

	void Time_us::IncrementTime_s(const double& t) {
		double s_us = 1e6;
		time_ += static_cast<uint64_t>(s_us*t);
	}

	void Time_us::SetTime_s(const double& t) {
		double s_us = 1e6;
		time_ = static_cast<uint64_t>(s_us*t);
	}
	/*****************************************************/


	/**************************************************/
	double Time_ms::GetTime_s()const {
		double us_ms = 1e-3;
		return time_ * us_ms;
	}

	double Time_ms::GetTime_ms()const {
		return static_cast<double>(time_);
	}

	uint64_t Time_ms::GetClock_ID()const {
		return 0;
	}

	void Time_ms::IncrementTime_s(const double& t) {
		double s_ms = 1e3;
		time_ += static_cast<uint64_t>(s_ms*t);
	}

	void Time_ms::SetTime_s(const double& t) {
		double s_ms = 1e3;
		time_ = static_cast<uint64_t>(s_ms*t);
	}
	/*************************************************/
}