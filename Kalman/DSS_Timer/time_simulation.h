#pragma once
#include<cstdint>
namespace DSS {

/* The base clase for simulated system time controlling. */
class TimeSim {
public:
	TimeSim():time_(0) {}

	virtual ~TimeSim() {}

	virtual double GetTime_s()const = 0;

	virtual double GetTime_ms()const = 0;

	virtual uint64_t GetClock_ID()const = 0;

	virtual void IncrementTime_s(const double& t) = 0;

	virtual void SetTime_s(const double& t) = 0;

	uint64_t GetTime() { return time_; }

	void ResetTime() { time_ = 0; }

	void IncrementTime() { ++time_; }

	void IncrementTime(const uint64_t& t) { time_+=t; }

	void SetTime(const uint64_t& t) { time_ =t; }

protected:
	uint64_t time_;
};

/* The minimun time interval is one microsecond. 
* Single simulated system can have only one global timer.
*/
class TimeGlobal_us: public TimeSim {
public:

	TimeGlobal_us():TimeSim(), clock_ID_(++countAuto_){

	}
	~TimeGlobal_us() { }

	double GetTime_s()const override;

	double GetTime_ms()const override;

	uint64_t GetClock_ID()const override;

	void IncrementTime_s(const double& t) override;

	void SetTime_s(const double& t) override;

private:
	uint64_t clock_ID_;
	static uint64_t countAuto_;
};

/* The minimun time interval is one millisecond. 
* Single simulated system can have only one global timer.
*/
class TimeGlobal_ms : public TimeSim {
public:
	TimeGlobal_ms() :TimeSim(), clock_ID_(++countAuto_) {

	}
	~TimeGlobal_ms(){}

	double GetTime_s()const override;

	double GetTime_ms()const override;

	uint64_t GetClock_ID()const override;

	void IncrementTime_s(const double& t) override;

	void SetTime_s(const double& t)override;

private:
	uint64_t clock_ID_;
	static uint64_t countAuto_;
};

class Time_us :public TimeSim {
	Time_us() :TimeSim(){

	}
	~Time_us() {}

	double GetTime_s()const override;

	double GetTime_ms()const override;

	uint64_t GetClock_ID()const override;

	void IncrementTime_s(const double& t) override;

	void SetTime_s(const double& t) override;

};

class Time_ms :public TimeSim {
	Time_ms() :TimeSim() {

	}
	~Time_ms() {}

	double GetTime_s()const override;

	double GetTime_ms()const override;

	uint64_t GetClock_ID()const override;

	void IncrementTime_s(const double& t) override;

	void SetTime_s(const double& t) override;

};




}