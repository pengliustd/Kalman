
#include<iostream>
#include<fstream>
#include<iomanip>


#include"../DSS_Timer/time_simulation.h"
#include"../Sensors/sensor_from_txt.h"
#include"../MSG/message.hpp"
#include"../DSS_Math/geo.h"
#include"../DSS_Math/quaternion.hpp"
#include"../Estimator/estimator.h"

using namespace std;
using namespace DSS;

const static string InFilePath("input/");

const static string OutFilePath("output/");

static fstream imufile(InFilePath + (string)"imu.txt", fstream::in);
static fstream gpsfile(InFilePath + (string)"gps.txt", fstream::in);
static fstream magfile(InFilePath + (string)"mag.txt", fstream::in);
static fstream barofile(InFilePath + (string)"baro.txt", fstream::in);


static fstream outfile1(OutFilePath + (string)"imu.txt", fstream::out);
static fstream outfile2(OutFilePath + (string)"gps.txt", fstream::out);
static fstream outfile3(OutFilePath + (string)"mag.txt", fstream::out);
static fstream outfile4(OutFilePath + (string)"baro.txt", fstream::out);

void file_open() {
	outfile1.setf(ios::fixed);
	outfile2.setf(ios::fixed);
	outfile3.setf(ios::fixed);
	outfile4.setf(ios::fixed);
}


void file_close() {
	imufile.close();
	gpsfile.close();
	magfile.close();
	barofile.close();

	outfile1.close();
	outfile2.close();
	outfile3.close();
	outfile4.close();
}


void task_timer() {
	DSS::TimeGlobal_us globalTime;

	for (int i = 0; i != 100; i++)
		globalTime.IncrementTime();

	cout <<"GetClock_ID\t"<< globalTime.GetClock_ID() << endl;
	cout << "GetTime\t" << globalTime.GetTime() << endl;
	cout << "GetTime_s\t" << globalTime.GetTime_s() << endl;
	cout << "GetTime_ms\t" << globalTime.GetTime_ms() << endl;

	globalTime.IncrementTime(2000);
	cout << endl;
	cout << "GetClock_ID\t" << globalTime.GetClock_ID() << endl;
	cout << "GetTime\t" << globalTime.GetTime() << endl;
	cout << "GetTime_s\t" << globalTime.GetTime_s() << endl;
	cout << "GetTime_ms\t" << globalTime.GetTime_ms() << endl;

	globalTime.SetTime_s(9.8);
	cout << endl;
	cout << "GetClock_ID\t" << globalTime.GetClock_ID() << endl;
	cout << "GetTime\t" << globalTime.GetTime() << endl;
	cout << "GetTime_s\t" << globalTime.GetTime_s() << endl;
	cout << "GetTime_ms\t" << globalTime.GetTime_ms() << endl;

}

//gps水平位置，设置7位有效数字
void task_sensor() {
	DSS::TimeGlobal_us globalTime;
	uint64_t timestamp;
	sensorTxt sensor;
	sensor.Initialize(imufile, gpsfile, magfile, barofile);

	sampleIMU_t imu;
	sampleGPS_t gps;
	sampleMag_t mag;
	sampleBaro_t baro;
	message<sampleIMU_t> mesimu;
	message<sampleGPS_t> mesgps;
	message<sampleMag_t> mesmag;
	message<sampleBaro_t> mesbaro;

	map_projection_reference_s ref;

	//表示水平位置，用于投影测试
	double x(0), y(0);

	file_open();

	while (1) {
		timestamp=globalTime.GetTime();
		if (sensor.run(timestamp)) {
			if (mesimu.CheckUpdate(imu)) {
				mesimu.CopyTo(imu);
				//outfile1 << timestamp<<"\t"<<imu.acc(0) << endl;
			}
			if (mesgps.CheckUpdate(gps)) {
				mesgps.CopyTo(gps);
				//投影测试
				if (map_projection_initialized(&ref)) {
					map_projection_project(&ref, gps.pos(0), gps.pos(1), &x, &y);
				}
				else {
					map_projection_init_timestamped(&ref, gps.pos(0), gps.pos(1), timestamp);
				}
				outfile2 <<setprecision(3)<< x <<"\t"<<y<< endl;
			}
			if (mesmag.CheckUpdate(mag)) {
				mesmag.CopyTo(mag);
				//outfile3 << mag.magStrenth(0) << endl;
			}
			if (mesbaro.CheckUpdate(baro)) {
				mesbaro.CopyTo(baro);
				//outfile4 << baro.altitude << endl;
			}
		}
		else {
			break;
		}
		
		if(globalTime.GetTime()%1000000==0)
			cout << "GetTime_s\t" << globalTime.GetTime_s() << endl;
		globalTime.IncrementTime(1000);

	}

	file_close();
}


void task_quaternion() {
	Eulerd euler(1, 2, 1);
	Quatd q(euler);

	cout << (Dcm(q.Inversed()) - Dcm(q).transpose()) << endl;

}


void task_kalman() {
	DSS::TimeGlobal_us globalTime;
	uint64_t timestamp;
	sensorTxt sensor;
	sensor.Initialize(imufile, gpsfile, magfile, barofile);

	Estimator estimator;

	message<controlState_t> mescon;
	controlState_t con;

	file_open();

	while (1) {
		timestamp = globalTime.GetTime();
		if (sensor.run(timestamp)) {
			estimator.TaskMain(timestamp);
			if (mescon.CheckUpdate(con)) {
				mescon.CopyTo(con);
				outfile1 << con.euler(0) << "\t" << con.euler(1) << "\t" << con.euler(2) << endl;
				outfile2 << con.localpos(0) << "\t" << con.localpos(1) << "\t" << con.localpos(2)<<"\t";
				outfile2 << con.vel(0) << "\t" << con.vel(1) << "\t" << con.vel(2) << endl;
			}
		}
		else {
			break;
		}

		if (globalTime.GetTime() % 1000000 == 0)
			cout << "GetTime_s\t" << globalTime.GetTime_s() << endl;
		globalTime.IncrementTime(1000);

	}

	file_close();
}