#pragma once
/* This file is to carried out message operation. */
#include<cstdint>
template<typename Type>
class message{
public:

bool CheckUpdate(const Type & par)const {
	bool update(false);
	if (par.timestamp<messa.timestamp) {
		update = true;
	}
	return update;
}
void CopyTo(Type & par)const {
	double interTime;
	interTime = (messa.timestamp - par.timestamp)/1000000.0;
	par = messa;
	par.intergral_time = interTime;
}

void Publish(const Type& par) {
	messa = par;
}
private:
	static Type messa;
};

template<typename Type> Type message<Type>::messa;