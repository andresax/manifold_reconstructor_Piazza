/*
 * Chronometer.cpp
 *
 *  Created on: 19 Aug 2016
 *      Author: enrico
 */

#include <Chronometer.h>

Chronometer::Chronometer() {
}
Chronometer::~Chronometer() {
}

void Chronometer::start(){
	if(ticking_) return;
	ticking_ = true;

	startTime_ = std::chrono::high_resolution_clock::now();
}

void Chronometer::stop(){
	if(!ticking_) return;
	ticking_ = false;

	auto lastDelta = std::chrono::high_resolution_clock::now() - startTime_;
	elapsed_ += std::chrono::duration_cast<std::chrono::microseconds>(lastDelta).count();
}

void Chronometer::reset(){
	ticking_ = false;
//	startTime_ = 0;
	elapsed_ = 0;
}

long long Chronometer::getMicroseconds(){
	if(ticking_){
		stop();
		start();
	}
	return elapsed_;
}

float Chronometer::getSeconds(){
	return getMicroseconds() / 1000000.0;
}

