/***************************************************************************
 
    file                 : SimpleDriver.h
    copyright            : (C) 2007 Daniele Loiacono
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#ifndef SIMPLEDRIVER_H_
#define SIMPLEDRIVER_H_

#include <iostream>
#include <cmath>
#include "BaseDriver.h"
#include "CarState.h"
#include "CarControl.h"
#include "SimpleParser.h"
#include "WrapperBaseDriver.h"

#define PI 3.14159265

using namespace std;

class SimpleDriver : public WrapperBaseDriver
{
public:
	
	// Constructor
	SimpleDriver(){stuck=0;};
	
	// SimpleDriver implements a simple and heuristic controller for driving
	virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	virtual void onShutdown();
	
	// Print a restart message 
	virtual void onRestart();

private:
	
	/* Gear Changing Constants*/
	
	// RPM values to change gear 
	static const int gearUp[6];
	static const int gearDown[6];
		
	/* Stuck constants*/
	
	// How many time steps the controller wait before recovering from a stuck position
	static const int stuckTime;
	// When car angle w.r.t. track axis is grather tan stuckAngle, the car is probably stuck
	static const float stuckAngle;
	
	/* Steering constants*/
	
	// Angle associated to a full steer command
	static const float steerLock;	
	// Min speed to reduce steering command 
	static const float steerSensitivityOffset;
	// Coefficient to reduce steering command at high speed (to avoid loosing the control)
	static const float wheelSensitivityCoeff;
	
	/* Accel and Brake Constants*/
	
	// max speed allowed
	static const float maxSpeed;
	// Min distance from track border to drive at  max speed
	static const float maxSpeedDist;
	// pre-computed sin10
	static const float sin10;
	// pre-computed cos10
	static const float cos10;
	
	/* ABS Filter Constants */
	
	// Radius of the 4 wheels of the car
	static const float wheelRadius[4];
	// min slip to prevent ABS
	static const float absSlip;						
	// range to normalize the ABS effect on the brake
	static const float absRange;
	// min speed to activate ABS
	static const float absMinSpeed;					

	// counter of stuck steps
	int stuck;
	
	// Solves the gear changing subproblems
	int getGear(CarState &cs);

	// Apply an ABS filter to brake command
	float filterABS(CarState &cs,float brake);

	// Steering with Keyboard : Hwancheol
	float keyboard_steering();
	float previous_steer = 0;

	// Speed Control with Keyboard : Hwancheol
	void keyboard_speedcontrol();


	// Cruise Control : Hwancheol
	void cruise_control(float current_speed);
	const float TARGET_SPEED = 40.0f;
	float accel;
	float brake;
	bool cc_mode_on = false;

	// Lane Keeping Assist System : Hwancheol
	float lane_keeping(float angle, float* distance_track, float current_speed);
	float const LANE_KEEPING_CONST = 0.000001f;
	float abs_f(float a);
	bool lkas_mode_on = false;
	void setMode_cc();
	void setMode_lkas();


	void print_state(float* track, float angle);

};

#endif /*SIMPLEDRIVER_H_*/
