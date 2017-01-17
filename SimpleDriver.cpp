/***************************************************************************
 
    file                 : SimpleDriver.cpp
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
#include "SimpleDriver.h"
#include <cstdio>
#include <conio.h>
#include <windows.h>

#define LEFT 75
#define RIGHT 77
#define UP 72
#define DOWN 80


/* Gear Changing Constants*/
const int SimpleDriver::gearUp[6]=
    {
        5000,6000,6000,6500,7000,0
    };
const int SimpleDriver::gearDown[6]=
    {
        0,2500,3000,3000,3500,3500
    };

/* Stuck constants*/
const int SimpleDriver::stuckTime = 25;
const float SimpleDriver::stuckAngle = .523598775f; //PI/6

/* Accel and Brake Constants*/
const float SimpleDriver::maxSpeedDist=70;
const float SimpleDriver::maxSpeed=150;
const float SimpleDriver::sin10 = 0.17365f;
const float SimpleDriver::cos10 = 0.98481f;

/* Steering constants*/
const float SimpleDriver::steerLock=0.785398f;
const float SimpleDriver::steerSensitivityOffset=80.0f;
const float SimpleDriver::wheelSensitivityCoeff=1;

/* ABS Filter Constants */
const float SimpleDriver::wheelRadius[4]={0.3179f,0.3179f,0.3276f,0.3276f};
const float SimpleDriver::absSlip=2.0;
const float SimpleDriver::absRange=3.0;
const float SimpleDriver::absMinSpeed=3.0;



int
SimpleDriver::getGear(CarState &cs)
{

    int gear = cs.getGear();
    int rpm  = cs.getRpm();

    // if gear is 0 (N) or -1 (R) just return 1 
    if (gear<1)
        return 1;
    // check if the RPM value of car is greater than the one suggested 
    // to shift up the gear from the current one     
    if (gear <6 && rpm >= gearUp[gear-1])
        return gear + 1;
    else
    	// check if the RPM value of car is lower than the one suggested 
    	// to shift down the gear from the current one
        if (gear > 1 && rpm <= gearDown[gear-1])
            return gear - 1;
        else // otherwhise keep current gear
            return gear;
}


CarControl
SimpleDriver::wDrive(CarState cs)
{
	// check if car is currently stuck
	if ( fabs(cs.getAngle()) > stuckAngle )
    {
		// update stuck counter
        stuck++;
    }
    else
    {
    	// if not stuck reset stuck counter
        stuck = 0;
    }

	// after car is stuck for a while apply recovering policy
    if (stuck > stuckTime)
    {
    	/* set gear and sterring command assuming car is 
    	 * pointing in a direction out of track */
    	
    	// to bring car parallel to track axis
        float steer = - cs.getAngle() / steerLock; 
        int gear=-1; // gear R
        
        // if car is pointing in the correct direction revert gear and steer  
        if (cs.getAngle()*cs.getTrackPos()>0)
        {
            gear = 1;
            steer = -steer;
        }
        // build a CarControl variable and return it
        CarControl cc (1.0,0.0,gear,steer);
        return cc;
    }

    else // car is not stuck
    {

		// �ܺ� ȯ�� ���� 
		float angle = cs.getAngle(); // �ڵ����� ����� Ʈ�� �� ���� ������ ���� 
		                             // -PI ���� PI ������ ���� ������. 

		float opponents[36]; // 36���� ���� �ڵ����� �����ϴ� ���� 
		                     // �� ������ 10������ ������ ����Ѵ�. (36���� ������ �ְ�, ���� 10���� ����ϸ� �� 360�� �� �������� �����Ѵ�)
		                     // �� ������ �����ϴ� ������ �ִ� ���� ����� ���� �ڵ������� �Ÿ��� �����ϰ� �ִ�. 
							 // 0~100 (m) ������ ���� ������. 

	
		float rpm = cs.getRpm(); // �ڵ��� ������ Rotation per minute (RPM) 
		float speedx = cs.getSpeedX(); // ���� �ڵ����� �����ϴ� ������ �ӵ� (km/h) 
		float track[19];       // 19���� ������ �ڵ��� �պκа� Ʈ���� ���κ� ������ �Ÿ��� �����Ѵ�. 
							   // �ڵ��� ���� ���� ������ 0�� ����, ����� 9�� ����, �������� 18�� ������ �����Ѵ�. 
		                       // �� ������ 10������ ������ ����Ѵ�. 
							   // 0~100 (m) ������ ���� ������. 

		float trackPos=cs.getTrackPos(); // �ڵ����� ���� Ʈ���� ��� ��ġ�� �ֳ�? 
		                               // ���߾ӿ� ������ 0
									   // ������ ���� ������ -1 
									   // ���� ���� ������ +1 
									   // 1���� ũ�ų� -1���� �۴ٸ� �ڵ����� Ʈ�� �ۿ� �ִ� ���̴�. 	  

		// �ڵ��� ������� ���� 
        int gear = getGear(cs); // ���� �ڵ��̴�. ��ǻ�Ͱ� �ڵ����� ������ �ش�. �������� �� �����ϰ� ������ gear ���� �ٲپ��ָ� �ȴ�. 
		                        // -1�� �����̰�, 0�� �߸�, 1~7�ܱ��� �ִ�. 
 
		float steer = previous_steer;   // �ڵ����� �ڵ��� �����Ѵ�. -1���� 1������ ���� ������. 
		               // -1�� ������ ���������� ���� ���̴�, +1�� ������ �������� ���� ���̴�. 
		               // ���� ������ �ִ� 45������ ���� ��ȯ�� �����ϴ�. 

		// ����� ���� ���� 
		int z; 

		// ���� �ʱ�ȭ 
		for(z=0;z<36;z++) opponents[z]=cs.getOpponents(z); 
		for(z=0;z<19;z++) track[z]=cs.getTrack(z); 



		
		
		
		
		
		/* Hwancheol Start */
		if (GetKeyState(0x43) < 0) 
			setMode_cc();

		if (GetKeyState(0x4C) < 0) {
			setMode_lkas();
		}

		float temp_steer = steer;
		if (lkas_mode_on) 
			temp_steer = lane_keeping(angle, track, speedx);
		else 
			temp_steer += keyboard_steering();
		
		if (cc_mode_on)
			cruise_control(speedx);
		else
			keyboard_speedcontrol();

		if (temp_steer >= 0) 
			steer = min(temp_steer, 1.0f);
		else if (temp_steer < 0) 
			steer = max(temp_steer, -1.0f);
		previous_steer = steer;

	
		print_state(track, angle);
		
		/* Hwancheol End */ 









		brake = filterABS(cs,brake);  
        // build a CarControl variable and return it
        CarControl cc(accel,brake,gear,steer);
        return cc;
    }
}

float
SimpleDriver::filterABS(CarState &cs,float brake)
{
	// convert speed to m/s
	float speed = cs.getSpeedX() / 3.6;
	// when spedd lower than min speed for abs do nothing
    if (speed < absMinSpeed)
        return brake;
    
    // compute the speed of wheels in m/s
    float slip = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        slip += cs.getWheelSpinVel(i) * wheelRadius[i];
    }
    // slip is the difference between actual speed of car and average speed of wheels
    slip = speed - slip/4.0f;
    // when slip too high applu ABS
    if (slip > absSlip)
    {
        brake = brake - (slip - absSlip)/absRange;
    }
    
    // check brake is not negative, otherwise set it to zero
    if (brake<0.0f)
    	return 0.0f;
    else
    	return brake;
}

void
SimpleDriver::onShutdown()
{
    cout << "Bye bye!" << endl;
}

void
SimpleDriver::onRestart()
{
    cout << "Restarting the race!" << endl;
}

float
SimpleDriver::keyboard_steering()
{
	float steer;
	if (GetKeyState(VK_LEFT) < 0) {
		steer = 0.05f;
		if (previous_steer < 0)
			steer -= previous_steer;
	}
	else if (GetKeyState(VK_RIGHT) < 0) {
		steer = -0.05f;
		if (previous_steer > 0)
			steer -= previous_steer;
	}
	else {
		steer = -previous_steer;
	}
	return steer;
}
void
SimpleDriver::keyboard_speedcontrol()
{
	if (GetKeyState(VK_UP) < 0) {
		accel += 0.05;
	}
	else if (GetKeyState(VK_DOWN) < 0) {
		brake += 0.05;
	}
	else {
		if (accel > 0)
			accel -= accel;
		else if (brake > 0)
			brake -= brake;
	}
	
}
void
SimpleDriver::cruise_control(float current_speed)
{
	const float KP = 0.15f;
	float error = TARGET_SPEED - current_speed;
	if (error >= 0.0f)
	{
		accel = min(error * KP, 1.0f);
		brake = 0;
	}
	else if (error < 0.0f)
	{
		accel = 0;
		brake = min(-error * KP, 1.0f);
	}
}
// current_speed - km/h, distance_track - m, angle - -1 ~ 1, 
float
SimpleDriver::lane_keeping(float angle, float* distance_track, float current_speed)
{
	float steer_angle;
	short trackPos;
	float total_width = distance_track[0] + distance_track[18];

	if (distance_track[0] - distance_track[18] >= (total_width/2.0f))
		trackPos = 0;
	else trackPos = 1;
	steer_angle = abs_f(angle) * 45 * 180 / PI + atan(LANE_KEEPING_CONST * (abs_f(distance_track[0] - distance_track[18] - (total_width/2.0f))) / (current_speed * 1000.0f / 3600.0f));
	if (trackPos == 1) return -steer_angle;
	return steer_angle;
}

float
SimpleDriver::abs_f(float a) {
	if (a >= 0.0f) return a;
	else return -a;
}

void
SimpleDriver::setMode_cc() {
	while (GetKeyState(0x43) < 0) {}
	cc_mode_on = !cc_mode_on;
	if (cc_mode_on)
		cout << "Cruise Control On" << endl;
	else
		cout << "Cruise Control Off" << endl;
}
void
SimpleDriver::setMode_lkas() {
	while (GetKeyState(0x4C) < 0) {}
	lkas_mode_on = !lkas_mode_on;
	if (lkas_mode_on)
		cout << "Lane Keeping On" << endl;
	else
		cout << "Lane Keeping Off" << endl;
}

void
SimpleDriver::print_state(float* track, float angle) {
	if (GetKeyState(0x50) < 0) {
		while (GetKeyState(0x50) < 0) {}
		cout << "���� �Ÿ� : " << track[0] << "\n������ �Ÿ� : " << track[18] << "\n���� : " << abs_f(angle) * 45 * PI << endl;
	}
}