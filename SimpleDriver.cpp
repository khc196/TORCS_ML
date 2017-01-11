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
		 
		float temp_steer = steer;
		temp_steer += keyboard_steering(); 
		cruise_control(speedx);
		if (temp_steer >= 0) 
			steer = min(temp_steer, 1.0f);
		else if (temp_steer < 0) 
			steer = max(temp_steer, -1.0f);
		previous_steer = steer;
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
SimpleDriver::cruise_control(float current_speed)
{
	const float KP = 0.1f;
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
