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

		// 외부 환경 변수 
		float angle = cs.getAngle(); // 자동차의 방향과 트랙 축 방향 사이의 각도 
		                             // -PI 에서 PI 사이의 값을 가진다. 

		float opponents[36]; // 36개의 상대방 자동차를 감지하는 센서 
		                     // 각 센서는 10도씩의 범위를 담당한다. (36개의 센서가 있고, 각각 10도씩 담당하면 총 360도 즉 전뱡향을 감지한다)
		                     // 각 센서가 감지하는 영역에 있는 가장 가까운 상대방 자동차와의 거리를 저장하고 있다. 
							 // 0~100 (m) 까지의 값을 가진다. 

	
		float rpm = cs.getRpm(); // 자동차 엔진의 Rotation per minute (RPM) 
		float speedx = cs.getSpeedX(); // 현재 자동차가 진행하는 방향의 속도 (km/h) 
		float track[19];       // 19개의 센서가 자동차 앞부분과 트랙의 끝부분 사이의 거리를 측정한다. 
							   // 자동차 진행 방향 왼쪽은 0번 센서, 가운데는 9번 센서, 오른쪽은 18번 센서가 측정한다. 
		                       // 각 센서는 10도씩의 범위를 담당한다. 
							   // 0~100 (m) 까지의 값을 가진다. 

		float trackPos=cs.getTrackPos(); // 자동차가 현재 트랙의 어디에 위치해 있나? 
		                               // 정중앙에 있으면 0
									   // 오른쪽 끝에 있으면 -1 
									   // 왼쪽 끝에 있으면 +1 
									   // 1보다 크거나 -1보다 작다면 자동차는 트랙 밖에 있는 것이다. 	  

		// 자동차 제어관련 변수 
        int gear = getGear(cs); // 기어는 자동이다. 컴퓨터가 자동으로 결정해 준다. 수동으로 기어를 조작하고 싶으면 gear 값을 바꾸어주면 된다. 
		                        // -1은 후진이고, 0은 중립, 1~7단까지 있다. 
 
		float steer = previous_steer;   // 자동차의 핸들을 조정한다. -1부터 1까지의 값을 가진다. 
		               // -1은 완전히 오른쪽으로 돌린 것이다, +1은 완전히 왼쪽으로 돌린 것이다. 
		               // 차의 바퀴는 최대 45도까지 방향 전환이 가능하다. 

		// 사용자 정의 변수 
		int z; 

		// 센서 초기화 
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
