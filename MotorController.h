
/*
  MotorController.h - Library for getting encoder position and velocity.
  Created by Zachary Potoskie, March 30, 2023.
*/
#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"
#include "encoder.h"

#define BOTTOM_CAP 45

struct PIDControl
{
    float proportional;
    float integral;
    float derivative;
};

class MotorController
{
    public:
        MotorController(int pin1, int pin2, int pin3, PIDControl gains, Encoder* encoder);
        void begin(void);
        void positionControl(float targetPosition);
        void rest(void);
        float openloopSpeed(int controlSignal);
        
        float currentPosition(void);
        float currentVelocity(void);
        float currentTime(void);

        int PWMPin;
        int directionPin1;
        int directionPin2;

        int PWMValue = 0;
        int motorDirection = 0;
    
    private:
        PIDControl _pid;
        Encoder *_encoder;

        void driveMotor(float controlSignal);
        void update(void);

        //PID-related
        float _currentTime = 0;
        float _previousTime = 0;
        float _deltaTime = 0;
        float _errorValue = 0;
        float _previousError = 0;
        float _errorDot = 0;
        float _errorIntegral = 0;
        float _previousPosition = 0;
        float _velocity = 0;
        
    };

    

#endif