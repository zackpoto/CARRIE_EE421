
/*
  DualMotorSpeedController.h - Library for getting encoder position and velocity.
  Created by Zachary Potoskie, March 30, 2023.
*/
#ifndef DualMotorSpeedController_h
#define DualMotorSpeedController_h

#include "Arduino.h"
#include "encoder.h"
// Remove library below after moving PID struct to common file
#include "MotorController.h"

class DualMotorSpeedController
{
    public:
        DualMotorSpeedController(int pin1, int pin2, int pin3, PIDControl gains, Encoder* encoder1, Encoder* encoder2);
        void begin(void);
        void velocityControl(float targetVelocity);
        void rest(void);
        
        float leftPosition(void);
        float rightPosition(void);
        float leftVelocity(void);
        float rightVelocity(void);
        float currentVelocity(void);

        void update(void);

        float openloopSpeed(int controlSignal);

        int PWMPin;
        int directionPin1;
        int directionPin2;

        int PWMValue = 0;
        int motorDirection = 0;
    
    private:
        PIDControl _pid;
        Encoder *_left_encoder;
        Encoder *_right_encoder;

        void driveMotor(float controlSignal);

        //PID-related
        float _currentTime = 0;
        float _previousTime = 0;
        float _deltaTime = 0;
        float _errorValue = 0;
        float _previousError = 0;
        float _errorDot = 0;
        float _errorIntegral = 0;

        float _previousLeftPosition = 0;
        float _previousRightPosition = 0;
        float _left_velocity = 0;
        float _right_velocity = 0;
        float _velocity = 0;

        const float _tipover = 0.8;
        
    };

    

#endif