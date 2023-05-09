/*
  MotorConrtoller.cpp - Library for getting encoder position and velocity.
  Created by Zachary Potoskie, March 30, 2023.
*/

#include "Arduino.h"
#include "DualMotorSpeedController.h"

DualMotorSpeedController::DualMotorSpeedController(int pin1, int pin2, int pin3, PIDControl gains, Encoder* encoder1, Encoder* encoder2)
{
    PWMPin = pin1;
    directionPin1 = pin2;
    directionPin2 = pin3;
    _pid = gains;
    _left_encoder = encoder1;
    _right_encoder = encoder2;
}

void DualMotorSpeedController::begin(void)
{
    pinMode(directionPin1, OUTPUT);
    pinMode(directionPin2, OUTPUT);
    pinMode(PWMPin, OUTPUT);
}

void DualMotorSpeedController::rest(void)
{
    analogWrite(PWMPin, 0);
}

void DualMotorSpeedController::velocityControl(float targetVelocity)
{
    update();
    _errorValue = targetVelocity - currentVelocity();
    _errorDot = (_errorValue - _previousError) / _deltaTime;
    _errorIntegral = _errorIntegral += (_errorValue*_deltaTime);
    _previousError = _errorValue;

    int controlSignal = (_pid.proportional*_errorValue)+(_pid.derivative*_errorDot)+(_pid.integral*_errorIntegral);

    driveMotor(controlSignal);
}

void DualMotorSpeedController::update(void)
{
    _currentTime = micros();
    _deltaTime = (_currentTime-_previousTime)/1000000.0;

    if (_deltaTime > _tipover) {
        _left_velocity = (leftPosition() - _previousLeftPosition) / _deltaTime;
        _right_velocity = (rightPosition() - _previousRightPosition) / _deltaTime;
        _previousLeftPosition = leftPosition();
        _previousRightPosition = rightPosition();
        _previousTime = _currentTime;
        _velocity = (_left_velocity + _right_velocity)/2;
    }
}

float DualMotorSpeedController::openloopSpeed(int controlSignal)
{
    update();
    driveMotor(controlSignal);
}

float DualMotorSpeedController::leftPosition(void)
{
    return -_left_encoder->angle();
}

float DualMotorSpeedController::rightPosition(void)
{
    return _right_encoder->angle();
}

float DualMotorSpeedController::leftVelocity(void)
{
    return _left_velocity;
}

float DualMotorSpeedController::rightVelocity(void)
{
    return _right_velocity;
}

float DualMotorSpeedController::currentVelocity(void)
{
    return _velocity;
}

void DualMotorSpeedController::driveMotor(float controlSignal)
{
    if (controlSignal < 0) {
        motorDirection = -1;
    } else if (controlSignal > 0) {
        motorDirection = 1;
    } else {
        motorDirection = 0;
    }

    PWMValue = (int)fabs(controlSignal);
    if (PWMValue > 255) {
        PWMValue = 255;
    }
    if (PWMValue < 30  && _errorValue != 0) {
        PWMValue = 30;
    }

    if (motorDirection == -1) {
        digitalWrite(directionPin1, LOW);
        digitalWrite(directionPin2, HIGH);
    } else if (motorDirection == 1) {
        digitalWrite(directionPin1, HIGH);
        digitalWrite(directionPin2, LOW);
    } else {
        digitalWrite(directionPin1, LOW);
        digitalWrite(directionPin2, LOW);
        PWMValue = 0;
    }
    analogWrite(PWMPin, PWMValue);
}