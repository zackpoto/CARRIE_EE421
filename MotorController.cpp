/*
  MotorConrtoller.cpp - Library for getting encoder position and velocity.
  Created by Zachary Potoskie, March 30, 2023.
*/

#include "Arduino.h"
#include "MotorController.h"

MotorController::MotorController(int pin1, int pin2, int pin3, PIDControl gains, Encoder* encoder)
{
    PWMPin = pin1;
    directionPin1 = pin2;
    directionPin2 = pin3;
    _pid = gains;
    _encoder = encoder;
}

void MotorController::begin(void)
{
    pinMode(directionPin1, OUTPUT);
    pinMode(directionPin2, OUTPUT);
    pinMode(PWMPin, OUTPUT);
}

void MotorController::rest(void)
{
    analogWrite(PWMPin, 0);
}

void MotorController::positionControl(float targetPosition)
{
    update();
    _errorValue = targetPosition - currentPosition();
    _errorDot = (_errorValue - _previousError) / _deltaTime;
    _errorIntegral = _errorIntegral += (_errorValue*_deltaTime);
    _previousError = _errorValue;

    int controlSignal = (_pid.proportional*_errorValue)+(_pid.derivative*_errorDot)+(_pid.integral*_errorIntegral);

    driveMotor(controlSignal);
}

float MotorController::openloopSpeed(int controlSignal)
{
    update();
    driveMotor(controlSignal);
}

void MotorController::update(void)
{
    _currentTime = micros();
    _deltaTime = (_currentTime-_previousTime)/1000000.0;
    _previousTime = _currentTime;
    _velocity = (currentPosition() - _previousPosition) / _deltaTime;
    _previousPosition = currentPosition();
}

float MotorController::currentPosition(void)
{
    return _encoder->angle();
}

float MotorController::currentVelocity(void)
{
    return _velocity;
}

float MotorController::currentTime(void)
{
    return _currentTime;
}

void MotorController::driveMotor(float controlSignal)
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
    if (PWMValue < BOTTOM_CAP  && _errorValue != 0) {
        PWMValue = BOTTOM_CAP;
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