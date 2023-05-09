/*
  encoder.cpp - Library for getting encoder position and velocity.
  Created by Zachary Potoskie, March 30, 2023.
*/

#include "Arduino.h"
#include "encoder.h"
	
EncoderManager *EncoderManager::instance = nullptr;

EncoderManager::EncoderManager(void)
{
    _device_count = 0;
}

void EncoderManager::begin(void)
{
    if (instance) return;
    instance = this;
}

void EncoderManager::addDevice(Encoder* newDevice)
{
    if (_device_count<5) {
        _devices[_device_count] = newDevice;

        switch (_device_count) {
            case 0:
                attachInterrupt(digitalPinToInterrupt(newDevice->interruptPin()), blink0, RISING);
                break;
            case 1:
                attachInterrupt(digitalPinToInterrupt(newDevice->interruptPin()), blink1, FALLING);
                break;
            case 2:
                attachInterrupt(digitalPinToInterrupt(newDevice->interruptPin()), blink2, FALLING);
                break;
            case 3:
                attachInterrupt(digitalPinToInterrupt(newDevice->interruptPin()), blink3, RISING);
                break;
            case 4:
                attachInterrupt(digitalPinToInterrupt(newDevice->interruptPin()), blink4, RISING);
                break;
        }

        pinMode(newDevice->directionPin(), INPUT);
        _device_count++;
    }
}

static void EncoderManager::blink0(void)
{
    //ISR function
    int aState = digitalRead(instance->_devices[0]->interruptPin());
    if (digitalRead(instance->_devices[0]->directionPin()) != aState) {
        instance->_devices[0]->changeCount(1);
    } else {
        instance->_devices[0]->changeCount(-1);
    } 
}

static void EncoderManager::blink1(void)
{
    //ISR function
    int aState = digitalRead(instance->_devices[1]->interruptPin());
    if (digitalRead(instance->_devices[1]->directionPin()) != aState) {
        instance->_devices[1]->changeCount(-1);
    } else {
        instance->_devices[1]->changeCount(1);
    } 
}

static void EncoderManager::blink2(void)
{
    //ISR function
    int aState = digitalRead(instance->_devices[2]->interruptPin());
    if (digitalRead(instance->_devices[2]->directionPin()) != aState) {
        instance->_devices[2]->changeCount(-1);
    } else {
        instance->_devices[2]->changeCount(1);
    } 
}
static void EncoderManager::blink3(void)
{
    //ISR function
    int aState = digitalRead(instance->_devices[3]->interruptPin());
    if (digitalRead(instance->_devices[3]->directionPin()) != aState) {
        instance->_devices[3]->changeCount(-1);
    } else {
        instance->_devices[3]->changeCount(1);
    } 
}
static void EncoderManager::blink4(void)
{
    //ISR function
    int aState = digitalRead(instance->_devices[4]->interruptPin());
    if (digitalRead(instance->_devices[4]->directionPin()) != aState) {
        instance->_devices[4]->changeCount(-1);
    } else {
        instance->_devices[4]->changeCount(1);
    } 
}


Encoder::Encoder(int pinA, int pinB, float ppr)
{
    _pinA = pinA;
    _pinB = pinB;
    _ppr = ppr;
}

void Encoder::zero(void)
{
    _count = 0;
}

float Encoder::angle(void)
{
    return _count * 360 / (_ppr) ;
}

void Encoder::changeCount(int increment)
{
    _count = _count + increment;
}

int Encoder::interruptPin(void)
{
    return _pinA;
}

int Encoder::directionPin(void)
{
    return _pinB;
}