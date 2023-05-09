
/*
  encoder.h - Library for getting encoder position and velocity.
  Created by Zachary Potoskie, March 30, 2023.
*/
#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

class Encoder
{
    public:
        Encoder(int pinA, int pinB, float ppr);
        void begin(void);
        void zero(void);
        int interruptPin(void);
        int directionPin(void);
        float angle(void);
        void changeCount(int increment);

    private:
        int _pinA;
        int _pinB;
        int _ppr;
        volatile float _count = 0;
};

class EncoderManager
{
    public:
        EncoderManager(void);
        void addDevice(Encoder* newDevice);
        void begin(void);
        static void blink0(void);
        static void blink1(void);
        static void blink2(void);
        static void blink3(void);
        static void blink4(void);
        static void blink(void);

    private:
        static EncoderManager *instance;
        Encoder* _devices[5];
        int _device_count;
        volatile int cur_device;
};

#endif