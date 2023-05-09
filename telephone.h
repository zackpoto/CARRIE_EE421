/*
  telephone.h - Library for getting encoder position and velocity.
  Created by Zachary Potoskie, March 30, 2023.
*/
#ifndef telephone_h
#define telephone_h

#include "Arduino.h"

const byte numChars = 100;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

char message[numChars] = {0};
char key[numChars] = {0};
int value = 0;

boolean newData = false;

char *token;

struct Input
{
  float delta;
  float velocity;
};

struct State
{
  float x;
  float y;
  float yaw;
  float v;
  float d;
};

struct Input input = {0,0};
bool shouldRest = 1;

void parseData(void) {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index
    struct Input new_input;

    token = strtok(tempChars,",");
    strcpy(message, token);
    token=strtok(NULL,",");

    if (!strcmp(message, "COMMAND")) {
        while(token != NULL) {
            strcpy(key, token);
            token=strtok(NULL,",");
            value = atof(token);
            token=strtok(NULL,",");

            if (!strcmp(key,"delta")) {
                new_input.delta = value;
            } 
            else if (!strcmp(key,"velocity")) {
                new_input.velocity = value;
            }
            input = new_input;
        }
        shouldRest = 0;
    } else if (!strcmp(message, "REQUEST")) {
        shouldRest = 0;
    } else if (!strcmp(message, "REST")) {
        shouldRest = 1;
    }
}

void recvWithStartEndMarkers(void) {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
bool check_inputs(State sendPacket, Input* ininput) {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            // because strtok() used in parseData() replaces the commas with \0
        parseData();
        newData = false;
        Serial.print("{");
        Serial.print("\"x\":");
        Serial.print(sendPacket.x);
        Serial.print(",\"y\":");
        Serial.print(sendPacket.y);
        Serial.print(",\"yaw\":");
        Serial.print(sendPacket.yaw);
        Serial.print(",\"v\":");
        Serial.print(sendPacket.v);
        Serial.print(",\"delta\":");
        Serial.print(sendPacket.d);
        Serial.print("}");
    }
    *ininput = input;
    return shouldRest;
}

#endif