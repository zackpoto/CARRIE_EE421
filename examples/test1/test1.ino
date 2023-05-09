#include <encoder.h>
#include <telephone.h>
#include <MotorController.h>
#include <DualMotorSpeedController.h>

#include <Wire.h>
#include <MPU6050.h>

/*
 * STEERING MOTOR/ENCODER SPECS
 * GEAR RATIO: 1/98
 * RAW ENCODER PULSES: 12 PPR
 * OUTPUT ENCODER: 12*98 = 1176 PPR
 * MAX SPEED: 60 RPM or 360˚/s (actual 340˚/s or 55.67 RPM)
 
 * DRIVE MOTOR/ENCODER SPECS
 * GEAR RATIO: 1/?
 * ENCODER PULSES: 53 PPR??
 * MAX SPEED: ????
 * WHEEL DIAM: 80mm
 * WHEELBASE: 0.22m
 * TRACK: 0.235m
 */

EncoderManager encoderManager = EncoderManager();

//Steering Motor Encoder
const int steeringEncoderPin1 = 3;
const int steeringEncoderPin2 = 4;
const float steeringEncoderPPR = 1176;
Encoder steeringEncoder = Encoder(steeringEncoderPin1, steeringEncoderPin2, steeringEncoderPPR);

//Steering Motor Controller
PIDControl steeringPID = {5, 0.00005, 0.01};
MotorController steeringController = MotorController(11, 12, 13, steeringPID, &steeringEncoder);

//Drive Enconders
const float driveEncoderPPR = 50;

//Left Drive Motor Encoder
const int leftdriveEncoderPin1 = 18;
const int leftdriveEncoderPin2 = 17;
Encoder leftDriveEncoder = Encoder(leftdriveEncoderPin1, leftdriveEncoderPin2, driveEncoderPPR);

//Right Drive Motor Encoder
const int rightdriveEncoderPin1 = 19;
const int rightdriveEncoderPin2 = 2;
Encoder rightDriveEncoder = Encoder(rightdriveEncoderPin1, rightdriveEncoderPin2, driveEncoderPPR);

DualMotorSpeedController speedController = DualMotorSpeedController(10, 9, 8, steeringPID, &leftDriveEncoder, &rightDriveEncoder);

struct Input command = {0, 0};
struct State state = {0, 0, 0, 0, 0};
bool rest = 1;

MPU6050 mpu;

// Timers
float timer = 0;
float prev_timer = 0;
float delta_time = 0;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

void setup()
{
  Serial.begin(115200);

  encoderManager.begin();
  encoderManager.addDevice(&steeringEncoder);
  encoderManager.addDevice(&rightDriveEncoder);
  encoderManager.addDevice(&leftDriveEncoder);
  
  steeringController.begin();
  speedController.begin();

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(0);
}

void loop()
{
  steeringController.positionControl(30.0);
//  if (prev_timer == 0) {
//    prev_timer = millis();
//  }
//  timer = millis();
////  Serial.print(timer);
////  Serial.print("\t");
////  Serial.println(prev_timer);
//  delta_time = (timer-prev_timer)/1000;
//  
//  if (delta_time > 0.005) {
//    // Read normalized values
//    Vector norm = mpu.readNormalizeGyro();
//  
//    // Calculate Pitch, Roll and Yaw
////    pitch = pitch + norm.YAxis * delta_time;
////    roll = roll + norm.XAxis * delta_time;
//    state.yaw = state.yaw + norm.ZAxis * delta_time;
//
//    prev_timer = millis();
//  }

//  state.d = steeringController.currentPosition();
//  sendReceive();
}

void sendReceive()
{
  rest = check_inputs(state, &command);
}
