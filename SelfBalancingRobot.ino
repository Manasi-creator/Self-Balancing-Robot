#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

class LMotorController {
public:
    int ENA_PIN;
    int IN1_PIN;
    int IN2_PIN;
    int ENB_PIN;
    int IN3_PIN;
    int IN4_PIN;
    double motorSpeedFactorLeft;
    double motorSpeedFactorRight;

    LMotorController(int ena, int in1, int in2, int enb, int in3, int in4, double factorLeft, double factorRight) {
        ENA_PIN = ena;
        IN1_PIN = in1;
        IN2_PIN = in2;
        ENB_PIN = enb;
        IN3_PIN = in3;
        IN4_PIN = in4;
        motorSpeedFactorLeft = factorLeft;
        motorSpeedFactorRight = factorRight;

        pinMode(ENA_PIN, OUTPUT);
        pinMode(IN1_PIN, OUTPUT);
        pinMode(IN2_PIN, OUTPUT);
        pinMode(ENB_PIN, OUTPUT);
        pinMode(IN3_PIN, OUTPUT);
        pinMode(IN4_PIN, OUTPUT);
    }

    void move(double motorPower, int minAbsSpeed) {
        int motorSpeed = abs(motorPower);

        if (motorSpeed > 0 && motorSpeed < minAbsSpeed) {
            motorSpeed = minAbsSpeed;
        } else if (motorSpeed > 255) {
            motorSpeed = 255;
        }

        int leftMotorSpeed = motorSpeed * motorSpeedFactorLeft;
        int rightMotorSpeed = motorSpeed * motorSpeedFactorRight;

        leftMotorSpeed = constrain(leftMotorSpeed, 0, 150);
        rightMotorSpeed = constrain(rightMotorSpeed, 0, 150);

        if (motorPower > 0) {
            digitalWrite(IN1_PIN, LOW);
            digitalWrite(IN2_PIN, HIGH);
            digitalWrite(IN3_PIN, LOW);
            digitalWrite(IN4_PIN, HIGH);
        }
        else if (motorPower < 0) {
            digitalWrite(IN1_PIN, HIGH);
            digitalWrite(IN2_PIN, LOW);
            digitalWrite(IN3_PIN, HIGH);
            digitalWrite(IN4_PIN, LOW);
        }
        else {
            digitalWrite(IN1_PIN, LOW);
            digitalWrite(IN2_PIN, LOW);
            digitalWrite(IN3_PIN, LOW);
            digitalWrite(IN4_PIN, LOW);
            leftMotorSpeed = 0;
            rightMotorSpeed = 0;
        }

        analogWrite(ENA_PIN, leftMotorSpeed);
        analogWrite(ENB_PIN, rightMotorSpeed);
    }
};

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

double originalSetpoint = 182.90;
double setpoint = originalSetpoint;
double input, output;

double Kp = 100;
double Ki = 30;
double Kd = 10;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double movementOffset = 0.0;

#define MIN_ABS_SPEED 20

double motorSpeedFactorLeft = 1.0;
double motorSpeedFactorRight = 1.0;

int ENA = 11;
int IN1 = 7;
int IN2 = 6;
int ENB = 10;
int IN3 = 5;
int IN4 = 4;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;
#endif

  Serial.begin(115200);
  while (!Serial);

  mpu.initialize();

  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    while(true);
  }
}

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[1] * 180/M_PI + 180;

    setpoint = originalSetpoint + movementOffset;

    Serial.print("Input: ");
    Serial.print(input);
    Serial.print("\tSetpoint: ");
    Serial.print(setpoint);
    Serial.print("\tOutput: ");
    Serial.println(output);
  }
}