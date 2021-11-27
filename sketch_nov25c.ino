#include <AFMotor.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
 
MPU6050 mpu;

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

const int FORWARD_SPEED = 150;
 
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
 
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
 
void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
 
mpu.initialize();
Serial.begin(115200);
devStatus = mpu.dmpInitialize();
 
// make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
 
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
 
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
 
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
 
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  motor1.setSpeed(200);
  motor1.run(RELEASE);
  
  motor2.setSpeed(200);
  motor2.run(RELEASE);
}
 
 
void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
 
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize);
 
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
 
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
 
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("Orientation: ");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print(", ");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print(", ");
    Serial.println(ypr[2] * 180/M_PI);
  }

  if(millis() > 4000) {
    float angle = fmod(RAD_TO_DEG * ypr[0] + 360, 360) - 180;
    int turnSpeed = (int)(angle * 8);
    int speedLeft = FORWARD_SPEED - turnSpeed;
    int speedRight = FORWARD_SPEED + turnSpeed;
  
    runMotor(motor1, speedLeft);
    runMotor(motor2, speedRight);
  
    Serial.print("Turn Speed: ");
    Serial.println(turnSpeed);
  }
}

void runMotor(AF_DCMotor motor, int power) {
  byte speed = min(255, abs(power));
  if(power > 0) {
    motor.run(FORWARD);
    motor.setSpeed(speed);
  } else if(power < 0) {
    motor.run(BACKWARD);
    motor.setSpeed(speed);
  } else {
    motor.run(RELEASE);
  }
}
