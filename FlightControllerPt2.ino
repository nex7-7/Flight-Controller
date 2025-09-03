/*
IMP Notes: Roll is along Y, and Pitch is along X
*/

#include <IBusBM.h>
#include <Servo.h>
#include <Wire.h>
// #include <SoftwareSerial.h>

// SoftwareSerial BTSerial(10, 11);

// IBus Object to Serially read data from the Reciever
IBusBM IBus;

// Control Flags
bool setupCheck = false;  //Indicates if setup is complete
bool connection = false;  //Indicates if connection is complete
bool power = false;       //Indicates power
bool FAILSAFE = false;    //Indicates if Reciever is connected
bool debug = false;
bool batteryCritical = false;  //Indicates if Battery voltage is lower than critical battery voltage;

// Pins
uint8_t LED = 13;
uint8_t DEBUG = 2;

//Timer Control
unsigned long timer;
uint8_t interval = 4;  //250Hz loop -> 4ms

// Testing Variables
int count = 1;

// Variables to store data from reciever
int16_t CH1 = 0;  // Roll
int16_t CH2 = 0;  // Pitch
int16_t CH3 = 0;  // Throttle
int16_t CH4 = 0;  // Yaw
// int16_t CH5 = 0;
// int16_t CH6 = 0;
int16_t CH7 = 0;  //Switch

// Servo objects to control ESCs
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

// Control Parameters
float roll;
float pitch;
float yaw;
uint16_t throttle = 0;

// Sensitivity/Range Control
uint8_t rollCtrl = 10;
uint8_t pitchCtrl = 10;
uint8_t yawCtrl = 10;
uint8_t throttleCtrl = 1;

// MPU6050 Variables
int16_t accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
float angleRoll, anglePitch;

int16_t gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float rotXcal = 0, rotYcal = 0, rotZcal = 0;
float accXcal = 0, accYcal = 0, accZcal = 0;

// PID Control Variables
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;


float PIDReturn[] = { 0, 0, 0 };

float PRateRoll = 0.6;
float PRatePitch = PRateRoll;
float PRateYaw = 2;
float IRateRoll = 3.5;
float IRatePitch = IRateRoll;
float IRateYaw = 12;
float DRateRoll = 0.03;
float DRatePitch = DRateRoll;
float DRateYaw = 0;

float PAngleRoll = 2;
float PAnglePitch = PAngleRoll;
float IAngleRoll = 0;
float IAnglePitch = IAngleRoll;
float DAngleRoll = 0;
float DAnglePitch = DAngleRoll;

// Kalman Filter Variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };

//ESC Control Values
uint16_t ESC1time;
uint16_t ESC2time;
uint16_t ESC3time;
uint16_t ESC4time;

//Battery Critical value;
const float batteryCriticalVal = 11.40;
float voltage;


void setup() {
  Serial.begin(115200);
  // BTSerial.begin(115200);
  IBus.begin(Serial);

  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

  debugCheck();
  // connect();
  setupESC();

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

  setupMPU();
  MPUCal();
  PIDreset();

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

  txCheck();
  // setupCheck = true;
  // power = true;
  digitalWrite(LED, HIGH);
  if (debug) Serial.println("Setup Complete");
  timer = millis();
}

void debugCheck() {
  pinMode(DEBUG, INPUT);
  if (digitalRead(DEBUG)) debug = true;
  debugSetup();
}

void debugSetup() {
  pinMode(4, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(12, INPUT);
}

void setupESC() {
  ESC1.attach(3);
  ESC2.attach(5);
  ESC3.attach(6);
  ESC4.attach(9);
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  ESC3.writeMicroseconds(1000);
  ESC4.writeMicroseconds(1000);
  delay(100);
}

void setupMPU() {
  // digitalWrite(LED, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0b1101000);  //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B);                   //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000);             //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x1B);                   //Starting register for Gyro Readings
  Wire.write(0x00001000);             //Setting the gyro to full scale +/- 1000deg./s
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x1C);                   //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00010000);             //Setting the accel to +/- 8g
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x1A);
  Wire.write(0x05);  // Applying Low Pass Filter
  Wire.endTransmission();
}

void MPUCal() {
  for (int i = 0; i < 2000; i++) {
    recordGyroRegisters();
    processGyroData();

    recordAccelRegisters();
    processAccelData();

    rotXcal += rotX;
    rotYcal += rotY;
    rotZcal += rotZ;

    accXcal += gForceX;
    accYcal += gForceY;
    accZcal += gForceZ;

    delay(1);
  }

  rotXcal = rotXcal / 2000;
  rotYcal = rotYcal / 2000;
  rotZcal = rotZcal / 2000;

  accXcal = accXcal / 2000;
  accYcal = accYcal / 2000;
  accZcal = accZcal / 2000;
  accZcal = 1 - accZcal;

  if (accXcal > 0.20) accXcal = 0.12;
  if (accYcal > 0.20) accYcal = 0.03;
  if (accZcal > 0.20) accYcal = 0.11;
}

void txCheck() {
  while (IBus.readChannel(2) > 1020 || IBus.readChannel(2) == 0 || IBus.readChannel(6) < 1020) {

    //if Throttle is not set to lowest, or is 0 (Tx is not on) or Power Switch (SwA) is not ON
    Serial.println(IBus.readChannel(6));
    delay(1);
  }
  setupCheck = true;
  power = true;
}

// void connect() {
//   while (!connection) {
//     if (BTSerial.available()) {
//       if (BTSerial.read() == 67) {
//         BTSerial.println("Connection Established");
//         connection = true;
//         delay(500);
//       }
//     }
//     delay(1);
//   }
// }

void loop() {
  if (setupCheck && power && millis() - timer > interval && count > 0) {
    timer = millis();
    // Serial.println(millis());

    readReciever();
    setPower();
    batteryVoltage();
    setFailsafe();

    if (!FAILSAFE) {
      inputConversion();
      // ESCtest();
    } else {
      roll = 0;
      pitch = 0;
      if (throttle > 1000) {
        throttle--;
      }
    }

    recordMPU();
    processMPU();
    kalmanCtrl();
    PIDctrl();
    ESCsignal();
    ESCctrl();

    if (debug) print();
    // Serial.println(millis());
    // count--;
  }
}

void readReciever() {
  CH1 = IBus.readChannel(0);
  CH2 = IBus.readChannel(1);
  CH3 = IBus.readChannel(2);
  CH4 = IBus.readChannel(3);
  // CH5 = IBus.readChannel(4);
  // CH6 = IBus.readChannel(5);
  CH7 = IBus.readChannel(6);
}

void inputConversion() {
  if (CH1 < 1000) CH1 = 1000;
  if (CH2 < 1000) CH2 = 1000;
  if (CH3 < 1000) CH3 = 1000;
  if (CH4 < 1000) CH4 = 1000;

  roll = (CH1 - 1500) * (1.0) / rollCtrl * (1.0);
  pitch = (CH2 - 1500) * (1.0) / pitchCtrl * (1.0);
  yaw = (CH4 - 1500) * (1.0) / yawCtrl * (1.0);
  throttle = CH3;
}

void recordMPU() {
  recordGyroRegisters();
  recordAccelRegisters();
}

void processMPU() {
  processGyroData();
  processAccelData();
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x43);                   //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);  //Request Gyro Registers (43 - 48)
  while (Wire.available() < 6)
    ;
  gyroX = Wire.read() << 8 | Wire.read();  //Store first two bytes into gyroX
  gyroY = Wire.read() << 8 | Wire.read();  //Store middle two bytes into gyroY
  gyroZ = Wire.read() << 8 | Wire.read();  //Store last two bytes into gyroZ
}

void processGyroData() {
  if (!setupCheck) {
    rotX = (float)gyroX / 65.5;
    rotY = (float)gyroY / 65.5;
    rotZ = (float)gyroZ / 65.5;
  } else {
    rotX = (float)gyroX / 65.5 - rotXcal;
    rotY = (float)gyroY / 65.5 - rotYcal;
    rotZ = (float)gyroZ / 65.5 - rotZcal;
  }
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x3B);                   //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);  //Request Accel Registers (3B - 40)
  while (Wire.available() < 6)
    ;
  accelX = Wire.read() << 8 | Wire.read();  //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read();  //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read();  //Store last two bytes into accelZ
}

void processAccelData() {
  if (!setupCheck) {
    gForceX = ((float)accelX / 4096.0);
    gForceY = ((float)accelY / 4096.0);
    gForceZ = ((float)accelZ / 4096.0);
  } else {
    gForceX = ((float)accelX / 4096.0) - accXcal;
    gForceY = ((float)accelY / 4096.0) - accYcal;
    gForceZ = ((float)accelZ / 4096.0) + accZcal;

    angleRoll = -atan(gForceX / sqrt(gForceY * gForceY + gForceZ * gForceZ)) * (180 / 3.142);
    anglePitch = atan(gForceY / sqrt(gForceX * gForceX + gForceZ * gForceZ)) * (180 / 3.142);
  }
}

void PIDfunction(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;

  float Iterm = PrevIterm + I * (Error + PrevError) * (((interval * 1.0) / 1000) / 2);
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;

  float Dterm = D * ((Error - PrevError) / ((interval * 1.0) / 1000));

  float PIDoutput = Pterm + Iterm + Dterm;
  if (PIDoutput > 400) PIDoutput = 400;
  else if (PIDoutput < -400) PIDoutput = -400;

  PIDReturn[0] = PIDoutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void PIDreset() {
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;

  PrevErrorAngleRoll = 0;
  PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0;
  PrevItermAnglePitch = 0;
}

void PIDctrl() {
  ErrorAngleRoll = roll - KalmanAngleRoll;
  ErrorAnglePitch = pitch - KalmanAnglePitch;

  PIDfunction(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  PIDfunction(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAnglePitch = PIDReturn[2];

  ErrorRateRoll = DesiredRateRoll - rotY;
  ErrorRatePitch = DesiredRatePitch - rotX;
  ErrorRateYaw = yaw - rotZ;

  PIDfunction(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  PIDfunction(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  PIDfunction(ErrorRateYaw, PRateYaw,
              IRateYaw, DRateYaw, PrevErrorRateYaw,
              PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];
}

void kalmanFilter(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + ((float)interval / 1000) * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + (((float)interval / 1000) * ((float)interval / 1000)) * 4 * 4;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void kalmanCtrl() {
  kalmanFilter(KalmanAngleRoll, KalmanUncertaintyAngleRoll, rotY, angleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalmanFilter(KalmanAnglePitch, KalmanUncertaintyAnglePitch, rotX, anglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
}

void ESCsignal() {
  if (throttle > 1900) throttle = 1900;

  ESC1time = (throttle - InputRoll - InputPitch - InputYaw);
  ESC2time = (throttle - InputRoll + InputPitch + InputYaw);
  ESC3time = (throttle + InputRoll + InputPitch - InputYaw);
  ESC4time = (throttle + InputRoll - InputPitch + InputYaw);

  if (ESC1time > 2000) ESC1time = 1999;
  if (ESC2time > 2000) ESC2time = 1999;
  if (ESC3time > 2000) ESC3time = 1999;
  if (ESC4time > 2000) ESC4time = 1999;

  uint16_t ThrottleIdle = 1180;
  if (ESC1time < ThrottleIdle) ESC1time = ThrottleIdle;
  if (ESC2time < ThrottleIdle) ESC2time = ThrottleIdle;
  if (ESC3time < ThrottleIdle) ESC3time = ThrottleIdle;
  if (ESC4time < ThrottleIdle) ESC4time = ThrottleIdle;

  uint16_t throttleCutOff = 1000;
  if (throttle < 1050) {
    ESC1time = throttleCutOff;
    ESC2time = throttleCutOff;
    ESC3time = throttleCutOff;
    ESC4time = throttleCutOff;
    PIDreset();
  }
}

void ESCctrl() {
  ESC1.writeMicroseconds(ESC1time);
  ESC2.writeMicroseconds(ESC2time);
  ESC3.writeMicroseconds(ESC3time);
  ESC4.writeMicroseconds(ESC4time);
}

void setFailsafe() {
  if ((CH1 < 1000 && CH2 < 1000 && CH3 < 1000 && CH4 < 1000) || batteryCritical) FAILSAFE = true;
}

void setPower() {
  if (CH7 > 10 && CH7 < 1020) {
    power = false;
    powerDown();
  } else if (CH7 > 1990 && CH7 < 2020) {
    power = true;
  }
}



void powerDown() {
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  ESC3.writeMicroseconds(1000);
  ESC4.writeMicroseconds(1000);

  PIDreset();

  roll = 0;
  pitch = 0;
  throttle = 1000;

  setupCheck = false;
  power = false;
  txCheck();
}


void batteryVoltage() {
  voltage = analogRead(A0) * ((5 * 2.52) / 1023.0);
  if (voltage < batteryCriticalVal) {
    batteryCritical = true;
  }
}

void ESCtest() {
  ESC1.writeMicroseconds(CH3);
  ESC2.writeMicroseconds(CH3);
  ESC3.writeMicroseconds(CH3);
  ESC4.writeMicroseconds(CH3);
}

void print() {
  if (digitalRead(4)) {
    Serial.println(throttle);
    Serial.println(roll);
    Serial.println(pitch);
    Serial.println(yaw);
  }

  if (digitalRead(7)) {
    Serial.print(1000);
    Serial.print(",");
    Serial.print(ESC1time);
    Serial.print(",");
    Serial.print(ESC2time);
    Serial.print(",");
    Serial.print(ESC3time);
    Serial.print(",");
    Serial.print(ESC4time);
    Serial.print(",");
    Serial.println(2000);
  }

  if (digitalRead(8)) {
    Serial.print(InputRoll);
    Serial.print(",");
    Serial.print(InputPitch);
    Serial.print(",");
    Serial.println(InputYaw);
  }

  if (digitalRead(12)) {
    Serial.print(DesiredRateRoll);
    Serial.print(",");
    Serial.println(DesiredRatePitch);
  }

  // Serial.print("Gyro (deg)");
  // Serial.print(" X=");
  // Serial.print(rotX);
  // Serial.print(" Y=");
  // Serial.print(rotY);
  // Serial.print(" Z=");
  // Serial.print(rotZ);
  // Serial.print(" Accel (g)");
  // Serial.print(" X=");
  // Serial.print(gForceX);
  // Serial.print(" Y=");
  // Serial.print(gForceY);
  // Serial.print(" Z=");
  // Serial.println(gForceZ);

  // BTSerial.println("Values: ");


  // Serial.print(KalmanAngleRoll);
  // Serial.println(KalmanAnglePitch);
}