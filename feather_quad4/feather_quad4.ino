// LoRa 32u4 simple quad / flight controller dev
// inspired by many people, but especially Nick Rehm, EEEnthusiast, HowToMechatronics, and Joop Brokking
// by Wolfy Bauer, August 2021 | v0.0

#include <Wire.h>
#include <LoRa.h>
#include <Servo.h>
#include <EEPROMAnything.h>
#include "util.h"

bool IS_ARMED = false;
bool IS_UNSAVED = false;

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();

  motor_init();
 // setMotorFailsafe();

  setRadioFailsafe();     // set all controls to defaults on startup
  lora_init();            // initialize lora module

  // Load saved PID values
  EEPROM_readAnything(0, pid_in.data);
  getDesPID();

  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);
  //Serial.println("WARMING UP in [3] ...");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  //Serial.println("WARMING UP in [2] ...");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  //Serial.println("WARMING UP in [1] ...");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println();
  IMU_init();             // 
  delay(10);
  calculate_IMU_error();
  delay(100);

  

}

void loop() {
  
  // -- LOOP START -- //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = micros();            // Current time actual time read
  dTime = (currentTime - previousTime) / 1000000.0f; // Divide by 1,000,000 to get seconds
  
  // IMU stuff
  getAccelData();
  getGyroData();
  compFilter();

  // radio stuff
  getRadioInput();
  controlANGLE();
  //getDesState();      // (moved inside getRadioInput(), for now)

  // check for PID tuning mode
  if(!PID_TUNE_ON) {
    
    //check if returning from PID tuning mode, save values
    if(IS_UNSAVED) {
      EEPROM_writeAnything(0, pid_in.data);
      IS_UNSAVED = false;
    }
    
    // check for arm mode
    if(data_in[L_click] && data_in[R_click]) IS_ARMED = true;
    if(!data_in[L_click] && !data_in[R_click]) IS_ARMED = false;
    
    //controlANGLE();
    if(IS_ARMED) {  // only do if ARMED
      //controlANGLE();
      // DO MOTOR STUFF HERE
      controlMixer();
      actuateMotors();
    } else {
      if(data_in[B_click]) {
        actuateMotors();
      } else {
        setMotorFailsafe();
      }
    }
    //printIMUData();
    //plotGyro();
    //plotARccel();
    //printRollPitchYaw();
    //printRadioData();
    //printDesState();
    //printStateError();
    //printPID();
    //Serial.print("STANDBY MODE|");
    //printPIDTuning();
    printMotorMix();
    //printTerms();
    //graphPitchPID();
    //graphRollPID();
  }

  if(PID_TUNE_ON) {
    setRadioFailsafe();
    setMotorFailsafe();
    IS_UNSAVED = true;
    Serial.print("PID TUNE MODE|");
    printPIDTuning();
  }

  // -- LOOP END -- //
  loopRate(LOOP_RATE);
}

void IMU_init() {
  
  // 1. 0x6B: DEVICE RESET, SLEEP, CYCLE, TEMP_DIS, CLKSEL
  // (register map sec. 4.28)
  Wire.beginTransmission(MPU_ADDR); // start talking to MPU
  Wire.write(0x6B);                 // access MPU register 6B
  Wire.write(0x00);                 // write all to 0 
  Wire.endTransmission();

  // 2. 0x1B: GYRO CONFIG
  // (register map sec. 4.4)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(GYRO_SCALE_SEL);
  Wire.endTransmission();

  // 3. 0x1C: ACCEL CONFIG
  // (register map sec. 4.5)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(ACCEL_SCALE_SEL);
  Wire.endTransmission();
  
}

void getGyroData() {
  
  // registers 43-48
  // (register map sec. 4.19)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);                   // address the first register
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 6);      // request 6 bytes
  while(Wire.available() < 6);         // ????? dontlike this...
  
  // get 16bit values for each axis, banged together in 2 byte packets
  gX_raw = Wire.read() << 8 | Wire.read();
  gY_raw = Wire.read() << 8 | Wire.read();
  gZ_raw = Wire.read() << 8 | Wire.read();

  // correct for IMU error
  gX_out = (gX_raw / GYRO_SCALE_FACTOR) - GyroErrorX;
  gY_out = (gY_raw / GYRO_SCALE_FACTOR) - GyroErrorY;
  gZ_out = (gZ_raw / GYRO_SCALE_FACTOR) - GyroErrorZ;

  // apply low pass filter
  gX_out = (1.0 - B_gyro) * gX_prev + B_gyro * gX_out;
  gY_out = (1.0 - B_gyro) * gY_prev + B_gyro * gY_out;
  gZ_out = (1.0 - B_gyro) * gZ_prev + B_gyro * gZ_out;
  gX_prev = gX_out;
  gY_prev = gY_out;
  gZ_prev = gZ_out;
  
}

void getAccelData() {
  // registers 3B-40
  // (register map sec. 4.17)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                   // address the first register
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 6);      // request 6 bytes
  while(Wire.available() < 6);        // ????? dontlike this...
  
  // get 16bit values for each axis, banged together in 2 byte packets
  aX_raw = Wire.read() << 8 | Wire.read();
  aY_raw = Wire.read() << 8 | Wire.read();
  aZ_raw = Wire.read() << 8 | Wire.read();

  // correct for IMU error
  aX_out = (aX_raw / ACCEL_SCALE_FACTOR) - AccErrorX;
  aY_out = (aY_raw / ACCEL_SCALE_FACTOR) - AccErrorY;
  aZ_out = (aZ_raw / ACCEL_SCALE_FACTOR) - AccErrorZ;

  // apply low pass filter
  aX_out = (1.0 - B_accel) * aX_prev + B_accel * aX_out;
  aY_out = (1.0 - B_accel) * aY_prev + B_accel * aY_out;
  aZ_out = (1.0 - B_accel) * aZ_prev + B_accel * aZ_out;
  aX_prev = aX_out;
  aY_prev = aY_out;
  aZ_prev = aZ_out;
  
}

void compFilter() {
  // convert accel data
  float aPitch = atan2(aX_out, aZ_out) * RAD_TO_DEG;
  float aRoll  = atan2(aY_out, aZ_out) * RAD_TO_DEG;

  // convert gyro data
  float gPitch = gY_out * dTime;
  float gRoll  = gX_out * dTime;

  // apply complementary filter
  roll_IMU = COMP_FACTOR * gRoll + (1-COMP_FACTOR) * aRoll;
  pitch_IMU = COMP_FACTOR * gPitch + (1-COMP_FACTOR) * aPitch;
  yaw_IMU = gZ_out * dTime;

  // convert to proper degrees and constrain to -180 to 180
  roll_out = constrain((roll_IMU * 50), -180.0, 180.0);
  pitch_out = constrain((pitch_IMU * 50), -180.0, 180.0);
  yaw_out = constrain((yaw_IMU * 50), -180.0, 180.0);
}

void calculate_IMU_error() {
  
  // Read accelerometer values ERROR_CYCLES times
  while (c < ERROR_CYCLES) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    aX_raw = Wire.read() << 8 | Wire.read();
    aY_raw = Wire.read() << 8 | Wire.read();
    aZ_raw = Wire.read() << 8 | Wire.read();
    // Sum all readings
    AccErrorX = AccErrorX + (aX_raw / ACCEL_SCALE_FACTOR);
    AccErrorY = AccErrorY + (aY_raw / ACCEL_SCALE_FACTOR);
    AccErrorZ = (AccErrorZ - 1.0) + (aZ_raw / ACCEL_SCALE_FACTOR);
    c++;
  }
  // Divide the sum by ERROR_CYCLES to get the error value
  AccErrorX = AccErrorX / ERROR_CYCLES;
  AccErrorY = AccErrorY / ERROR_CYCLES;
  AccErrorZ = AccErrorZ / ERROR_CYCLES;
  // reset counter variable for gyro data
  c = 0;
  // Read gyro values ERROR_CYCLES times
  while (c < ERROR_CYCLES) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    gX_raw = Wire.read() << 8 | Wire.read();
    gY_raw = Wire.read() << 8 | Wire.read();
    gZ_raw = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (gX_raw / GYRO_SCALE_FACTOR);
    GyroErrorY = GyroErrorY + (gY_raw / GYRO_SCALE_FACTOR);
    GyroErrorZ = GyroErrorZ + (gZ_raw / GYRO_SCALE_FACTOR);
    c++;
  }
  //Divide the sum by ERROR_CYCLES to get the averaged error value
  GyroErrorX = GyroErrorX / ERROR_CYCLES;
  GyroErrorY = GyroErrorY / ERROR_CYCLES;
  GyroErrorZ = GyroErrorZ / ERROR_CYCLES;
}

void loopRate(int freq) {
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - currentTime)) {
    checker = micros();
  }
}

void lora_init() {
  Serial.println("Attempting to start LoRa...");
  LoRa.setPins(SS,RST,DI0); // set correct pins for logic level converters etc
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);              // loop forever if LoRa init fails
  }
  Serial.println("LoRa started successfully!");
  Serial.println("RECEIVER init ok");
}

void getRadioInput() {
  int packetSize = LoRa.parsePacket();
  
  // receive normal control packet
  if(packetSize == PACKET_SIZE) {
    while(LoRa.available()) {
      for(int i=0; i<PACKET_SIZE; i++) {
        data_in[i] = LoRa.read();
      }
    }
    data_in[R_ypos] = 255 - data_in[R_ypos];
    getDesState();    // translate incoming bytes to usuable "desired state"
    PID_TUNE_ON = false;
  }
  // receive pid tuning packet
  if(packetSize == PID_PACKET_SIZE) {
    while(LoRa.available()) {
      for(int i=0; i<PID_PACKET_SIZE; i++) {
        pid_in.bytes_in[i] = LoRa.read();
      }
    }
    getDesPID();    // translate incoming bytes to new PID tuning values
    PID_TUNE_ON = true;
  }
}


void getDesState() {

  thro_des = data_in[P_pos] / 255.0;
  roll_des = ((data_in[R_xpos] - 127) / 255.0) * maxRoll;
  //roll_des = 0 - roll_des; // <------ REVERSE IT??
  pitch_des = ((data_in[R_ypos] - 127) / 255.0) * maxPitch;
  //pitch_des = 0 - pitch_des; // <------ REVERSE IT??
  yaw_des = ((data_in[L_xpos] - 127) / 255.0) * maxYaw;

  roll_passthru = roll_des / (2.0 * maxRoll);
  pitch_passthru = pitch_des / (2.0 * maxPitch);
  yaw_passthru = yaw_des / (2.0 * maxYaw);
  
}

void getDesPID() {
  Kp_roll_angle = pid_in.data.p_val;
  Kp_pitch_angle = pid_in.data.p_val;
  Ki_roll_angle = pid_in.data.i_val;
  Ki_pitch_angle = pid_in.data.i_val;
  Kd_roll_angle = pid_in.data.d_val;
  Kd_pitch_angle = pid_in.data.d_val;
}

void controlANGLE() {
  
  //Roll
  error_roll = roll_des - roll_out;
  integral_roll = integral_roll_prev + error_roll * dTime;
  if (data_in[P_pos] < I_CUTOFF) {   //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = gX_out;
  roll_PID = 0.01 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll); //scaled by .01 to bring within -1 to 1 range
  //roll_PID = 0 - roll_PID; // <------ REVERSE IT??

  //Pitch
  error_pitch = pitch_des - pitch_out;
  integral_pitch = integral_pitch_prev + error_pitch * dTime;
  if (data_in[P_pos] < I_CUTOFF) {   //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = gY_out;
  pitch_PID = .01 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch); //scaled by .01 to bring within -1 to 1 range
  //pitch_PID = 0 - pitch_PID; // <------ REVERSE IT??

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - yaw_out;
  integral_yaw = integral_yaw_prev + error_yaw * dTime;
  if (data_in[P_pos] < I_CUTOFF) {   //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dTime; 
  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlMixer() {
  mm1_out = thro_des + pitch_PID + roll_PID + yaw_PID;
  mm2_out = thro_des + pitch_PID - roll_PID - yaw_PID;
  mm3_out = thro_des - pitch_PID - roll_PID + yaw_PID;
  mm4_out = thro_des - pitch_PID + roll_PID - yaw_PID;

  m1_out = mm1_out * 180;
  m2_out = mm2_out * 180;
  m3_out = mm3_out * 180;
  m4_out = mm4_out * 180;

  m1_out = constrain(m1_out, 0, 180);
  m2_out = constrain(m2_out, 0, 180);
  m3_out = constrain(m3_out, 0, 180);
  m4_out = constrain(m4_out, 0, 180);
}

void actuateMotors() {
  if(data_in[B_click]) {      // if in ESC cal mode, 
    motor_cal_out = map(data_in[P_pos], 0, 255, 0, 180);
    m1_servo.write(motor_cal_out);
    m2_servo.write(motor_cal_out);
    m3_servo.write(motor_cal_out);
    m4_servo.write(motor_cal_out);
  } else {
    m1_servo.write(m1_out);
    m2_servo.write(m2_out);
    m3_servo.write(m3_out);
    m4_servo.write(m4_out);
  }
}
