#ifndef UTIL_H
#define UTIL_H

// BSFrance LoRa32u4 v1.1 / Adafruit feather 32u4
#define SCK     15   // GPIO5  -- SX1278's SCK
#define MISO    14   // GPIO19 -- SX1278's MISO
#define MOSI    16   // GPIO27 -- SX1278's MOSI
#define SS      8    // GPIO18 -- SX1278's CS
#define RST     4    // GPIO14 -- SX1278's RESET
#define DI0     7    // GPIO26 -- SX1278's IRQ(Interrupt Request)

// MISC defines
#define BAND      915E6     // radio frequency band USA= 915E6
#define LOOP_RATE 2000      // regulated by loop_rate() function at the end of loop()
#define BAUD_RATE 9600    // Serial communication
const int MPU_ADDR = 0x68;  // AD0 LOW = 0x68, HIGH = 0x69

// MOTOR OUTPUT PINS
#define M1_PIN 11
#define M2_PIN 6
#define M3_PIN 9
#define M4_PIN 10

// GYRO SCALE SELECT (only one)
#define GYRO_250DPS         // default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

// ACCEL SCALE SELECT (only one)
#define ACCEL_2G            // default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G

// GYRO SCALE FACTORS (don't touch)
#if defined GYRO_250DPS
  #define GYRO_SCALE_SEL 0b00000000
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE_SEL 0b00001000
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE_SEL 0b00010000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE_SEL 0b00011000
  #define GYRO_SCALE_FACTOR 16.4
#endif

// ACCEL SCALE FACTORS (don't touch)
#if defined ACCEL_2G
  #define ACCEL_SCALE_SEL 0b00000000
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE_SEL 0b00001000
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE_SEL 0b00010000
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE_SEL 0b00011000
  #define ACCEL_SCALE_FACTOR 2048.0
#endif

int16_t aX_raw, aY_raw, aZ_raw;       // raw accel data
float aX_out, aY_out, aZ_out;         // error corrected
float aX_prev, aY_prev, aZ_prev;      // previous corrected values for filter

int16_t gX_raw, gY_raw, gZ_raw;       // raw gyro data
float gX_out, gY_out, gZ_out;         // error corrected (<---- Kd terms use this)
float gX_prev, gY_prev, gZ_prev;      // previous corrected values for filter

float roll_IMU, pitch_IMU, yaw_IMU;   // after comp filter, before scale+constrain
float roll_out, pitch_out, yaw_out;   // FINAL IMU OUTPUT in degrees, -180 to 180 

// global timer stuff
unsigned long currentTime, previousTime;
float dTime;                          // aka dt, deltaTime, etc

// imu error calc stuff
#define ERROR_CYCLES 1200              // set desired # of error check cycles on startup
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
int c = 0;

// complementary filter param set
#define COMP_FACTOR 0.98              // default: gyro=0.98, acc=0.02

// lowpass filter param set, tuned for 2k loop (probably dont touch)
float B_accel = 0.14;     // default: 0.14
float B_gyro = 0.1;       // default: 0.1

// P_pos low cutoff, in this case set to point at which motors actually turn on irl (~95)
const uint8_t I_CUTOFF = 100;

// ------ RADIO STUFF ------ //

// array to hold control data values
#define PACKET_SIZE 11
enum CONTROL_SIGNAL {
       L_xpos,    // left stick x
       L_ypos,    // left stick y
       L_click,   // left stick button
       R_xpos,    // right stick x
       R_ypos,    // right stick y
       R_click,   // right stick button
       P_pos,     // slider (pot) postion
       A_click,   // select button
       B_click,   // back button
       C_click,   // up button
       D_click    // down button
     };
byte data_in[PACKET_SIZE], data_translated[PACKET_SIZE];

// function to set all control data to defaults
void setRadioFailsafe() {
  // set all data to initial defaults
  data_in[L_xpos] = 127;
  data_in[L_ypos] = 127;
  data_in[L_click] = 0;
  data_in[R_xpos] = 127;
  data_in[R_ypos] = 127;
  data_in[R_click] = 0;
  data_in[P_pos] = 0;
  data_in[A_click] = 0;
  data_in[B_click] = 0;
  data_in[C_click] = 0;
  data_in[D_click] = 0;
}

// ------ PID TUNING MODE / TELEMETRY STUFF ------ //
bool PID_TUNE_ON = false;


typedef struct PID_data_t {
  float p_val;
  float i_val;
  float d_val;
};

typedef union PID_packet_t {
  PID_data_t data;
  uint8_t bytes_in[sizeof(PID_data_t)];
};
PID_packet_t pid_in;
#define PID_PACKET_SIZE sizeof(pid_in)


// ------ PID STUFF ------ //

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 20.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 20.0;     //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode 
float maxPitch = 20.0;    //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
float maxYaw = 60.0;     //Max yaw rate in deg/sec

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)


// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

// Controller:  TODO: read through and play around with controlANGLE2
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

// ------ MOTOR STUFF ------ //

int m1_out, m2_out, m3_out, m4_out, motor_cal_out = 0;
float mm1_out, mm2_out, mm3_out, mm4_out = 0;
Servo m1_servo, m2_servo, m3_servo, m4_servo;

void motor_init() {
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(M3_PIN, OUTPUT);
  pinMode(M4_PIN, OUTPUT);
  m1_servo.attach(M1_PIN);
  m2_servo.attach(M2_PIN);
  m3_servo.attach(M3_PIN);
  m4_servo.attach(M4_PIN);

  
  m1_servo.write(0);
  m2_servo.write(0);
  m3_servo.write(0);
  m4_servo.write(0);
  
}

void setMotorFailsafe() {
  m1_out = 0;
  m2_out = 0;
  m3_out = 0;
  m4_out = 0;
  
  m1_servo.write(0);
  m2_servo.write(0);
  m3_servo.write(0);
  m4_servo.write(0);
}


// ------ DEBUG / TESTING STUFF ------ //


void printIMUData() {
  Serial.print("Gyro");
  Serial.print(" X=");
  Serial.print(gX_out);
  Serial.print(" Y=");
  Serial.print(gY_out);
  Serial.print(" Z=");
  Serial.print(gZ_out);
  Serial.print(" Accel");
  Serial.print(" X=");
  Serial.print(aX_out);
  Serial.print(" Y=");
  Serial.print(aY_out);
  Serial.print(" Z=");
  Serial.println(aZ_out);
}

void plotAccel() {
  Serial.print(aX_out);
  Serial.print(",");
  Serial.print(aY_out);
  Serial.print(",");
  Serial.println(aZ_out);
}

void plotGyro() {
  Serial.print(gX_out);
  Serial.print(",");
  Serial.print(gY_out);
  Serial.print(",");
  Serial.println(gZ_out);
}

void printRollPitchYaw() {
  Serial.print(roll_out);
  Serial.print(",");
  Serial.print(pitch_out);
  Serial.print(",");
  Serial.println(yaw_out);
}

void printRadioData() {
  Serial.print("LX:");
  Serial.print(data_in[L_xpos]);
  Serial.print(" ");
  Serial.print("LY:");
  Serial.print(data_in[L_ypos]);
  Serial.print(" ");
  Serial.print("LB:");
  Serial.print(data_in[L_click]);
  Serial.print(" ");
  Serial.print("RX:");
  Serial.print(data_in[R_xpos]);
  Serial.print(" ");
  Serial.print("RY:");
  Serial.print(data_in[R_ypos]);
  Serial.print(" ");
  Serial.print("RB:");
  Serial.print(data_in[R_click]);
  Serial.print(" ");
  Serial.print("P:");
  Serial.print(data_in[P_pos]);
  Serial.print(" ");
  Serial.print("S:");
  Serial.print(data_in[A_click]);
  Serial.print(" ");
  Serial.print("B:");
  Serial.print(data_in[B_click]);
  Serial.print(" ");
  Serial.print("U:");
  Serial.print(data_in[C_click]);
  Serial.print(" ");
  Serial.print("D:");
  Serial.println(data_in[D_click]);
}

void printDesState() {
  Serial.print("THRO:");
  Serial.print(thro_des);
  Serial.print(" ROLL:");
  Serial.print(roll_des);
  Serial.print(" PITCH:");
  Serial.print(pitch_des);
  Serial.print(" YAW:");
  Serial.println(yaw_des);
}

void printStateError() {
  Serial.print("ERROR:   ROLL:");
  Serial.print(error_roll);
  Serial.print(" PITCH:");
  Serial.print(error_pitch);
  Serial.print(" YAW:");
  Serial.println(error_yaw);
}

void printPID() {
  Serial.print("PID:   ROLL:");
  Serial.print(roll_PID);
  Serial.print(" PITCH:");
  Serial.print(pitch_PID);
  Serial.print(" YAW:");
  Serial.println(yaw_PID);
}

void printPIDTuning() {
  Serial.print("Kp:");
  Serial.print(Kp_roll_angle);
  Serial.print(" Ki:");
  Serial.print(Ki_roll_angle);
  Serial.print(" Kd:");
  Serial.println(Kd_roll_angle);
}

void printMotorMix() {
  if(!data_in[B_click]) {
    Serial.print("Motors:   M1:");
    Serial.print(m1_out);
    Serial.print(" M2:");
    Serial.print(m2_out);
    Serial.print(" M3:");
    Serial.print(m3_out);
    Serial.print(" M4:");
    Serial.println(m4_out);
  } else {
    Serial.print("MOTOR CALIBRATION:  ");
    Serial.println(motor_cal_out);
  }
}

void printTerms() {
  Serial.print("PITCH:PTerm:");
  Serial.print(error_pitch);
  Serial.print(" ITerm:");
  Serial.print(integral_pitch);
  Serial.print(" DTerm");
  Serial.print(derivative_pitch);
  Serial.print("| ROLL:PTerm:");
  Serial.print(error_roll);
  Serial.print(" ITerm:");
  Serial.print(integral_roll);
  Serial.print(" DTerm");
  Serial.println(derivative_roll);
}

void graphPitchPID() {
  Serial.print(error_pitch);
  Serial.print("\t");
  Serial.print(integral_pitch);
  Serial.print("\t");
  Serial.println(derivative_pitch);
}

void graphRollPID() {
  Serial.print(error_roll);
  Serial.print("\t");
  Serial.print(integral_roll);
  Serial.print("\t");
  Serial.println(derivative_roll);
}

#endif
