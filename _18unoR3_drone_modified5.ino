// connect MPU 6050 sensor ///////////////////////////////////////////////////////////////////////////////
#include<Wire.h>

const int MPU_addr=0x68;  
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// setting MPU-6050 sensor
void initMPU6050() {
  Wire.begin();                     // call Wire.begin function to make I2C communication
  Wire.beginTransmission(MPU_addr); // call Wire.beginTransmission function which was used to communicate
                                   calibAccelGyroSetup // with I2C slave module. communicate with MPU-6050 which have 0x68 value
  Wire.write(0x6B);                 // call Wire.write function which store 1 byte sending data in memory que
  Wire.write(0);     
  Wire.endTransmission(true);       // call Wire.endTransmission function which finish transmission
                                    // to send more than one data stored in que by wire.write to slave module
}

// read accel, gyro values
void readAccelGyro() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  
  AcX=Wire.read()<<8|Wire.read();  
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  Tmp=Wire.read()<<8|Wire.read();   // you can't delete this line although you do not use this Tmp
                                    // because GyX, GyY, GyZ were readed after Tmp readed
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();

  // Serial.print("AcX = ");   Serial.print(AcX);
  // Serial.print("  | AcY = ");   Serial.print(AcY);
  // Serial.print("  | AcZ = ");   Serial.print(AcZ);
  // Serial.print("  | Tmp = ");   Serial.print(Tmp / 340.00 + 36.53);
  // Serial.print("  | GyX = ");   Serial.print(GyX);
  // Serial.print("  | GyY = ");   Serial.println(GyY);
  // Serial.print("  | GyZ = ");   Serial.println(GyZ);
}

float dt;
float accel_angle_x,    accel_angle_y,    accel_angle_z; 
//float gyro_angle_x,     gyro_angle_y,     gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
extern float roll_output, pitch_output, yaw_output; 
extern float motorA_speed, motorB_speed, motorC_speed, motorD_speed;
  
float baseAcX, baseAcY, baseAcZ;      // define acceleration variables
float baseGyX, baseGyY, baseGyZ ;     // define gyroscope variables

// calibration Accel, Gyro variables into average values of 10 times
void calibAccelGyroSetup() {
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0; 
  
  for(int i=0;i<10;i++) {
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    // delay(10);
  }
  
  baseAcX = sumAcX / 10; 
  baseAcY = sumAcY / 10; 
  baseAcZ = sumAcZ / 10; 
  
  baseGyX = sumGyX / 10; 
  baseGyY = sumGyY / 10; 
  baseGyZ = sumGyZ / 10;   

  // Serial.print("AcX = ");   Serial.print(baseAcX);
  // Serial.print("  | AcY = ");   Serial.print(baseAcY);
  // Serial.print("  | AcZ = ");   Serial.print(baseAcZ);
  // Serial.print("  | GyX = ");   Serial.print(baseGyX);
  // Serial.print("  | GyY = ");   Serial.print(baseGyY);
  // Serial.print("  | GyZ = ");   Serial.println(baseGyZ);
}

// calibration Accel, Gyro variables into average values of 10 times
void calibAccelGyroLoop() {
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0; 
  
  for(int i=0;i<10;i++) {
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    // delay(10);
  }
  
  AcX = sumAcX / 10; 
  AcY = sumAcY / 10; 
  AcZ = sumAcZ / 10; 
  
  GyX = sumGyX / 10; 
  GyY = sumGyY / 10; 
  GyZ = sumGyZ / 10;   

  // Serial.print("AcX = ");   Serial.print(AcX);
  // Serial.print("  | AcY = ");   Serial.print(AcY);
  // Serial.print("  | AcZ = ");   Serial.print(AcZ);
  // Serial.print("  | GyX = ");   Serial.print(GyX);
  // Serial.print("  | GyY = ");   Serial.print(GyY);
  // Serial.print("  | GyZ = ");   Serial.println(GyZ);
  // delay(50);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// control PID ///////////////////////////////////////////////////////////////////////////////////////////

unsigned long t_now;        // setting current time variable
unsigned long t_prev;       // setting preview time variable

// initialize t_prev variable using mrcros function
void initDT() {
  t_prev = micros();
}

// setting time interval and check current time
void calcDT() {
  t_now = micros();
  dt = (t_now - t_prev)/1000000.0;
  t_prev = t_now;

  // Serial.print("t_prev: "); Serial.print(t_prev);
  // Serial.print(" | t_now: "); Serial.print(t_now);
  // Serial.print(" | dt: "); Serial.print(dt*1000);
}

// calculate Roll, Pitch, Yaw's Degrees using Acceleration values
void calcAccelYPR() {
  float accel_x, accel_y, accel_z;
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180/3.14159;
  
  accel_x = AcX - baseAcX;    // calibrate accel sensor value to subtract initial average value
  accel_y = AcY - baseAcY;    // from current value
  accel_z = AcZ + (16384 - baseAcZ);  // calibrate z accel sensor value. ideal value of z is 16384
                                      // so subtract baseAcZ from 16384
  
  accel_yz = sqrt(pow(accel_y,2) + pow(accel_z,2));             // calculate Tilted angle on x axis
  accel_angle_y = atan(-accel_x/accel_yz)*RADIANS_TO_DEGREES;   // accel_angle_y is Pitch
                                                                // gravity direction is (+)
  
  accel_xz = sqrt(pow(accel_x,2) + pow(accel_z,2));             // calculate Tilted angle on y axis
  accel_angle_x = atan( accel_y/accel_xz)*RADIANS_TO_DEGREES;   // accel_angle_y is Roll
                                                                // gravity direction is (+)
  
  accel_angle_z = 0;    // set accel_angle_z value into 0

  // Serial.print("baseAcX: "); Serial.print(baseAcX);
  // Serial.print(" | baseAcY: "); Serial.print(baseAcY);
  // Serial.print(" | baseAcZ: "); Serial.print(baseAcZ);
  // Serial.print(" || accel_x: "); Serial.print(accel_x);
  // Serial.print(" | accel_y: "); Serial.print(accel_y);
  // Serial.print(" | accel_z: "); Serial.println(accel_z);
  // Serial.print("accel_angle_x: "); Serial.print(accel_angle_x);
  // Serial.print(" | accel_angle_y: "); Serial.println(accel_angle_y);
  // Serial.print(" | accel_angle_z: "); Serial.println(accel_angle_z);
}

float gyro_x, gyro_y, gyro_z;   // define gyro variables

// calculate Roll, Pitch, Yaw's Degrees using Gyroscope values
void calcGyroYPR() {  
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;   // 250 degree/second : 32767 = 1 degree/second : x, x = 131

  // gyro values convert into Degree/Second
  gyro_x = (GyX - baseGyX)/GYROXYZ_TO_DEGREES_PER_SEC;    // convert angular velocity after calibration
  gyro_y = (GyY - baseGyY)/GYROXYZ_TO_DEGREES_PER_SEC;    // by subtracting initial average value from
  gyro_z = (GyZ - baseGyZ)/GYROXYZ_TO_DEGREES_PER_SEC;    // current gyroscope sensor values

//  gyro_angle_x += gyro_x * dt;    // add angular velocity * dt into current angular velocity
//  gyro_angle_y += gyro_y * dt;    // add angular velocity * dt into current angular velocity
//  gyro_angle_z += gyro_z * dt;    // add angular velocity * dt into current angular velocity

  // Serial.print("gyro_x: "); Serial.print(gyro_x);
  // Serial.print(" | gyro_y: "); Serial.print(gyro_y);
  // Serial.print(" | gyro_z: "); Serial.println(gyro_z);
}

// calculate Roll, Pitch, Yaw's Degrees using Filtered values
void calcFilteredYPR() {
  const float ALPHA = 0.96;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;
  
  tmp_angle_x = filtered_angle_x + gyro_x * dt;   // assign filtered angle and gyroscope angle
  tmp_angle_y = filtered_angle_y + gyro_y * dt;   // into temporary angle
  tmp_angle_z = filtered_angle_z + gyro_z * dt;
  
  filtered_angle_x =    // place 0.96 weight on temporary angle and 0.04 on acceleration angle
    ALPHA * tmp_angle_x + (1.0-ALPHA) * accel_angle_x;    // then set current filtered angle
  filtered_angle_y = 
    ALPHA * tmp_angle_y + (1.0-ALPHA) * accel_angle_y;    
  filtered_angle_z = tmp_angle_z;   // set current filtered angle only using gyroscope sensor value

  // Serial.print("filtered_angle_x: "); Serial.print(filtered_angle_x);
  // Serial.print(" | filtered_angle_y: "); Serial.print(filtered_angle_y);
  // Serial.print(" | filtered_angle_z: "); Serial.println(filtered_angle_z);
}

float base_roll_target_angle;
float base_pitch_target_angle;
float base_yaw_target_angle;

extern float roll_target_angle;
extern float pitch_target_angle;
extern float yaw_target_angle;

// initialize Yaw, Pitch, Roll. set average value of 10 times
void initYPR() {
  
  for(int i=0;i<10;i++) {
    readAccelGyro();
    calcDT();
    calcAccelYPR();  
    calcGyroYPR();    
    calcFilteredYPR(); 
  
    base_roll_target_angle += filtered_angle_x;
    base_pitch_target_angle += filtered_angle_y;
    base_yaw_target_angle += filtered_angle_z;

    // delay(100);
  }
  
  base_roll_target_angle /= 10;
  base_pitch_target_angle /= 10;
  base_yaw_target_angle /= 10;
  
  roll_target_angle = base_roll_target_angle;
  pitch_target_angle = base_pitch_target_angle;
  yaw_target_angle = base_yaw_target_angle;

  // Serial.print("roll_target_angle: "); Serial.print(roll_target_angle);
  // Serial.print(" | pitch_target_angle: "); Serial.print(pitch_target_angle);
  // Serial.print(" | yaw_target_angle: "); Serial.println(yaw_target_angle);
}

void dualPID(
          float target_angle,
          float angle_in,
          float rate_in,
          float stabilize_kp, 
          float stabilize_ki, 
          float rate_kp,
          float rate_ki,
          float& stabilize_iterm,
          float& rate_iterm,
          float& output
          ) {
  float angle_error;
  float desired_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm;  
  
  angle_error = target_angle - angle_in;  // get current angle error to subtract current angle
                                          // from goal angle value
                                          
  stabilize_pterm  = stabilize_kp * angle_error;    // get stable proportional value to multiply
                                                    // stable proportional parameter with current angle error
  stabilize_iterm += stabilize_ki * angle_error * dt;   // get stable integral value to multiply
                                                    // stable integral parameter with current angle error
                                                    // and sensor input cycle. then accumulate thouse
  
  desired_rate = stabilize_pterm;   // set goal angluar velocity into stable proportional value.  

  rate_error = desired_rate - rate_in;    // get current angular velocity error to subtract current input
                                          // anglur velocity from goal angular veloity value  
  
  rate_pterm  = rate_kp * rate_error;   // get ancular velocity proportional value to multiply current
                                        // angular velocity error with angular velocity proportional parameter
  rate_iterm += rate_ki * rate_error * dt;  // get angular velocity integral value to multiply
                                        // current angular velocity error and sensor input cycle with
                                        // angular velocity integral parameter
  
  output = rate_pterm + rate_iterm + stabilize_iterm; // get output value to add angular velocity proportional
                                        // and angular velocity integral, stable integral values
}

float roll_target_angle = 0.0;    // variable to store target angle value of the roll
float roll_angle_in;              // variable to store current input angle value of the roll
float roll_rate_in;               // variable to store current input angular velocity value of the roll
float roll_stabilize_kp = 0.03;      // stable proportional parameter of the roll. initialize into 1
float roll_stabilize_ki = 0.01;      // stable integral parameter of the roll. initialize into 0
float roll_rate_kp = 0.03;           // angular velocity propotional parameter of the roll. initialize into 1
float roll_rate_ki = 0.01;           // angular velocity integral parameter of the roll. initialize into 0
float roll_stabilize_iterm; // variable to store accumulated values of the stable integral of the roll
float roll_rate_iterm; // variable to store accumulated values of the angular velocity integral of the roll
float roll_output;                // variable to store output value of the roll

float pitch_target_angle = 0.0;   // variable to store target angle value of the pitch
float pitch_angle_in;
float pitch_rate_in;
float pitch_stabilize_kp = 0.03;
float pitch_stabilize_ki = 0.01;
float pitch_rate_kp = 0.03;
float pitch_rate_ki = 0.01;
float pitch_stabilize_iterm;
float pitch_rate_iterm;
float pitch_output;

float yaw_target_angle = 0.0;     // variable to store target angle value of the yaw
float yaw_angle_in;
float yaw_rate_in;
float yaw_stabilize_kp = 0.03;
float yaw_stabilize_ki = 0.01;
float yaw_rate_kp = 0.03;
float yaw_rate_ki = 0.01;
float yaw_stabilize_iterm;
float yaw_rate_iterm;
float yaw_output;

void calcYPRtoDualPID() {
  
  roll_angle_in = filtered_angle_x;   // use filtered_angly_y for roll's angular input
  roll_rate_in = gyro_x;              // use gyro_y for roll's angular velocity input
  dualPID(  roll_target_angle,        // get roll's output using dualPID function
            roll_angle_in,
            roll_rate_in,
            roll_stabilize_kp, 
            roll_stabilize_ki,
            roll_rate_kp,
            roll_rate_ki, 
            roll_stabilize_iterm,
            roll_rate_iterm,
            roll_output);
            
  pitch_angle_in = filtered_angle_y;  // use filtered_angly_x for pitch's angular input
  pitch_rate_in = gyro_y;             // use gyro_x for pitch's angular velocity input
  dualPID(  pitch_target_angle,       // get pitch's output using dualPID function
            pitch_angle_in,
            pitch_rate_in,
            pitch_stabilize_kp, 
            pitch_stabilize_ki,
            pitch_rate_kp,
            pitch_rate_ki, 
            pitch_stabilize_iterm,
            pitch_rate_iterm,
            pitch_output);
            
  yaw_angle_in = filtered_angle_z;    // use filtered_angly_x for yaw's angular input
  yaw_rate_in = gyro_z;               // use gyro_x for yaw's angular velocity input
  dualPID(  yaw_target_angle,         // get yaw's output using dualPID function
            yaw_angle_in,
            yaw_rate_in,
            yaw_stabilize_kp, 
            yaw_stabilize_ki,
            yaw_rate_kp,
            yaw_rate_ki, 
            yaw_stabilize_iterm,
            yaw_rate_iterm,
            yaw_output);

  // Serial.print("roll_output: "); Serial.print(roll_output);
  // Serial.print(" | pitch_output: "); Serial.print(pitch_output);
  // Serial.print(" | yaw_output: "); Serial.println(yaw_output);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Motor settings //////////////////////////////////////////////////////////////////////////////////////
#define THROTTLE_MAX 255    // setting MAX motor speed variable
#define THROTTLE_MIN 0      // setting MIN motor speed variable

int motorA_pin = 3; //6;     // settings motor pins
int motorB_pin = 9;          // A, C: CW | B, D: CCW
int motorC_pin = 10;
int motorD_pin = 11; //5;

// initialize Motor speed into THROTTLE_MIN
void initMotorSpeed() {

  Serial.println("initMotorMax");
  for(int i=0;i<100;i++) {
    analogWrite(motorA_pin, THROTTLE_MAX);
    analogWrite(motorB_pin, THROTTLE_MAX);
    analogWrite(motorC_pin, THROTTLE_MAX);
    analogWrite(motorD_pin, THROTTLE_MAX);
    delay(29);
    analogWrite(motorA_pin, THROTTLE_MIN);
    analogWrite(motorB_pin, THROTTLE_MIN);
    analogWrite(motorC_pin, THROTTLE_MIN);
    analogWrite(motorD_pin, THROTTLE_MIN);
    delay(1);
  }
  Serial.println("initMotorMin");
  for(int i=0;i<100;i++) {
    analogWrite(motorA_pin, THROTTLE_MAX);
    analogWrite(motorB_pin, THROTTLE_MAX);
    analogWrite(motorC_pin, THROTTLE_MAX);
    analogWrite(motorD_pin, THROTTLE_MAX);
    delay(1);
    analogWrite(motorA_pin, THROTTLE_MIN);
    analogWrite(motorB_pin, THROTTLE_MIN);
    analogWrite(motorC_pin, THROTTLE_MIN);
    analogWrite(motorD_pin, THROTTLE_MIN);
    delay(29);
  }
}

// update Motor speed using calcMotorSpeed()
// after adding function about filter and PID, add 'update Motor Speed'
void updateMotorSpeed() {  
    
  // Serial.println("updateMotorSpeed");
  analogWrite(motorA_pin, motorA_speed);
  analogWrite(motorB_pin, motorB_speed);
  analogWrite(motorC_pin, motorC_speed);
  analogWrite(motorD_pin, motorD_speed);

  // Serial.print("  || motorA = ");   Serial.print(motorA_speed);
  // Serial.print("  | motorB = ");   Serial.print(motorB_speed);
  // Serial.print("  | motorC = ");   Serial.print(motorC_speed);
  // Serial.print("  | motorD = ");   Serial.println(motorD_speed);
  // delay(100);
  
  // analogWrite(motorA_pin, 140); // it start from 140 to 255
  // analogWrite(motorB_pin, 140);
  // analogWrite(motorC_pin, 140);
  // analogWrite(motorD_pin, 140);
}

float throttle = 175;     // define throttle variable
float motorA_speed, motorB_speed, motorC_speed, motorD_speed;     // define motor's speed variables

float setMotorA = 0;
float setMotorB = 18.7;
float setMotorC = 33.9;
float setMotorD = 10.9;

// calculate motor speed using Roll, Pitch, Yaw values from cylcYPRtoStdPID function
void calcMotorSpeed() {
  // Serial.println("calcMotorSpeed");
  motorA_speed = (throttle == 0) ? 0:
    throttle + roll_output - pitch_output + yaw_output + setMotorA;
  motorB_speed = (throttle == 0) ? 0:
    throttle - roll_output - pitch_output - yaw_output + setMotorB;
  motorC_speed = (throttle == 0) ? 0:
    throttle - roll_output + pitch_output + yaw_output + setMotorC;
  motorD_speed = (throttle == 0) ? 0:
    throttle + roll_output + pitch_output - yaw_output + setMotorD;

  if(motorA_speed < 0) motorA_speed = 0;      // calibrate motor's speed between 0 and 255
  if(motorA_speed > 255) motorA_speed = 255;  // because arduino pwm output value is 0 to 255
  if(motorB_speed < 0) motorB_speed = 0;      // it defence unexpected work
  if(motorB_speed > 255) motorB_speed = 255;
  if(motorC_speed < 0) motorC_speed = 0; 
  if(motorC_speed > 255) motorC_speed = 255;
  if(motorD_speed < 0) motorD_speed = 0; 
  if(motorD_speed > 255) motorD_speed = 255;
  
  Serial.print("throttle = ");   Serial.print(throttle);
  Serial.print(" | roll_output = ");   Serial.print(roll_output);      // x, y axis's rotation direction is -
  Serial.print(" | pitch_output = ");   Serial.print(pitch_output); // x, y axis's oponent rotation direction is +
  Serial.print(" | yaw_output = ");   Serial.print(yaw_output);
  // delay(100);

  Serial.print(" || motorA_speed: "); Serial.print(motorA_speed);
  Serial.print(" | motorB_speed: "); Serial.print(motorB_speed);
  Serial.print(" | motorC_speed: "); Serial.print(motorC_speed);
  Serial.print(" | motorD_speed: "); Serial.println(motorD_speed);
  // delay(50);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Motor Settings
  initMotorSpeed();          // initialize Motor speed into THROTTLE_MIN
  
  // Filter Setting
  initMPU6050();             // setting MPU-6050
  Serial.begin(115200);     // start serial communication
  calibAccelGyroSetup();          // calibration Accel, Gyro variables into average values of 10 times
  
  // PID Settings
  initDT();                  // initialize t_prev variable using mrcros function
  initYPR();
}

void loop() {
  // Filter settings
  // calibAccelGyro();          // calibration Accel, Gyro variables into average values of 10 times
  // readAccelGyro();      // read accel, gyro values
  calibAccelGyroLoop(); // read accel, gyro values after calibration
  calcDT();             // setting time interval and check no time
  calcAccelYPR();       // calculate Roll, Pitch, Yaw's Degrees using Acceleration values
  calcGyroYPR();        // calculate Roll, Pitch, Yaw's Degrees using Gyroscope values
  calcFilteredYPR();    // calculate Roll, Pitch, Yaw's Degrees using Filtered values
  calcYPRtoDualPID();   // calculate YPR using dual PID function
  
  // Serial.print(" || angley_y: "); Serial.print(filtered_angle_y);
  // Serial.print("| angley_x: "); Serial.print(filtered_angle_x);
  
  // Serial.print("| angley_z: "); Serial.print(filtered_angle_z);
  // Serial.print(" || ");
  // Serial.print("| MotorA: "); Serial.print(motorA_speed);
  // Serial.print("| MotorA: "); Serial.print(motorB_speed);
  // Serial.print("| MotorC: "); Serial.print(motorC_speed);
  // Serial.print("| MotorD: "); Serial.println(motorD_speed);

  // Motor Settings
  // calcMotorSpeed();
  calcMotorSpeed();     // calculate motor speed using Roll, Pitch, Yaw values from cylcYPRtoStdPID function  
  updateMotorSpeed();   // update Motor speed using calcMotorSpeed()

  Serial.print("t_now: "); Serial.print(t_now);
  Serial.print(" | ");
}
