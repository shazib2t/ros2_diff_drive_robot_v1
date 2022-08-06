/**
  ROS2 differential drive robot 
  Name: Robot firmware
  Purpose: Differential drive robot for ros2 navigation and other ros2 based tasks.

  @author Mohammad Manzur Murshid
  @version 1.0 07/14/2022
  
    Copyright <2022> <Mohammad Manzur Murshid>

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions 
are met:
1. Redistributions of source code must retain the above copyright 
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright 
   notice, this list of conditions and the following disclaimer in 
   the documentation and/or other materials provided with the 
   distribution.
3. Neither the name of the copyright holder nor the names of its 
   contributors may be used to endorse or promote products derived 
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
*/


#include <micro_ros_arduino.h>
#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
//MPU9250 mpu;

//#include "MPU9250.h"
//MPU9250 mpu;

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
//MMM
#include <geometry_msgs/msg/twist.h>
//MMM 11-19-2021
#include <std_msgs/msg/header.h> //comes with imu sensor
//#include <std_msgs/msg/string.h> //comes with imu sensor

//#include <rosidl_runtime_c/string.h>
//#include <rosidl_runtime_c/string_functions.h>

#include <sensor_msgs/msg/imu.h>  //comes with imu sensor
#include <geometry_msgs/msg/quaternion_stamped.h> //comes with imu sensor
#include <geometry_msgs/msg/vector3_stamped.h> //comes with imu sensor 
#include <geometry_msgs/msg/vector3.h> //comes with imu sensor 
#include <nav_msgs/msg/odometry.h> //for publishing odom msgs



//MMM
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;


rcl_publisher_t publisher;  //this publisher is for /tf
rcl_publisher_t publisher_i;  //this publisher is for /imu
rcl_publisher_t publisher_nav;  //this publisher is for /odom
tf2_msgs__msg__TFMessage * tf_message;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
//MMM 11-19-2021
sensor_msgs__msg__Imu * imu; //adding to publish imu sensor data
nav_msgs__msg__Odometry  *odometry; //this is for odom


//MMM 07-19-2022--------------DMP imu from 6050----------
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

int16_t ax, ay, az;
int16_t gx, gy, gz;

float ypr[3];
float yaw;
//MMM
#define PWM_MIN 0
#define PWMRANGE 255

// Declare functions
void setupPins();
float mapPwm(float x, float out_min, float out_max);

//Pins for motors
#define left_back 3
#define left_front 4
#define right_front 5
#define right_back 6
#define nD2 7
#define nSF 8

const int R = 0.061;  //Radius of wheel in meter
const int L = 0.235;  //Length of wheel base in meter

/*----This is encoder section--------------*/
/*-----------------------------------------*/
/*-----------------------------------------*/
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 22
#define ENC_IN_RIGHT_A 9

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 21
#define ENC_IN_RIGHT_B 10

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;
 
// One-second interval for measurements
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
/*-----------------------------------------*/
/*-----------------------------------------*/
/*----This is end of encoder section-------*/

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);
//cIMU    IMU;


//MMM----------------------------------------------------
//-------------velocity and position valiables-----------



float x = 0.0;
float y = 0.0;
float d = 0.0;
float th = 0.0;
float delta_d = 0.0;
float delta_th = 0.0;
float th_plus = 0.0;
float d_x = 0.0;
float d_y = 0.0;

float position_z = 0.0;
float deltaTime = 0.0;
float current_time = 0.0;
float last_time = 0.0;


volatile int tick_x = 0;
volatile int tick_y = 0;
float deltaLeft = 0;
float deltaRight = 0;
volatile int _PreviousLeftEncoderCounts = 0;
volatile int _PreviousRightEncoderCounts = 0;
double DistancePerCount = (2* 3.14159265 * 0.06) / 150; //radius = 60mm = 0.06
double lengthBetweenTwoWheels = 0.24; //240mm

//double deltaTime = 0.0;
unsigned long prev_odom_update = 0;

//calcuate time offset from ros agent to robot
unsigned long long time_offset = 0;

//MMM this function will calculate the position and velocity from encoder data 

void get_pos_vel_for_odom(){
  struct timespec tv = {0};
  clock_gettime(0, &tv);
  current_time = tv.tv_nsec;//.tv_sec;
  deltaTime = (current_time - last_time); // delta time to find the position and velocity
  
  //tick counter
  tick_x = left_wheel_tick_count;
  tick_y = right_wheel_tick_count;
  

  // extract the wheel velocities from the tick signals count
  deltaLeft  = (tick_x - _PreviousLeftEncoderCounts) * DistancePerCount;
  deltaRight = (tick_y - _PreviousRightEncoderCounts) * DistancePerCount;

  //distance traveled is the average of the two wheels 
  d = (deltaLeft + deltaRight)/2;
  //this approximation works (in radians) for small angles
  th = ((deltaRight - deltaLeft)/lengthBetweenTwoWheels);

  //calculate velocities
  delta_d = d / (deltaTime/1000000000);
  delta_th = th / (deltaTime/1000000000);


  if (d != 0){
    //calculate distance traveled in x and y
    x = cos(th) *d;
    y = -sin(th) *d;
    
    //calculate the final position of the robot
    
    d_x +=  cos(th_plus) * x - sin(th_plus) * y;
    d_y +=  sin(th_plus) * x + cos(th_plus) * y;
  }

  if (th!=0){
    th_plus += th;
    
  }
  
  _PreviousLeftEncoderCounts  = tick_x;
  _PreviousRightEncoderCounts = tick_y;

  last_time = current_time;  
  
}


//Calculate the quartenion from the angles
const void euler_to_quat(float x, float y, float z, double* q) {
    float c1 = cos((y*3.14159265/180.0)/2);
    float c2 = cos((z*3.14159265/180.0)/2);
    float c3 = cos((x*3.14159265/180.0)/2);

    float s1 = sin((y*3.14159265/180.0)/2);
    float s2 = sin((z*3.14159265/180.0)/2);
    float s3 = sin((x*3.14159265/180.0)/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
}


//MMM--------------------------------
void setupPins()
{
  // Status motor
  pinMode(LED_PIN, OUTPUT);  
  pinMode(left_back, OUTPUT);
  pinMode(left_front, OUTPUT);
  pinMode(right_front, OUTPUT);
  pinMode(right_back, OUTPUT);
  pinMode(nD2, OUTPUT);
  digitalWrite(nD2, HIGH); // default to on
  pinMode(nSF, INPUT);
  stop();
}

void stop()
{
  analogWrite(left_back, 0);
  analogWrite(left_front, 0);
  analogWrite(right_front, 0);
  analogWrite(right_back, 0);
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);

  // Cap values at [-1 .. 1]
  //float x = max(min(msg->linear.x, 1.0f), -1.0f);
  //float z = max(min(msg->angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  //float l = (2*x - z*L)/2*R;
  //float r = (2*x + z*L)/2*R ; 
  
  float l = (msg->linear.x - msg->angular.z) / 2;
  float r = (msg->linear.x + msg->angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

  // Set direction pins and PWM

  if (l < 0){
    analogWrite(left_front, 0);
    analogWrite(left_back, lPwm);
  }
  else if (l > 0){
    analogWrite(left_back, 0);
    analogWrite(left_front, lPwm);
  }
  if (r < 0){
    analogWrite(right_front, 0);
    analogWrite(right_back, rPwm);
  }
  else if (r > 0){
    analogWrite(right_back, 0);
    analogWrite(right_front, rPwm);
  }
  else{
    stop();
  }
  
  
  //analogWrite(right_front, lPwm);
  //analogWrite(right_back, rPwm);
  //analogWrite(left_front, lPwm);
  //analogWrite(left_back, rPwm);
}


// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

//-------------------------------------



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



void setup() {
  set_microros_transports();
  setupPins();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  /*----This is encoder section--------------*/
  /*-----------------------------------------*/
  /*-----------------------------------------*/
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  /*-----------------------------------------*/
  /*-----------------------------------------*/
  /*----This is end of encoder section-------*/

  
  Wire.begin();

  //MMM
  /*
  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
  */
  // calibrate anytime you want to
  //mpu.calibrateAccelGyro();
  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);  

  /*-------------------------------new DMP for imu---------------------*/
  /*----------IMU 6050 DMP code working for 9250, so using it here ----*/
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      //Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  }else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
  }
        
  /*----------------------end of DMP code-------------------------*/
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  //MMM-----------------------------------------
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    //"micro_ros_arduino_twist_subscriber"));
    "cmd_vel"));

    //----------------------------------

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));


  //MMM 11-19-2021
  //create sensor msgs publisher
  
  RCCHECK(rclc_publisher_init_default(
    &publisher_i,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data"));


   //MMM 12-12-2021
  //create nav msgs publisher
  
  RCCHECK(rclc_publisher_init_default(
    &publisher_nav,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom/unfiltered"));
  

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

    //syntime with microros
    syncTime();

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  //MMM--------------------
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  //-----------------------------------------------------------------
  //--------------------------tf data--------------------------------
  tf_message = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 1);

  tf_message->transforms.data[0].header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string1[] = "map";
  memcpy(tf_message->transforms.data[0].header.frame_id.data, string1, strlen(string1) + 1);
  tf_message->transforms.data[0].header.frame_id.size = strlen(tf_message->transforms.data[0].header.frame_id.data);
  tf_message->transforms.data[0].header.frame_id.capacity = 100;

  char string2[] = "odom";
  tf_message->transforms.data[0].child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(tf_message->transforms.data[0].child_frame_id.data, string2, strlen(string2) + 1);
  tf_message->transforms.data[0].child_frame_id.size = strlen(tf_message->transforms.data[0].child_frame_id.data);
  tf_message->transforms.data[0].child_frame_id.capacity = 100;
  //-----------------------------------------------------------------
  //--------------------------END of tf data--------------------------------

  //-----------------------------------------------------------------
  //--------------------------Imu data--------------------------------
  
  imu = sensor_msgs__msg__Imu__create();
  std_msgs__msg__Header__init(&imu->header);
  //geometry_msgs__msg__QuaternionStamped__Sequence__init(&imu->orientation, 1);
  //geometry_msgs__msg__Vector3Stamped__Sequence__init(&imu->angular_velocity, 1);
  //geometry_msgs__msg__Vector3Stamped__Sequence__init(&imu->linear_acceleration, 1);
  
  //imu->header.frame_id.data= "/imu";
  
  imu->header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string[] = "imu_link";
  memcpy(imu->header.frame_id.data, string, strlen(string) + 1);
  imu->header.frame_id.size = strlen(imu->header.frame_id.data);
  imu->header.frame_id.capacity = 100;
  

  //-----------------------------------------------------------------
  //--------------------------END of imu data--------------------------------


  //-----------------------------------------------------------------
  //--------------------------Odometry data--------------------------------
  
  odometry = nav_msgs__msg__Odometry__create();
  
  //std_msgs__msg__Header__init(&odometry->header);
  //rosidl_runtime_c__String__init(&odometry->child_frame_id);
  //std_msgs__msg__String__init(&odometry->child_frame_id, 1);
  //geometry_msgs__msg__TransformStamped__Sequence__init(&odometry->transforms, 1);
  //geometry_msgs__msg__QuaternionStamped__Sequence__init(&imu->orientation, 1);
  //geometry_msgs__msg__Vector3Stamped__Sequence__init(&imu->angular_velocity, 1);
  //geometry_msgs__msg__Vector3Stamped__Sequence__init(&imu->linear_acceleration, 1);
  
  //imu->header.frame_id.data= "/imu";
  
  odometry->header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string_odom[] = "odom";
  memcpy(odometry->header.frame_id.data, string_odom, strlen(string_odom) + 1);
  odometry->header.frame_id.size = strlen(odometry->header.frame_id.data);
  odometry->header.frame_id.capacity = 100;

  char string_odom_2[] = "base_footprint";
  odometry->child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(odometry->child_frame_id.data, string_odom_2, strlen(string_odom_2) + 1);
  odometry->child_frame_id.size = strlen(odometry->child_frame_id.data);
  odometry->child_frame_id.capacity = 100;
  

  //-----------------------------------------------------------------
  //--------------------------END of odometry data--------------------------------
  
  
}

/*----This is encoder section--------------*/
/*-----------------------------------------*/
/*-----------------------------------------*/

// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count == encoder_maximum) {
      right_wheel_tick_count = encoder_minimum;
    }
    else {
      right_wheel_tick_count--;  
    }    
  }
  else {
    if (right_wheel_tick_count == encoder_minimum) {
      right_wheel_tick_count = encoder_maximum;
    }
    else {
      right_wheel_tick_count++;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count == encoder_maximum) {
      left_wheel_tick_count = encoder_minimum;
    }
    else {
      left_wheel_tick_count--;  
    }  
  }
  else {
    if (left_wheel_tick_count == encoder_minimum) {
      left_wheel_tick_count = encoder_maximum;
    }
    else {
      left_wheel_tick_count++;  
    }   
  }
}

/*-----------------------------------------*/
/*-----------------------------------------*/
/*----This is end of encoder section-------*/


/*-----------------------------------------*/
/*-----------------------------------------*/
/*----Timer section------------------------*/

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

/*-----------------------------------------*/
/*-----------------------------------------*/
/*----This is end of timer section-------*/




//MMM Imu loop function
void imu_pub(){
  //MMM 11-19-2021
  //struct timespec tv = {0};
  //clock_gettime(0, &tv);

  struct timespec time_stamp = getTime();

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  
  imu->orientation.x = q.x;//(double) mpu.getQuaternionX(); //q[2];
  imu->orientation.y = q.y;//(double) mpu.getQuaternionY();  //q[1];
  imu->orientation.z = q.z;//(double) mpu.getQuaternionZ();  //q[3]; 
  imu->orientation.w = q.w;//(double) mpu.getQuaternionW();  //q[0];

  imu->angular_velocity.x = gx;//(double) mpu.getGyroX(); 
  imu->angular_velocity.y = gy;//(double) mpu.getGyroY(); 
  imu->angular_velocity.z = gz;//(double) mpu.getGyroZ();   

  imu->linear_acceleration.x = aaReal.x;//(double) mpu.getAccX(); 
  imu->linear_acceleration.y = aaReal.y;//(double) mpu.getAccY(); 
  imu->linear_acceleration.z = aaReal.z;//(double) mpu.getAccZ();  
  
  imu->header.stamp.nanosec = time_stamp.tv_nsec;
  imu->header.stamp.sec = time_stamp.tv_sec;
}

//MMM tf loop function
void tf_pub(){
  //struct timespec tv = {0};
  //clock_gettime(0, &tv);
  struct timespec time_stamp = getTime();
  //euler_to_quat(mpu.getRoll()- 9.50, mpu.getPitch() -1.80, mpu.getYaw() - 40.50, q);

  mpu.dmpGetQuaternion(&q, fifoBuffer);

  float rotation_z = sin(th_plus/2);
  float rotation_w = cos(th_plus/2);
  
  tf_message->transforms.data[0].header.stamp.nanosec = time_stamp.tv_nsec;
  tf_message->transforms.data[0].header.stamp.sec = time_stamp.tv_sec;
  tf_message->transforms.data[0].transform.translation.x = d_x; //(double) mpu.getEulerX();//mpu.getPitch();
  tf_message->transforms.data[0].transform.translation.y = d_y; //(double) mpu.getEulerY();//mpu.getRoll();
  tf_message->transforms.data[0].transform.translation.z = 0; //(double) mpu.getEulerZ();//mpu.getYaw();

  tf_message->transforms.data[0].transform.rotation.x = q.x;//q[1];//(float) mpu.getQuaternionX();// /(180/3.14159265);  //(double) q[1]; //mpu.getQuaternionW(); //q[2];
  tf_message->transforms.data[0].transform.rotation.y = q.y;//-q[2];//-(float) mpu.getQuaternionY();// /(180/3.14159265);  //(double) q[2]; //mpu.getQuaternionX();  //q[1];
  tf_message->transforms.data[0].transform.rotation.z = rotation_z; //q.z;//-q[3];//-(float) mpu.getQuaternionZ(); // /(180/3.14159265);  //(double) q[3]; //mpu.getQuaternionY();  //q[3]; 
  tf_message->transforms.data[0].transform.rotation.w = rotation_w; //q.w;//q[0];//(float) mpu.getQuaternionW(); // /(180/3.14159265);  //(double) q[0]; //mpu.getQuaternionZ();  //q[0];
  
  
}


//MMM nav loop function
void nav_pub(){

  get_pos_vel_for_odom();
  //struct timespec tv = {0};
  //clock_gettime(0, &tv);
  struct timespec time_stamp = getTime();

  //euler_to_quat(mpu.getRoll()- 9.50, mpu.getPitch() -1.80, mpu.getYaw() - 40.50, q);

  mpu.dmpGetQuaternion(&q, fifoBuffer);

  float rotation_z = sin(th_plus/2);
  float rotation_w = cos(th_plus/2);
   
  odometry->pose.pose.position.x = d_x;
  odometry->pose.pose.position.y = d_y;
  odometry->pose.pose.position.z = 0.0; //(double) position_x;
  odometry->pose.pose.orientation.x = q.x;//q[1];//(float) mpu.getQuaternionX();
  odometry->pose.pose.orientation.y = q.y;//-q[2];//-(float) mpu.getQuaternionY();
  odometry->pose.pose.orientation.z = rotation_z; //q.z;//-q[3];//-(float) mpu.getQuaternionZ();
  odometry->pose.pose.orientation.w = rotation_w; //q.w;//q[0];//(float) mpu.getQuaternionW();

  //odometry->pose.covariance[0] = 0.001;
  //odometry->pose.covariance[7] = 0.001;
  //odometry->pose.covariance[35] = 0.001;
  odometry->twist.twist.linear.x = delta_d; 
  odometry->twist.twist.linear.y = 0.0; 
  odometry->twist.twist.linear.z = 0.0; // (double) velocity_z;

  odometry->twist.twist.angular.x = 0.0; //(double) velocity_x; 
  odometry->twist.twist.angular.y = 0.0; //(double) velocity_y; 
  odometry->twist.twist.angular.z = delta_th; //vth; // (double) velocity_z;

  //odometry->twist.covariance[0] = 0.001;
  //odometry->twist.covariance[7] = 0.001;
  //odometry->twist.covariance[35] = 0.001;
  
  odometry->header.stamp.nanosec = time_stamp.tv_nsec;
  odometry->header.stamp.sec = time_stamp.tv_sec;
  
}

void loop() {

  //delay(100);
  
  
  //struct timespec tv = {0};
  //clock_gettime(0, &tv);
  struct timespec time_stamp = getTime();

  //mpu.update();
  //get_pos_vel_for_odom();
  //imu_pub();
  //nav_pub();
  //tf_pub();

  /*if (mpu.update()) {
  static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            get_pos_vel_for_odom();
            imu_pub();
            nav_pub();
            tf_pub();
            RCSOFTCHECK(rcl_publish(&publisher, tf_message, NULL));
            RCSOFTCHECK(rcl_publish(&publisher_i, imu, NULL));
            RCSOFTCHECK(rcl_publish(&publisher_nav, odometry, NULL));
            prev_ms = millis();
        }
    }

  */

  /*-------------------------DMP code-----------------*/
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  //if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
  (mpu.dmpGetCurrentFIFOPacket(fifoBuffer));
   mpu.dmpGetQuaternion(&q, fifoBuffer);
  //get_pos_vel_for_odom();
  imu_pub();
  nav_pub();
  //tf_pub();
  //RCSOFTCHECK(rcl_publish(&publisher, tf_message, NULL));
  RCSOFTCHECK(rcl_publish(&publisher_i, imu, NULL));
  RCSOFTCHECK(rcl_publish(&publisher_nav, odometry, NULL));
    
  //}
  /*------------------end of DMP code-----------------*/
  //RCSOFTCHECK(rcl_publish(&publisher, tf_message, NULL));
  //RCSOFTCHECK(rcl_publish(&publisher_i, imu, NULL));
  //RCSOFTCHECK(rcl_publish(&publisher_nav, odometry, NULL));
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
}
